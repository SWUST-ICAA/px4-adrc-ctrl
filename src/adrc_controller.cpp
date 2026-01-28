#include "px4_adrc_ctrl/adrc_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace px4_adrc_ctrl
{
namespace
{

/**
 * @brief Convert ROS time to PX4 timestamp in microseconds.
 * @param t ROS time.
 * @return Microseconds.
 */
uint64_t to_us(const rclcpp::Time &t)
{
  return static_cast<uint64_t>(t.nanoseconds() / 1000ULL);
}

/**
 * @brief Check if a float value is finite.
 * @param v Input float.
 * @return True if finite.
 */
bool is_finite(float v)
{
  return std::isfinite(static_cast<double>(v));
}

/**
 * @brief Convert PX4 quaternion (w,x,y,z) to Eigen quaternion (w,x,y,z).
 * @param q PX4 quaternion array.
 * @return Eigen quaternion.
 */
Eigen::Quaterniond quat_from_px4_wxyz(const std::array<float, 4> &q)
{
  return Eigen::Quaterniond(static_cast<double>(q[0]), static_cast<double>(q[1]),
                            static_cast<double>(q[2]), static_cast<double>(q[3]));
}

/**
 * @brief Ensure quaternion uses the shortest representation (w >= 0).
 * @param q Input quaternion.
 * @return Quaternion with non-negative scalar part.
 */
Eigen::Quaterniond ensure_shortest(const Eigen::Quaterniond &q)
{
  Eigen::Quaterniond out = q;
  if (out.w() < 0.0) {
    out.coeffs() *= -1.0;
  }
  return out;
}

/**
 * @brief Build desired body->world attitude from desired body-z direction and yaw.
 *
 * World: NED. Body: FRD. The returned quaternion follows PX4 convention (passive rotation body->world).
 *
 * @param b3_w Desired body-z axis expressed in world frame (unit or non-unit).
 * @param yaw_rad Desired yaw in radians.
 * @return Desired quaternion q_des (body->world).
 */
Eigen::Quaterniond attitude_from_b3_yaw(const Eigen::Vector3d &b3_w, double yaw_rad)
{
  const Eigen::Vector3d b3 = b3_w.normalized();

  // Desired heading direction in NED xy-plane.
  Eigen::Vector3d c1(std::cos(yaw_rad), std::sin(yaw_rad), 0.0);

  // If b3 is (almost) parallel to c1, pick an alternative heading axis.
  Eigen::Vector3d b2 = b3.cross(c1);
  if (b2.squaredNorm() < 1e-8) {
    c1 = Eigen::Vector3d(0.0, 1.0, 0.0);
    b2 = b3.cross(c1);
  }
  b2.normalize();
  Eigen::Vector3d b1 = b2.cross(b3);
  b1.normalize();

  Eigen::Matrix3d R_wb;
  R_wb.col(0) = b1;
  R_wb.col(1) = b2;
  R_wb.col(2) = b3;

  Eigen::Quaterniond q(R_wb);
  q.normalize();
  return q;
}

}  // namespace

/**
 * @brief Quadrotor X control allocation from wrench to motor thrusts.
 *
 * Motor order is fixed:
 *  - M1: front-right
 *  - M2: back-left
 *  - M3: front-left
 *  - M4: back-right
 */
class MotorAllocation
{
public:
  /**
   * @brief Construct allocation matrix for Quadrotor X.
   * @param arm_length_m Center-to-motor distance.
   * @param yaw_moment_coeff_Nm_per_N Yaw moment coefficient (Nm per N thrust).
   * @param yaw_signs Yaw signs for each motor (+1/-1).
   */
  MotorAllocation(double arm_length_m, double yaw_moment_coeff_Nm_per_N,
                  const std::array<double, 4> &yaw_signs)
  {
    const double d = arm_length_m / std::sqrt(2.0);
    A_.setZero();
    // T
    A_.row(0) << 1.0, 1.0, 1.0, 1.0;
    // tau_x = d*(f2 + f3 - f1 - f4)
    A_.row(1) << -d, +d, +d, -d;
    // tau_y = d*(f1 + f3 - f2 - f4)
    A_.row(2) << +d, -d, +d, -d;
    // tau_z = k * sum(sign_i * f_i)
    A_.row(3) << yaw_moment_coeff_Nm_per_N * yaw_signs[0],
      yaw_moment_coeff_Nm_per_N * yaw_signs[1],
      yaw_moment_coeff_Nm_per_N * yaw_signs[2],
      yaw_moment_coeff_Nm_per_N * yaw_signs[3];

    A_inv_ = A_.inverse();
  }

  /**
   * @brief Allocate motor thrusts and scale torques to satisfy per-motor bounds.
   *
   * This keeps the collective thrust fixed and scales the torque component only, which avoids
   * losing altitude control when saturation occurs.
   *
   * @param wrench [T, tau_x, tau_y, tau_z] where T is total thrust (N), taus are body torques (Nm).
   * @param f_min_N Minimum per-motor thrust.
   * @param f_max_N Maximum per-motor thrust.
   * @return Per-motor thrusts (N), size 4.
   */
  Eigen::Vector4d allocate_with_torque_scaling(const Eigen::Vector4d &wrench,
                                               double f_min_N, double f_max_N) const
  {
    const Eigen::Vector4d w_T(wrench[0], 0.0, 0.0, 0.0);
    const Eigen::Vector4d w_tau(0.0, wrench[1], wrench[2], wrench[3]);

    const Eigen::Vector4d f_T = A_inv_ * w_T;
    const Eigen::Vector4d f_tau = A_inv_ * w_tau;

    // Find s in [0, 1] such that f = f_T + s*f_tau stays within [f_min, f_max].
    double s_low = 0.0;
    double s_high = 1.0;
    for (int i = 0; i < 4; i++) {
      const double a = f_tau[i];
      const double b = f_T[i];
      if (std::abs(a) < 1e-12) {
        continue;
      }

      const double s1 = (f_min_N - b) / a;
      const double s2 = (f_max_N - b) / a;
      const double lo = std::min(s1, s2);
      const double hi = std::max(s1, s2);
      s_low = std::max(s_low, lo);
      s_high = std::min(s_high, hi);
    }

    const double s_low01 = std::max(0.0, s_low);
    const double s_high01 = std::min(1.0, s_high);
    const double s = (s_low01 <= s_high01) ? s_high01 : 0.0;

    Eigen::Vector4d f = f_T + s * f_tau;
    for (int i = 0; i < 4; i++) {
      f[i] = std::clamp(f[i], f_min_N, f_max_N);
    }
    return f;
  }

private:
  Eigen::Matrix4d A_{};
  Eigen::Matrix4d A_inv_{};
};

/**
 * @brief Construct the ADRC controller node.
 */
AdrcControllerNode::AdrcControllerNode() : rclcpp::Node("px4_adrc_controller")
{
  // Topics
  odom_topic_ = declare_parameter<std::string>("topics.vehicle_odometry", "/fmu/out/vehicle_odometry");
  status_topic_ = declare_parameter<std::string>("topics.vehicle_status", "/fmu/out/vehicle_status_v1");
  traj_topic_ = declare_parameter<std::string>("topics.trajectory_setpoint", "/adrc/trajectory_setpoint");
  offboard_mode_topic_ =
    declare_parameter<std::string>("topics.offboard_control_mode", "/fmu/in/offboard_control_mode");
  actuator_motors_topic_ = declare_parameter<std::string>("topics.actuator_motors", "/fmu/in/actuator_motors");
  thrust_sp_topic_ =
    declare_parameter<std::string>("topics.vehicle_thrust_setpoint", "/fmu/in/vehicle_thrust_setpoint");
  vehicle_command_topic_ = declare_parameter<std::string>("topics.vehicle_command", "/fmu/in/vehicle_command");
  trajectory_trigger_topic_ =
    declare_parameter<std::string>("topics.trajectory_start_trigger", "/adrc/trajectory_start");

  // Timing / behavior
  rate_hz_ = declare_parameter<double>("control.rate_hz", 1000.0);
  odom_timeout_ms_ = declare_parameter<int>("control.odom_timeout_ms", 200);
  setpoint_timeout_ms_ = declare_parameter<int>("control.setpoint_timeout_ms", 200);

  // Automatic Offboard switch (arming is manual).
  auto_offboard_enabled_ = declare_parameter<bool>("offboard.auto_switch", true);
  offboard_warmup_ms_ = declare_parameter<int>("offboard.warmup_ms", 800);
  offboard_cmd_period_ms_ = declare_parameter<int>("offboard.cmd_period_ms", 200);
  target_system_ = declare_parameter<int>("offboard.target_system", 1);
  target_component_ = declare_parameter<int>("offboard.target_component", 1);
  source_system_ = declare_parameter<int>("offboard.source_system", 1);
  source_component_ = declare_parameter<int>("offboard.source_component", 1);

  // Takeoff / trigger
  takeoff_height_m_ = declare_parameter<double>("takeoff.height_m", 1.0);
  takeoff_reached_tol_m_ = declare_parameter<double>("takeoff.reached_tol_m", 0.1);
  takeoff_yaw_rad_ = declare_parameter<double>("takeoff.yaw_rad", 0.0);
  trigger_pulse_ms_ = declare_parameter<int>("trigger.pulse_ms", 500);
  trigger_period_ms_ = declare_parameter<int>("trigger.period_ms", 100);

  // Vehicle / allocation
  mass_kg_ = declare_parameter<double>("vehicle.mass_kg", 1.5);
  gravity_mps2_ = declare_parameter<double>("vehicle.gravity_mps2", 9.80665);
  arm_length_m_ = declare_parameter<double>("vehicle.arm_length_m", 0.17);
  yaw_moment_coeff_Nm_per_N_ = declare_parameter<double>("vehicle.yaw_moment_coeff_Nm_per_N", 0.01);
  max_motor_thrust_N_ = declare_parameter<double>("vehicle.max_motor_thrust_N", 6.0);

  // PX4 standard Quadrotor X: M1/M2 CCW, M3/M4 CW (viewed from above).
  // With PX4 NED/FRD conventions, positive yaw torque corresponds to CCW motors increasing thrust.
  const std::array<double, 4> yaw_signs_px4 = {+1.0, +1.0, -1.0, -1.0};
  alloc_ = std::make_unique<MotorAllocation>(arm_length_m_, yaw_moment_coeff_Nm_per_N_, yaw_signs_px4);

  // Two independent MBD instances: position (accel) and attitude (torque).
  adrc_pos_ = std::make_unique<uav_adrc>();
  adrc_att_ = std::make_unique<uav_adrc>();

  // Subscriptions
  odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mutex_);
      odom_ = *msg;
      odom_rx_time_ = this->get_clock()->now();
      have_odom_ = true;
    });

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    status_topic_, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mutex_);
      status_ = *msg;
      status_rx_time_ = this->get_clock()->now();
      have_status_ = true;
    });

  traj_sub_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    traj_topic_, rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mutex_);
      traj_sp_ = *msg;
      traj_rx_time_ = this->get_clock()->now();
      have_traj_ = true;
    });

  // Publishers
  offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_mode_topic_, 10);
  motors_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>(actuator_motors_topic_, 10);
  thrust_sp_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(thrust_sp_topic_, 10);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic_, 10);
  trajectory_trigger_pub_ = create_publisher<std_msgs::msg::Bool>(trajectory_trigger_topic_, 10);

  // Control loop
  const auto period_ns = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
  timer_ = create_wall_timer(period_ns, [this]() { this->on_timer(); });
}

/**
 * @brief Destructor. Defined out-of-line to keep MotorAllocation incomplete in the header.
 */
AdrcControllerNode::~AdrcControllerNode() = default;

/**
 * @brief Timer callback: publish offboard mode, request OFFBOARD if needed, and run control loop.
 */
void AdrcControllerNode::on_timer()
{
  const rclcpp::Time now = get_clock()->now();

  // Snapshot inputs.
  px4_msgs::msg::VehicleOdometry odom{};
  px4_msgs::msg::TrajectorySetpoint traj{};
  px4_msgs::msg::VehicleStatus status{};
  rclcpp::Time odom_rx_time{};
  rclcpp::Time traj_rx_time{};
  rclcpp::Time status_rx_time{};
  bool have_odom = false;
  bool have_traj = false;
  bool have_status = false;

  {
    std::lock_guard<std::mutex> lk(mutex_);
    odom = odom_;
    traj = traj_sp_;
    status = status_;
    odom_rx_time = odom_rx_time_;
    traj_rx_time = traj_rx_time_;
    status_rx_time = status_rx_time_;
    have_odom = have_odom_;
    have_traj = have_traj_;
    have_status = have_status_;
  }

  const bool odom_ok = have_odom && ((now - odom_rx_time).nanoseconds() / 1000000LL <= odom_timeout_ms_);
  const bool sp_ok = have_traj && ((now - traj_rx_time).nanoseconds() / 1000000LL <= setpoint_timeout_ms_);
  // Use last known arming/nav state even if status is momentarily stale to keep offboard stream stable.
  const bool armed = have_status && (status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  const bool offboard = have_status && (status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
  const bool inputs_ok_for_offboard = odom_ok;  // before trajectory, only odom is required

  publish_offboard_mode(now, armed);

  // Mission phase transitions.
  if (!armed) {
    phase_ = MissionPhase::kWaitArmed;
    takeoff_origin_valid_ = false;
    trigger_start_time_.reset();
    last_trigger_pub_time_.reset();
  }
  if (phase_ == MissionPhase::kWaitArmed && armed) {
    phase_ = MissionPhase::kWaitOffboard;
  }
  if (phase_ == MissionPhase::kWaitOffboard && armed && offboard) {
    phase_ = MissionPhase::kTakeoff;
    takeoff_origin_valid_ = false;
  }
  if ((phase_ == MissionPhase::kTakeoff || phase_ == MissionPhase::kTriggerTrajectory ||
       phase_ == MissionPhase::kWaitTrajectory || phase_ == MissionPhase::kTrack) &&
      armed && !offboard) {
    // If OFFBOARD is lost, go back to waiting for OFFBOARD.
    phase_ = MissionPhase::kWaitOffboard;
    takeoff_origin_valid_ = false;
    trigger_start_time_.reset();
    last_trigger_pub_time_.reset();
  }

  // Automatic OFFBOARD switch once armed and odom is valid.
  maybe_request_offboard(now, armed, offboard, inputs_ok_for_offboard);

  // Before OFFBOARD, only publish idle outputs (do not try to control).
  if (!armed || !offboard || !odom_ok) {
    // Automatic safe behavior:
    // - Disarmed: publish zero motor setpoints (stop/disarmed).
    // - Armed but not in OFFBOARD yet: publish zero motor setpoints (keeps streaming inputs for OFFBOARD).
    if (armed) {
      publish_idle_outputs(now);
    } else {
      publish_nan_outputs(now);
    }
    return;
  }

  if (odom.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "VehicleOdometry.pose_frame is %u (expected NED); outputting idle",
                         static_cast<unsigned>(odom.pose_frame));
    publish_idle_outputs(now);
    return;
  }

  // Generate effective reference setpoint depending on mission phase.
  px4_msgs::msg::TrajectorySetpoint eff_sp{};
  if (phase_ == MissionPhase::kTrack) {
    // Require external trajectory during tracking; if missing, fall back to hover at current.
    eff_sp = traj;
    if (!sp_ok) {
      eff_sp.position[0] = odom.position[0];
      eff_sp.position[1] = odom.position[1];
      eff_sp.position[2] = odom.position[2];
      eff_sp.yaw = static_cast<float>(takeoff_yaw_rad_);
    }
  } else {
    // Hold XY at takeoff origin, and command climb to target altitude.
    if (!takeoff_origin_valid_) {
      takeoff_origin_ned_[0] = static_cast<double>(odom.position[0]);
      takeoff_origin_ned_[1] = static_cast<double>(odom.position[1]);
      takeoff_origin_ned_[2] = static_cast<double>(odom.position[2]);
      takeoff_origin_valid_ = true;
    }
    eff_sp.position[0] = static_cast<float>(takeoff_origin_ned_[0]);
    eff_sp.position[1] = static_cast<float>(takeoff_origin_ned_[1]);
    eff_sp.position[2] = static_cast<float>(takeoff_origin_ned_[2] - takeoff_height_m_);
    eff_sp.yaw = static_cast<float>(takeoff_yaw_rad_);
  }

  // Takeoff completion check and trigger publishing.
  if (phase_ == MissionPhase::kTakeoff) {
    const double z0 = takeoff_origin_ned_[2];
    const double z = static_cast<double>(odom.position[2]);
    const double altitude_up_m = z0 - z;  // NED: z down, so up is -z
    if (altitude_up_m >= (takeoff_height_m_ - takeoff_reached_tol_m_)) {
      phase_ = MissionPhase::kTriggerTrajectory;
      trigger_start_time_ = now;
      last_trigger_pub_time_.reset();
    }
  }
  if (phase_ == MissionPhase::kTriggerTrajectory) {
    publish_trajectory_trigger(now);
    if (trigger_start_time_) {
      const int64_t elapsed_ms = (now - *trigger_start_time_).nanoseconds() / 1000000LL;
      if (elapsed_ms >= trigger_pulse_ms_) {
        phase_ = MissionPhase::kWaitTrajectory;
      }
    }
  }
  if (phase_ == MissionPhase::kWaitTrajectory) {
    // Keep hovering until we receive fresh trajectory data, then start tracking.
    if (sp_ok && trigger_start_time_ && (traj_rx_time > *trigger_start_time_)) {
      phase_ = MissionPhase::kTrack;
    }
  }

  // Position ref/state (TrajectorySetpoint is NED). If ref is NaN, hold current.
  const Eigen::Vector3d p_ref_ned(
    is_finite(eff_sp.position[0]) ? static_cast<double>(eff_sp.position[0]) : static_cast<double>(odom.position[0]),
    is_finite(eff_sp.position[1]) ? static_cast<double>(eff_sp.position[1]) : static_cast<double>(odom.position[1]),
    is_finite(eff_sp.position[2]) ? static_cast<double>(eff_sp.position[2]) : static_cast<double>(odom.position[2]));
  const Eigen::Vector3d p_ned(static_cast<double>(odom.position[0]), static_cast<double>(odom.position[1]),
                              static_cast<double>(odom.position[2]));

  // 1) Position ADRC -> accel command (NED).
  uav_adrc::ExtU_uav_adrc_T in_pos{};
  in_pos.XPostionRef = p_ref_ned.x();
  in_pos.YPostionRef = p_ref_ned.y();
  in_pos.ZPostionRef = p_ref_ned.z();
  in_pos.XPostionState = p_ned.x();
  in_pos.YPostionState = p_ned.y();
  in_pos.ZPostionState = p_ned.z();
  in_pos.q_ex = 0.0;
  in_pos.q_ey = 0.0;
  in_pos.q_ez = 0.0;

  adrc_pos_->setExternalInputs(&in_pos);
  adrc_pos_->step();
  const auto out_pos = adrc_pos_->getExternalOutputs();
  const Eigen::Vector3d a_cmd_ned(out_pos.XAcceleration, out_pos.YAcceleration, out_pos.ZAcceleration);

  // 2) Desired attitude from thrust vector (accel command + yaw).
  const double yaw_des = is_finite(eff_sp.yaw) ? static_cast<double>(eff_sp.yaw) : 0.0;
  const Eigen::Vector3d g_ned(0.0, 0.0, gravity_mps2_);
  const Eigen::Vector3d f_des_ned = mass_kg_ * (a_cmd_ned - g_ned);
  const double T_N = f_des_ned.norm();
  const Eigen::Vector3d b3_des_w = (T_N > 1e-6) ? (-f_des_ned / T_N) : Eigen::Vector3d(0.0, 0.0, 1.0);
  const Eigen::Quaterniond q_des = attitude_from_b3_yaw(b3_des_w, yaw_des);

  // Current attitude (body->world).
  const Eigen::Quaterniond q_curr =
    quat_from_px4_wxyz({odom.q[0], odom.q[1], odom.q[2], odom.q[3]}).normalized();

  // Error quaternion: q_err = q_des^{-1} ⊗ q_curr, vector part -> MBD.
  const Eigen::Quaterniond q_err = ensure_shortest(q_des.conjugate() * q_curr);

  // 3) Attitude ADRC -> body torque command.
  uav_adrc::ExtU_uav_adrc_T in_att = in_pos;
  in_att.q_ex = q_err.x();
  in_att.q_ey = q_err.y();
  in_att.q_ez = q_err.z();

  adrc_att_->setExternalInputs(&in_att);
  adrc_att_->step();
  const auto out_att = adrc_att_->getExternalOutputs();
  const Eigen::Vector3d tau_body_Nm(out_att.BodyXTorque, out_att.BodyYTorque, out_att.BodyZTorque);

  // 4) Allocation -> normalized motor commands.
  const Eigen::Vector4d wrench(T_N, tau_body_Nm.x(), tau_body_Nm.y(), tau_body_Nm.z());
  const Eigen::Vector4d f_N = alloc_->allocate_with_torque_scaling(wrench, 0.0, max_motor_thrust_N_);

  // Publish ActuatorMotors (unused channels set to 0.0).
  px4_msgs::msg::ActuatorMotors motors_msg{};
  motors_msg.timestamp = to_us(now);
  motors_msg.timestamp_sample = motors_msg.timestamp;
  motors_msg.reversible_flags = 0;
  for (int i = 0; i < px4_msgs::msg::ActuatorMotors::NUM_CONTROLS; i++) {
    motors_msg.control[i] = 0.0f;
  }
  for (int i = 0; i < 4; i++) {
    const double u = std::clamp(f_N[i] / max_motor_thrust_N_, 0.0, 1.0);
    motors_msg.control[i] = static_cast<float>(u);
  }
  motors_pub_->publish(motors_msg);

  // Publish thrust setpoint to avoid land detector issues in direct_actuator mode.
  const double f_total_max_N = 4.0 * max_motor_thrust_N_;
  const double collective_norm = std::clamp(T_N / f_total_max_N, 0.0, 1.0);

  px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
  thrust_msg.timestamp = motors_msg.timestamp;
  thrust_msg.timestamp_sample = motors_msg.timestamp_sample;
  thrust_msg.xyz[0] = 0.0f;
  thrust_msg.xyz[1] = 0.0f;
  thrust_msg.xyz[2] = static_cast<float>(-collective_norm);  // body z is down; upward thrust is negative z
  thrust_sp_pub_->publish(thrust_msg);
}

/**
 * @brief Request OFFBOARD via VEHICLE_CMD_DO_SET_MODE (arming remains manual).
 * @param now Current time.
 * @param armed True if vehicle is armed.
 * @param offboard True if already in offboard.
 * @param inputs_ok True if odometry and setpoint inputs are fresh.
 */
void AdrcControllerNode::maybe_request_offboard(const rclcpp::Time &now, bool armed, bool offboard, bool inputs_ok)
{
  if (!auto_offboard_enabled_) {
    offboard_warmup_start_time_.reset();
    last_offboard_cmd_time_.reset();
    return;
  }

  if (!(armed && inputs_ok) || offboard) {
    offboard_warmup_start_time_.reset();
    last_offboard_cmd_time_.reset();
    return;
  }

  if (!offboard_warmup_start_time_) {
    offboard_warmup_start_time_ = now;
    last_offboard_cmd_time_.reset();
  }

  const int64_t warmup_elapsed_ms = (now - *offboard_warmup_start_time_).nanoseconds() / 1000000LL;
  if (warmup_elapsed_ms < offboard_warmup_ms_) {
    return;
  }

  if (last_offboard_cmd_time_) {
    const int64_t since_last_cmd_ms = (now - *last_offboard_cmd_time_).nanoseconds() / 1000000LL;
    if (since_last_cmd_ms < offboard_cmd_period_ms_) {
      return;
    }
  }

  // PX4 custom main mode for OFFBOARD is 6.
  constexpr float kMavModeFlagCustomEnabled = 1.0f;
  constexpr float kPx4CustomMainModeOffboard = 6.0f;

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = to_us(now);
  cmd.param1 = kMavModeFlagCustomEnabled;
  cmd.param2 = kPx4CustomMainModeOffboard;
  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  cmd.target_system = static_cast<uint8_t>(std::clamp(target_system_, 0, 255));
  cmd.target_component = static_cast<uint8_t>(std::clamp(target_component_, 0, 255));
  cmd.source_system = static_cast<uint8_t>(std::clamp(source_system_, 0, 255));
  cmd.source_component = static_cast<uint16_t>(std::clamp(source_component_, 0, 65535));
  cmd.from_external = true;
  cmd.confirmation = 0;
  vehicle_command_pub_->publish(cmd);

  last_offboard_cmd_time_ = now;
}

/**
 * @brief Publish OffboardControlMode, enabling direct_actuator only when armed.
 * @param now Current time.
 */
void AdrcControllerNode::publish_offboard_mode(const rclcpp::Time &now, bool armed)
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = to_us(now);
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = armed;
  offboard_mode_pub_->publish(msg);
}

/**
 * @brief Publish idle motor commands (safe zeros) plus VehicleThrustSetpoint.
 * @param now Current time.
 */
void AdrcControllerNode::publish_idle_outputs(const rclcpp::Time &now)
{
  px4_msgs::msg::ActuatorMotors motors_msg{};
  motors_msg.timestamp = to_us(now);
  motors_msg.timestamp_sample = motors_msg.timestamp;
  motors_msg.reversible_flags = 0;
  for (int i = 0; i < px4_msgs::msg::ActuatorMotors::NUM_CONTROLS; i++) {
    motors_msg.control[i] = 0.0f;
  }
  for (int i = 0; i < 4; i++) {
    motors_msg.control[i] = 0.0f;
  }
  motors_pub_->publish(motors_msg);

  px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
  thrust_msg.timestamp = motors_msg.timestamp;
  thrust_msg.timestamp_sample = motors_msg.timestamp_sample;
  thrust_msg.xyz[0] = 0.0f;
  thrust_msg.xyz[1] = 0.0f;
  thrust_msg.xyz[2] = 0.0f;
  thrust_sp_pub_->publish(thrust_msg);
}

/**
 * @brief Publish zero motor commands (disarm/stop) without thrust setpoint.
 * @param now Current time.
 */
void AdrcControllerNode::publish_nan_outputs(const rclcpp::Time &now)
{
  px4_msgs::msg::ActuatorMotors motors_msg{};
  motors_msg.timestamp = to_us(now);
  motors_msg.timestamp_sample = motors_msg.timestamp;
  motors_msg.reversible_flags = 0;
  for (int i = 0; i < px4_msgs::msg::ActuatorMotors::NUM_CONTROLS; i++) {
    motors_msg.control[i] = 0.0f;
  }
  motors_pub_->publish(motors_msg);

  // Keep publishing thrust setpoint in direct_actuator setups to avoid land detector edge cases.
  px4_msgs::msg::VehicleThrustSetpoint thrust_msg{};
  thrust_msg.timestamp = motors_msg.timestamp;
  thrust_msg.timestamp_sample = motors_msg.timestamp_sample;
  thrust_msg.xyz[0] = 0.0f;
  thrust_msg.xyz[1] = 0.0f;
  thrust_msg.xyz[2] = 0.0f;
  thrust_sp_pub_->publish(thrust_msg);
}

/**
 * @brief Publish a one-shot (pulsed) trajectory start trigger.
 * @param now Current time.
 */
void AdrcControllerNode::publish_trajectory_trigger(const rclcpp::Time &now)
{
  if (trigger_period_ms_ <= 0) {
    return;
  }
  if (last_trigger_pub_time_) {
    const int64_t since_last_ms = (now - *last_trigger_pub_time_).nanoseconds() / 1000000LL;
    if (since_last_ms < trigger_period_ms_) {
      return;
    }
  }

  std_msgs::msg::Bool msg{};
  msg.data = true;
  trajectory_trigger_pub_->publish(msg);
  last_trigger_pub_time_ = now;
}

}  // namespace px4_adrc_ctrl
