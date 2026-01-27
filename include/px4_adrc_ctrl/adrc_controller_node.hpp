// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "std_msgs/msg/bool.hpp"

#include "uav_adrc.h"

namespace px4_adrc_ctrl
{

class MotorAllocation;

class AdrcControllerNode final : public rclcpp::Node
{
public:
  AdrcControllerNode();
  ~AdrcControllerNode() override;

private:
  enum class MissionPhase
  {
    kWaitArmed,
    kWaitOffboard,
    kTakeoff,
    kTriggerTrajectory,
    kWaitTrajectory,
    kTrack
  };

  void on_timer();
  void maybe_request_offboard(const rclcpp::Time &now, bool armed, bool offboard, bool inputs_ok);

  void publish_offboard_mode(const rclcpp::Time &now);
  void publish_idle_outputs(const rclcpp::Time &now);
  void publish_nan_outputs(const rclcpp::Time &now);
  void publish_trajectory_trigger(const rclcpp::Time &now);

  // Parameters
  std::string odom_topic_;
  std::string status_topic_;
  std::string traj_topic_;
  std::string offboard_mode_topic_;
  std::string actuator_motors_topic_;
  std::string thrust_sp_topic_;
  std::string vehicle_command_topic_;
  std::string trajectory_trigger_topic_;

  double rate_hz_{};
  int odom_timeout_ms_{};
  int setpoint_timeout_ms_{};
  int status_timeout_ms_{};
  bool publish_idle_before_offboard_{};
  double idle_motor_throttle_{};

  bool auto_offboard_enabled_{};
  int offboard_warmup_ms_{};
  int offboard_cmd_period_ms_{};
  int target_system_{};
  int target_component_{};
  int source_system_{};
  int source_component_{};

  double takeoff_height_m_{};
  double takeoff_reached_tol_m_{};
  double takeoff_yaw_rad_{};
  int trigger_pulse_ms_{};
  int trigger_period_ms_{};

  double mass_kg_{};
  double gravity_mps2_{};
  double arm_length_m_{};
  double yaw_moment_coeff_Nm_per_N_{};
  double max_motor_thrust_N_{};
  std::array<double, 4> yaw_signs_{};

  std::unique_ptr<MotorAllocation> alloc_;
  std::unique_ptr<uav_adrc> adrc_pos_;
  std::unique_ptr<uav_adrc> adrc_att_;

  // ROS I/O
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_sub_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_sp_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_trigger_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Latest received data
  std::mutex mutex_;
  px4_msgs::msg::VehicleOdometry odom_{};
  px4_msgs::msg::TrajectorySetpoint traj_sp_{};
  px4_msgs::msg::VehicleStatus status_{};
  rclcpp::Time odom_rx_time_{};
  rclcpp::Time traj_rx_time_{};
  rclcpp::Time status_rx_time_{};
  bool have_odom_{false};
  bool have_traj_{false};
  bool have_status_{false};

  // Offboard request state
  std::optional<rclcpp::Time> offboard_warmup_start_time_{};
  std::optional<rclcpp::Time> last_offboard_cmd_time_{};

  // Mission state
  MissionPhase phase_{MissionPhase::kWaitArmed};
  bool takeoff_origin_valid_{false};
  std::array<double, 3> takeoff_origin_ned_{};
  std::optional<rclcpp::Time> trigger_start_time_{};
  std::optional<rclcpp::Time> last_trigger_pub_time_{};
};

}  // namespace px4_adrc_ctrl
