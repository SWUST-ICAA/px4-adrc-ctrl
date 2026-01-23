# ROS2-PX4代码编写注意事项

## 1) Land Detector 与 `vehicle_thrust_setpoint`（direct_actuator 模式风险）
- PX4 的 land detector 会使用 `vehicle_thrust_setpoint` 来判断“低油门/落地”状态。
- 在 `direct_actuator` 模式下，如果不发布该 topic，PX4 可能长期认为油门 = 0，从而误判落地并自动上锁（disarm）。
- 结论：使用 `direct_actuator` 时，务必确保 `vehicle_thrust_setpoint` 以合理频率/数值被发布，避免被当作长期 0 油门。

## 2) 坐标系约定：FRD-NED
- PX4 内部坐标系约定为 FRD-NED：
  - 机体系（Body）：Forward（前）、Right（右）、Down（下）
  - 地理系（World）：North（北）、East（东）、Down（下）

## 2.1) 矩阵运算：使用 Eigen
- 涉及矩阵/向量/姿态（旋转）等运算时，优先使用 C++ 的 Eigen 库（例如 `Eigen::Vector3f`、`Eigen::Matrix3f`、`Eigen::Quaternionf`），避免手写矩阵运算导致的维度/坐标系错误。

## 2.2) PX4 四元数定义与向量变换方向（body->NED）
- 在 PX4 中，四元数（quaternion，例如 `vehicle_attitude` 消息中的 `q` 字段）表示从机体坐标系（body frame，通常为 FRD：Forward-Right-Down）到世界坐标系（NED：North-East-Down）的旋转。
- 用于将机体系向量变换到世界系：`v_ned = q * v_body * q^{-1}`（`*` 为四元数乘法；对单位四元数，`q^{-1}` 等于共轭）。
- 该惯例基于 Hamiltonian 四元数（标量在前：`w, x, y, z`），并在 PX4 的矩阵库与消息定义中一致实现。
- 若需要从世界系变回机体系，则使用四元数的逆（共轭）进行反向变换；在 ROS 2 集成/姿态估计等常见场景中，PX4 已按此定义提供姿态，通常无需额外“反转”，除非特定上下文要求。

## 3) ROS 2 订阅 PX4 话题必须设置兼容 QoS（发布通常不必）
- 用 ROS 2 订阅 PX4 发布的话题时，必须指定合适（兼容）的 QoS（服务质量）设置，才能监听这些话题。
- 需注意：在此场景下，ROS 代码在“发布”时通常无需设置 QoS 参数（PX4 的 QoS 设置与 ROS 的默认发布 QoS 设置兼容）。

### 3.1 不兼容原因：ROS 2 默认订阅者 QoS 与 PX4 发布者 QoS 不匹配
- 在 ROS 2 中订阅 PX4 发布的 uORB 话题时，默认的订阅者 QoS 配置文件与 PX4 的发布者 QoS 不兼容。
- 典型的 PX4 发布者 QoS：
  - Reliability：Best Effort
  - Durability：Transient Local
  - History：Keep Last（Depth = 0）
- ROS 2 默认订阅者 QoS：
  - Reliability：Reliable
  - Durability：Volatile
  - History：Keep Last（Depth = 10）
- 不兼容的主要原因是可靠性策略：Reliable 订阅者无法与 Best Effort 发布者匹配。

### 3.2 推荐做法：订阅时使用 `sensor_data` QoS（或等价配置）
要兼容 PX4 的 Best Effort 发布者，必须在订阅时指定合适的 QoS 配置文件。推荐使用 ROS 2 预定义的 `sensor_data` QoS，它包括：

- History：Keep Last
- Depth：5
- Reliability：Best Effort
- Durability：Volatile

这确保了订阅者可以接收 PX4 的 Best Effort 消息，而不会要求更高的可靠性。

示例（rclcpp）：

```cpp
auto sub = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
  "/fmu/out/vehicle_odometry",
  rclcpp::SensorDataQoS(),
  callback);
```

## 4) Quadrotor X：MAIN 输出与电机顺序/旋向
- 在 PX4 飞行控制软件中，“X”型四旋翼（Quadrotor X configuration）的电机顺序通常指 MAIN 输出引脚（MAIN1、MAIN2、MAIN3、MAIN4）连接到具体电机位置的对应关系。
- 该配置的标准电机编号从前右电机开始，顺时针方向依次分配。

### 4.1 电机编号（MAIN 映射）
- Motor 1（MAIN1）：前右（front-right）
- Motor 2（MAIN2）：后左（back-left）
- Motor 3（MAIN3）：前左（front-left）
- Motor 4（MAIN4）：后右（back-right）

### 4.2 螺旋桨旋向（propeller rotation）
- PX4 文档中：绿色表示顺时针（CW），蓝色表示逆时针（CCW）。
- 典型配置中，对角线上的电机旋转方向相同，以平衡扭矩：
  - Motor 1（前右）：CCW
  - Motor 2（后左）：CCW
  - Motor 3（前左）：CW
  - Motor 4（后右）：CW

## 5) 可用话题（ROS 2 <-> PX4）
以下是能够使用的话题：

### 5.1 `/fmu/in/*`
```text
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/arming_check_reply
/fmu/in/aux_global_position
/fmu/in/config_control_setpoints
/fmu/in/config_overrides_request
/fmu/in/distance_sensor
/fmu/in/goto_setpoint
/fmu/in/manual_control_input
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/unregister_ext_component
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_command_mode_executor
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_visual_odometry
```

### 5.2 `/fmu/out/*`
```text
/fmu/out/airspeed_validated
/fmu/out/arming_check_request
/fmu/out/battery_status
/fmu/out/collision_constraints
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/home_position
/fmu/out/manual_control_setpoint
/fmu/out/message_format_response
/fmu/out/mode_completed
/fmu/out/position_setpoint_triplet
/fmu/out/register_ext_component_reply
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_global_position
/fmu/out/vehicle_gps_position
/fmu/out/vehicle_land_detected
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status_v1
/fmu/out/vtol_vehicle_status
```
