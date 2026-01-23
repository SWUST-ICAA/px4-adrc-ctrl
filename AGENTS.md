# Repository Guidelines

## Project Structure

- `adrc_controller/include/`: Public headers for the ADRC controller (Simulink-generated API, e.g. `uav_adrc.h`).
- `adrc_controller/src/`: Implementation sources (generated C++).
- `ROS2-PX4代码编写注意事项.md`: ROS 2 ↔ PX4 integration notes and pitfalls.
- `LICENSE`, `README.md`: Project metadata.

This repository primarily contains generated controller code; it is typically built/linked by a parent PX4/ROS 2 workspace or another consumer project.

## Build, Test, and Development Commands

No standalone build system (no `CMakeLists.txt`/`package.xml`) is provided here. For a quick local compile check:

```bash
mkdir -p /tmp/px4-adrc-ctrl-build
g++ -std=c++17 -O2 -Iadrc_controller/include -c adrc_controller/src/uav_adrc.cpp -o /tmp/px4-adrc-ctrl-build/uav_adrc.o
g++ -std=c++17 -O2 -Iadrc_controller/include -c adrc_controller/src/uav_adrc_data.cpp -o /tmp/px4-adrc-ctrl-build/uav_adrc_data.o
ar rcs /tmp/px4-adrc-ctrl-build/libuav_adrc.a /tmp/px4-adrc-ctrl-build/uav_adrc.o /tmp/px4-adrc-ctrl-build/uav_adrc_data.o
```

To consume in another project, add:
- include path: `adrc_controller/include`
- sources: `adrc_controller/src/*.cpp`

## Coding Style & Naming Conventions

- Generated files: avoid manual reformatting; prefer changing the Simulink model and regenerating code.
- New hand-written C/C++ code (if added): use 2 spaces indentation, `snake_case` for files, `UpperCamelCase` for types, and `lower_snake_case` for functions/variables.

## Testing Guidelines

- No automated tests are currently included. At minimum, keep the “compile check” command working.
- If adding tests, place them under `tests/` and document how to run them (e.g., via CMake/CTest in the consuming workspace).

## Commit & Pull Request Guidelines

- Follow existing history: short, imperative commit messages (e.g., “Add …”, “Fix …”).
- PRs should include: what changed, why (controller/params/behavior), how it was verified (build command and/or flight/sim logs), and any PX4 topic/QoS implications.

## PX4/ROS 2 Integration Notes

- Coordinate frames: assume PX4 FRD (body) and NED (world); be explicit about transforms.
- ROS 2 subscriptions to PX4 topics: use compatible QoS (often `rclcpp::SensorDataQoS()`).
- If using `direct_actuator`, ensure `/fmu/in/vehicle_thrust_setpoint` is published to avoid land-detector disarm behavior.

## 每次修改以后，使用中文回答我，告诉我修改的内容