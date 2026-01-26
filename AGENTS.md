# Repository Guidelines

This repository contains an ADRC (Active Disturbance Rejection Control) controller intended to be integrated with PX4 via ROS 2.

## Project Structure & Module Organization

- `MBD_code/`: Simulink-generated controller code.
- `MBD_code/include/`: generated public headers (e.g., `uav_adrc.h`).
- `MBD_code/src/`: generated implementation (e.g., `uav_adrc.cpp`, `uav_adrc_data.cpp`).
- `src/`, `include/`: reserved for hand-written integration code (ROS 2 nodes, wrappers). Currently empty.
- `launch/`: ROS 2 launch files (planned).
- `config/`: runtime parameters (YAML) for nodes (planned).
- `ROS2-PX4代码编写注意事项.md`: PX4/ROS 2 integration gotchas and conventions used by this project.

## Build, Test, and Development Commands

- Compile check the generated controller (fast sanity check):
  - `g++ -std=c++17 -I MBD_code/include -c MBD_code/src/uav_adrc.cpp -o /tmp/uav_adrc.o`
  - `g++ -std=c++17 -I MBD_code/include -c MBD_code/src/uav_adrc_data.cpp -o /tmp/uav_adrc_data.o`
- If/when this repository is packaged as a ROS 2 package in a workspace:
  - `colcon build --packages-select px4-adrc-ctrl`
  - `colcon test --packages-select px4-adrc-ctrl`

## Coding Style & Naming Conventions

- Treat `MBD_code/**` as generated output: avoid manual edits; regenerate from Simulink instead and commit the full diff.
- For new C++ integration code: use C++17, keep formatting consistent (2-space indent), and prefer `snake_case` for files and functions, `UpperCamelCase` for types.
- Use Eigen for frame/matrix math (see `ROS2-PX4代码编写注意事项.md`).

## Testing Guidelines

- There are no repository-local automated tests yet; at minimum, run the compile check above for any `MBD_code` changes.
- For new ROS 2 code, add unit tests under `test/` and wire them into `colcon test` (e.g., `ament_cmake_gtest`).

## Commit & Pull Request Guidelines

- Commit messages follow short, imperative, sentence-case wording (examples in history: “Add…”, “Modify…”, “Regenerate…”).
- PRs should include: what changed, how to reproduce/regenerate (especially for `MBD_code`), and how it was validated (build logs, simulation results, plots).

## PX4/ROS 2 Integration Notes (Read Before Changing Interfaces)

- PX4 topics often require compatible subscriber QoS; prefer `rclcpp::SensorDataQoS()` when subscribing.
- PX4 uses FRD-NED conventions; be explicit about frame transforms.
- In `direct_actuator` workflows, ensure `vehicle_thrust_setpoint` is published to avoid land-detector disarm behavior.

