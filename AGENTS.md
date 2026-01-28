# Repository Guidelines

This repository contains an ADRC controller implementation, primarily as Simulink-generated (ERT) C/C++ code intended to be integrated into a PX4/ROS 2 workspace.

## Project Structure & Module Organization

- `MBD_code/include/`: public headers generated from the Simulink model (e.g., `uav_adrc.h`).
- `MBD_code/src/`: generated implementation files (e.g., `uav_adrc.cpp`, `uav_adrc_data.cpp`).
- `include/`, `src/`: reserved for hand-written wrappers/integration code (currently minimal/empty).
- `config/`, `launch/`: reserved for runtime configuration and launch files (currently empty).
- `ROS2-PX4代码编写注意事项.md`: PX4/ROS 2 integration notes (QoS, frames, topic usage).

## Build, Test, and Development Commands

There is no standalone build system committed in this repo. Use one of the following checks:

```bash
# Quick compile sanity check for the generated code
g++ -std=c++17 -I MBD_code/include -c MBD_code/src/uav_adrc.cpp
g++ -std=c++17 -I MBD_code/include -c MBD_code/src/uav_adrc_data.cpp
```

When used inside a larger PX4/ROS 2 workspace, build with that workspace’s normal tooling (for example, `colcon build`).

## Coding Style & Naming Conventions

- Do not hand-edit `MBD_code/**` unless you are intentionally patching generated output; prefer changing the Simulink model and regenerating.
- Keep formatting consistent with existing generated files (2-space indents, no re-wrapping large comment blocks).
- New code (if added under `src/`/`include/`) should use clear, descriptive names and avoid abbreviations that collide with PX4/ROS concepts.

## Testing Guidelines

- No automated test suite is included yet. At minimum, ensure the generated sources compile cleanly and validate behavior in the consuming PX4/ROS 2 simulation or hardware setup.
- If you add hand-written logic, add a small, reproducible test harness (and document how to run it).

## PX4/ROS 2 Integration Notes

- PX4 uses FRD (body) and NED (world) conventions; avoid implicit frame flips.
- When subscribing to PX4 topics from ROS 2, set a compatible QoS (often `rclcpp::SensorDataQoS()`); see `ROS2-PX4代码编写注意事项.md`.

## Commit & Pull Request Guidelines

- Commit messages in history are short and verb-led (e.g., “update MBD code”, “Modify …”). Follow that pattern; keep subject lines concise.
- PRs should explain whether changes are regenerated output vs. hand edits, and (if regenerated) include the Simulink model/version and regeneration settings used.

## 使用中文回答我，并且每次告诉我你做了什么

## 编译请你注意位置，当前是功能包内，不要在这里编译