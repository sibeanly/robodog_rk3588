# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

`roboparty_deploy` is the ROS2 Humble deployment framework for Roboparty RPO/Roboto humanoid robots. **This repository itself contains almost no source code** — it is an integration/deploy wrapper around four git submodules that hold the real packages:

- `src/inference` → `roboparty_inference` (policy inference node, configs, launch)
- `src/motors` → `roboparty_motors` (Damiao CAN motor driver, C++ + `motors_py` bindings)
- `src/imu` → `roboparty_imu` (HiPNUC IMU driver, C++ + `imu_py` bindings)
- `tools/create_ap` → WiFi access-point tooling

Always work with submodules initialized:
```bash
git submodule update --init --recursive
```
The `src/*` directories will be empty otherwise, and `colcon build` will find nothing to compile.

Target hardware: Orange Pi 5 Plus (Ubuntu 22.04, kernel 5.10 RT) and RDK X5 (Ubuntu 22.04, kernel 6.1.83). C++17. Built and run on aarch64 and x86_64 Linux.

## Build & run

Build the workspace (from repo root):
```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Run everything (builds, then launches inference + joy in background `screen` sessions):
```bash
./tools/start_robot.sh
```
`start_robot.sh` is the canonical entry point — it sources ROS2, runs `colcon build --symlink-install`, sets up Fast DDS (`rmw_fastrtps_cpp` + `assets/rt_fastdds_profile.xml`, shared-memory transport), then starts `inference_session` and `joy_session` screens. Inspect with `screen -r inference_session` / `screen -r joy_session`; stop with `screen -S <name> -X quit`.

Launch inference alone (no rebuild/scripts):
```bash
ros2 launch roboparty_inference inference.launch.py
```

There is no test suite in this wrapper repo; tests, if any, live in the submodule packages.

## Switching policy models

The active policy is selected by editing which config file `src/inference/launch/inference.launch.py` loads (the `configs = [...]` list pointing at `config/<name>.yaml`). Available configs include `inference.yaml` (default), `inference_amp.yaml`, `inference_attn_enc.yaml`, `inference_beyondmimic.yaml`, `inference_getup.yaml`, `inference_interrupt.yaml`. After editing, re-run `./tools/start_robot.sh`. There is no CLI flag for this — it is config-file-driven.

## Robot control surface

Motors are controlled via ROS2 services (`std_srvs/srv/Trigger`) or the gamepad. Key services: `/init_motors`, `/deinit_motors`, `/start_inference`, `/stop_inference`, `/clear_errors`, `/set_zeros`, `/reset_joints`, `/refresh_joints`, `/read_joints`, `/read_imu`. Example:
```bash
ros2 service call /init_motors std_srvs/srv/Trigger
```

Gamepad (`joy_node`): X = init/deinit motors, A = reset motors, B = start/pause inference, Y = gamepad/cmd_vel toggle, LB = switch policy mode (beyondmimic/interrupt), RB = switch motion sequence (beyondmimic), right stick = move, LT/RT = turn.

## Python SDK (`imu_py`, `motors_py`, `robot_py`)

These modules are **generated at build time** from the submodule packages — they do not exist until `colcon build` + `source install/setup.bash` has run. `robot_py.RobotInterface(config_file)` loads the full robot (motors + IMU) from a YAML config; `motors_py`/`imu_py` expose individual drivers. See `scripts/` for working examples.

Scripts of note:
- `scripts/set_zero.py` — manual per-motor zero calibration (interactive, uses `scripts/config/set_zero.yaml`). Puts each motor in damping mode; Enter writes zero, Space skips.
- `scripts/motion_player.py` — replays recorded motion `.npz` files via `robot_py`; supports USD→URDF joint-order remapping.
- `scripts/{imu,motors}_py_example.py` — SDK usage examples.

## Configuration that matters

- `src/inference/config/robot.yaml` — central robot config: motor IDs, CAN interface mapping, IMU baudrate/frequency (default `921600` / `500Hz`, must be >200Hz), and `motor_zero_offset`. The waist-yaw offset entry (`2.093` vs `0.0`) depends on the calibration method used — do not change blindly.
- CAN mapping by default: `can0`=left leg, `can1`=right leg+waist, `can2`=left hand, `can3`=right hand (order of USB-to-CAN insertion without udev rules).
- `assets/*.deb` — Orange Pi 5 Plus 5.10 RT kernel packages (RDK X5 uses a pre-flashed RT image instead; do not install these there).
- `assets/99-auto-up-devs-{orangepi,sunrise}.rules` — udev rules binding physical USB ports to CAN/IMU devices so insertion order doesn't matter. `KERNELS` values must match the actual wiring.
- `assets/rt_fastdds_profile.xml` — Fast DDS shared-memory profile loaded by `start_robot.sh`.

## Safety & environment constraints

- Real-time priorities require `rtprio`/`memlock` limits set in `/etc/security/limits.conf` for the runtime user (verify with `ulimit -r` → `98`).
- Motor zeroing is a one-time setup; re-run only after motor service/replacement/zero-loss. Two paths: the `/set_zeros` service (motors initialized, inference stopped, robot held at target pose) or `scripts/set_zero.py` (manual, per-motor).
- This is real-hardware robotics code — motor commands actuate physical joints. Review changes that touch motor control, zero offsets, or CAN mapping against the hardware before running.
