# DM-IMU-L1 C++ Driver Reuse + rviz Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reuse the ready-made DamiaoImuDriver from `jeston-main` into the `src/imu` submodule, build `imu_py`, and visualize the live DM-IMU-L1 in rviz via the existing `scripts/damiao_imu_*` prototype.

**Architecture:** Copy jeston-main's 6-file delta (damiao driver + factory + `IMUSerialPort::write()` + CMake) over public master `baab9a3d` in the `src/imu` submodule, commit it on a local submodule branch `feat/damiao-driver`, re-pin the superproject gitlink, `colcon build`, then run the existing hardware acceptance test and rviz launch. No new driver code is written — this is verbatim reuse of proven code, so TDD is adapted: the compile gate (colcon build) replaces unit-test-first for the copied code, and the existing `scripts/damiao_imu_test.py` is the behavioral acceptance gate.

**Tech Stack:** ROS2 Humble, ament_cmake, C++17, pybind11 (`imu_py`), spdlog, Fast DDS (`rmw_fastrtps_cpp`), rviz2 + `ros-humble-imu-tools`, DM-IMU-L1 USB CDC-ACM @ 921600 8N1.

## Global Constraints

- DM-IMU-L1 is physically connected at `/dev/ttyACM0` (idVendor 6877 / idProduct 4d55 / serial DMIMU20250212). Stable symlink target is `/dev/dm_imu` (set by udev rule).
- Serial config: 921600 baud, 8N1, USB CDC-ACM. Driver only supports `interface_type=="serial"`.
- `imu_py.IMUDriver.create_imu(imu_id=0, interface_type="serial", interface="/dev/dm_imu", imu_type="DAMIAO", baudrate=921600)` is the consumer contract — must remain unchanged.
- Output units contract (matches HiPNUC + `damiao_imu_test.py`): `get_quat()`→{w,x,y,z} unit quat (native EKF), `get_ang_vel()`→rad/s, `get_lin_acc()`→m/s² (|a|≈9.8 at rest), `get_temperature()`→0.0 (frame has no temp).
- `/imu` topic: `sensor_msgs/Imu`, frame_id `imu_link`, QoS RELIABLE/KeepLast(10)/Volatile (so rviz's default Reliable Imu display subscribes without QoS mismatch).
- HiPNUC path must stay intact: factory uses `else if (imu_type=="DAMIAO")` — the HIPNUC branch and existing tests are untouched.
- The `src/imu` submodule is a separate git repo at `.git/modules/imu`. Delta commits go there FIRST, then the superproject gitlink is re-pinned. Two-repo commit discipline.
- Real-hardware safety: IMU is read-only (no motor actuation). Safe to run anytime. But the RX thread sets `SCHED_FIFO` priority 80 — requires `rtprio`/`memlock` limits (verify `ulimit -r` → 98 per CLAUDE.md); if unavailable, port still works (priority set fails with a logged error but RX continues).
- This is real-hardware robotics code. The DM-IMU driver does NOT actuate motors — it only reads the IMU over serial. No motor/CAN/zero-offset changes in this plan.

---

## File Structure

**Submodule `src/imu/` (separate git repo `.git/modules/imu`):**

- Create `src/drivers/damiao/damiao_imu_driver.hpp` — `class DamiaoImuDriver : public IMUDriver` declaration.
- Create `src/drivers/damiao/damiao_imu_driver.cpp` — driver implementation (parse + CRC + config commands + getters).
- Create `src/drivers/damiao/CMakeLists.txt` — `add_library(damiao_imu STATIC ...)`, links `imu_protocol`.
- Modify `src/drivers/CMakeLists.txt` — add `add_subdirectory(damiao)`.
- Modify `src/imu_driver.cpp` — factory `else if (imu_type=="DAMIAO")` branch + include.
- Modify `src/protocol/serial/serial_port.hpp` — add `ssize_t write(...)` declaration.
- Modify `src/protocol/serial/serial_port.cpp` — add `write()` implementation.
- Modify `CMakeLists.txt` (top-level of submodule) — add `damiao_imu` to link/install/export (3 spots).

**Superproject (this repo, `feat/rk3588-humble-port`):**

- Re-pin `src/imu` gitlink (via `git add src/imu`).
- No other superproject source changes in this plan (scripts/assets already exist).

**Existing files reused as-is (do NOT modify):**

- `scripts/damiao_imu_node.py` — `/imu` publisher (200Hz, RELIABLE, frame_id imu_link).
- `scripts/damiao_imu_test.py` — hardware acceptance test (|q|~1, |a|~9.8, rad/s, 500-1000Hz).
- `scripts/damiao_imu_rviz.launch.py` — node + static tf + rviz2.
- `scripts/run_damiao_imu.sh` — node + static tf runner.
- `assets/damiao_imu.rviz` — rviz config (Fixed Frame world, rviz_imu_plugin/Imu on /imu Reliable).
- `assets/99-dm-imu-jetsonthor.rules` — udev rule → `/dev/dm_imu`.

---

### Task 1: Bring jeston-main delta into the src/imu submodule working tree

Copy the 6-file delta from `/home/orange5plus/code/robodog_jeston-main/src/imu/` into the submodule working tree at `src/imu/`. This makes the proven code present locally before any commit/build.

**Files:**
- Create: `src/imu/src/drivers/damiao/CMakeLists.txt`
- Create: `src/imu/src/drivers/damiao/damiao_imu_driver.hpp`
- Create: `src/imu/src/drivers/damiao/damiao_imu_driver.cpp`
- Modify: `src/imu/src/drivers/CMakeLists.txt`
- Modify: `src/imu/src/imu_driver.cpp`
- Modify: `src/imu/src/protocol/serial/serial_port.hpp`
- Modify: `src/imu/src/protocol/serial/serial_port.cpp`
- Modify: `src/imu/CMakeLists.txt`

**Interfaces:**
- Consumes: public master `baab9a3d` working tree (current state of `src/imu`).
- Produces: a working tree identical to jeston-main's `src/imu` (verified by `diff -rq`).

- [ ] **Step 1: Confirm submodule working tree is at public master and clean**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
GIT_DIR=.git/modules/imu git status --short
GIT_DIR=.git/modules/imu git rev-parse HEAD
```
Expected: HEAD = `baab9a3d4dd13901814a4741314ccb58bf156792`, empty status. If status is non-empty, stop and reconcile before proceeding.

- [ ] **Step 2: Copy the new damiao driver directory**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
mkdir -p src/imu/src/drivers/damiao
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/drivers/damiao/CMakeLists.txt src/imu/src/drivers/damiao/
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/drivers/damiao/damiao_imu_driver.hpp src/imu/src/drivers/damiao/
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/drivers/damiao/damiao_imu_driver.cpp src/imu/src/drivers/damiao/
```

- [ ] **Step 3: Copy the 4 modified files (overwrite)**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/drivers/CMakeLists.txt src/imu/src/drivers/CMakeLists.txt
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/imu_driver.cpp src/imu/src/imu_driver.cpp
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/protocol/serial/serial_port.hpp src/imu/src/protocol/serial/serial_port.hpp
cp /home/orange5plus/code/robodog_jeston-main/src/imu/src/protocol/serial/serial_port.cpp src/imu/src/protocol/serial/serial_port.cpp
cp /home/orange5plus/code/robodog_jeston-main/src/imu/CMakeLists.txt src/imu/CMakeLists.txt
```

- [ ] **Step 4: Verify the working tree now matches jeston-main exactly**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
diff -rq src/imu /home/orange5plus/code/robodog_jeston-main/src/imu 2>&1 | grep -v -E 'build|install|\.git|COLCON_IGNORE|__pycache__|\.deb'
```
Expected: no output (trees identical modulo build/install/git artifacts). If any `differ` or `Only in` lines remain for source files, re-copy the offending file.

- [ ] **Step 5: Confirm the factory and CMake wiring are present**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
grep -n 'DAMIAO' src/imu/src/imu_driver.cpp
grep -n 'damiao' src/imu/src/drivers/CMakeLists.txt
grep -n 'write' src/imu/src/protocol/serial/serial_port.hpp
grep -n 'damiao_imu' src/imu/CMakeLists.txt
```
Expected: factory shows `else if (imu_type == "DAMIAO")` + `#include "drivers/damiao/damiao_imu_driver.hpp"`; drivers CMake shows `add_subdirectory(damiao)`; serial_port.hpp shows `ssize_t write(const uint8_t* data, size_t length);`; top CMake shows `damiao_imu` in 3 places (target_link_libraries, install TARGETS, ament_export_libraries).

No commit yet — build verification (Task 3) happens before committing, so a broken copy is caught early.

---

### Task 2: Install runtime dependencies (udev + imu_tools) and verify device

The rviz IMU display needs `ros-humble-imu-tools`; the stable device path needs the udev rule. Verify both before building.

**Files:**
- Install (system): `assets/99-dm-imu-jetsonthor.rules` → `/etc/udev/rules.d/`
- Install (apt): `ros-humble-imu-tools`

**Interfaces:**
- Produces: `/dev/dm_imu` symlink exists; `rviz_imu_plugin/Imu` display class available.

- [ ] **Step 1: Install the udev rule and trigger**

Run:
```bash
sudo cp /home/orange5plus/code/robodog_jeston/assets/99-dm-imu-jetsonthor.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Expected: no error output.

- [ ] **Step 2: Verify the stable symlink exists**

Run:
```bash
ls -l /dev/dm_imu
```
Expected: `/dev/dm_imu -> ...` pointing at the ttyACM device (the DM-IMU-L1). If missing, run `udevadm info -a /dev/ttyACM0 | grep -i serial` and confirm `DMIMU20250212` matches the rule; if the device is on a different tty, the rule's KERNELS/port binding still matches by idVendor/idProduct/serial so the symlink should appear regardless of ttyACM number.

- [ ] **Step 3: Install the rviz IMU plugin and verify**

Run:
```bash
sudo apt update && sudo apt install -y ros-humble-imu-tools
```
Then verify the display class is registered:
```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix rviz_imu_plugin 2>/dev/null || dpkg -L ros-humble-imu-tools | grep -E 'rviz_imu_plugin'
```
Expected: a path under `/opt/ros/humble/` showing the `rviz_imu_plugin` library is installed.

- [ ] **Step 4: Verify rtprio privilege for the RX thread**

Run:
```bash
ulimit -r
```
Expected: `98` (per CLAUDE.md). If lower, the SCHED_FIFO set fails with a logged error but the driver still reads data — note this but do not block; recommend the user add rtprio/memlock in `/etc/security/limits.conf` per CLAUDE.md.

No commit (system changes only).

---

### Task 3: Build the workspace and verify imu_py exposes the DAMIAO factory

This is the compile gate. A clean `colcon build` proves the delta compiles and links. Then a Python import proves `imu_py` loads and `create_imu` accepts `imu_type="DAMIAO"`.

**Files:**
- Build outputs: `build/`, `install/` (gitignored).

**Interfaces:**
- Consumes: working-tree delta from Task 1.
- Produces: `install/setup.bash` providing `imu_py` with `DamiaoImuDriver` reachable via the factory.

- [ ] **Step 1: Build the workspace**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
Expected: `colcon` reports `0 packages failed`; `roboparty_imu` (the imu package) builds. If a compile error appears in `damiao_imu_driver.cpp` or `serial_port.cpp`, it means the copy in Task 1 was incomplete — re-run Task 1 Step 4 and rebuild.

- [ ] **Step 2: Source the install and verify imu_py imports**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "import imu_py; print('imu_py OK', dir(imu_py))"
```
Expected: prints `imu_py OK` and a list including `IMUDriver`.

- [ ] **Step 3: Verify the DAMIAO factory path exists without opening hardware**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "
import imu_py
# create_imu will try to open /dev/nonexistent and throw — that's fine;
# we only need to confirm DAMIAO is dispatched (not 'IMU type not supported').
try:
    imu_py.IMUDriver.create_imu(0, 'serial', '/dev/nonexistent', 'DAMIAO', 921600)
    print('UNEXPECTED: no exception')
except RuntimeError as e:
    msg = str(e)
    assert 'not supported' not in msg, f'factory rejected DAMIAO: {msg}'
    print('DAMIAO dispatched OK (failed at device open as expected):', msg)
"
```
Expected: prints `DAMIAO dispatched OK ...` with a message like "Failed to open serial port: /dev/nonexistent". This proves the factory's `else if (imu_type=="DAMIAO")` branch is wired (a non-DAMIAO/unknown type would print "IMU type not supported").

No commit yet — the submodule commit (Task 6) happens after hardware acceptance confirms correctness.

---

### Task 4: Hardware acceptance test (damiao_imu_test.py)

The behavioral gate. Confirms the driver reads live frames, outputs correct units (|q|~1, |a|~9.8 m/s², rad/s), and streams 500-1000Hz.

**Files:**
- Test: `scripts/damiao_imu_test.py` (existing, run as-is).

**Interfaces:**
- Consumes: `imu_py` from Task 3, `/dev/dm_imu` from Task 2.
- Produces: pass/fail verdict on unit correctness and stream rate.

- [ ] **Step 1: Run the acceptance test against live hardware**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/damiao_imu_test.py
```
Expected: the script prints samples and a "Unit verification" block showing:
- `|quat| = ~1.0` (within ±0.05)
- `|lin_acc| = ~9.8 m/s^2` (within ±1.0; if it's ~1.0, accel is in g and the driver's m/s² assumption is wrong — see Task 7 Risk note)
- `|ang_vel| max/median` small at rest (rad/s, not 60-180 deg/s)
- Frequency 500-1000 Hz

The script returns 0 on success, non-zero on failure.

- [ ] **Step 2: If |lin_acc| ≈ 1.0 instead of 9.8, the module outputs accel in g**

The reused driver stores accel as m/s² directly (assumes SI). If the test shows |a|≈1.0, the module is outputting g. This is a real-data finding, not a plan deviation — record the observed value. The fix (if needed) is out of scope for verbatim reuse but documented in Task 7; if |a|≈9.8, no action.

- [ ] **Step 3: Confirm quaternion tracks device orientation**

While the test runs (or in a separate run), manually tilt the IMU and re-run; confirm the printed `quat(w,x,y,z)` values change coherently with tilt (e.g., flipping the module 180° about one axis inverts the corresponding quaternion components). Expected: quat changes predictably with orientation, |q| stays ~1.

No commit (test is read-only).

---

### Task 5: rviz visualization verification

Confirm `/imu` publishes at ~200Hz and rviz renders the IMU orientation live.

**Files:**
- Run: `scripts/damiao_imu_rviz.launch.py` (existing) or `scripts/run_damiao_imu.sh` + `rviz2`.
- Config: `assets/damiao_imu.rviz` (existing).

**Interfaces:**
- Consumes: `imu_py` + `/dev/dm_imu` + `ros-humble-imu_tools`.
- Produces: `/imu` topic at ~200Hz; live IMU axes in rviz.

- [ ] **Step 1: Launch the IMU node + static tf + rviz**

Run (in one terminal):
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
export DISPLAY=${DISPLAY:-:1}
ros2 launch scripts/damiao_imu_rviz.launch.py
```
Expected: rviz2 window opens with Grid + IMU display; terminal logs `publishing /imu q(w,x,y,z)=...` from `damiao_imu_node`. The IMU display shows no "error subscribing" (QoS matches: node RELIABLE, rviz display Reliable).

- [ ] **Step 2: Verify /imu rate and fields in a second terminal**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /imu
ros2 topic echo /imu --once
```
Expected: `hz` shows ~200 Hz; `echo` shows a `sensor_msgs/Imu` with non-zero `orientation` (normalized quat), non-zero `angular_velocity` and `linear_acceleration`, `header.frame_id: imu_link`.

- [ ] **Step 3: Confirm live tracking in rviz**

Tilt the physical IMU while watching rviz. Expected: the IMU axes (red box/axis) rotate to match the device orientation in real time. If the axes don't move, check the rviz IMU display's Topic Reliability is `Reliable` and the Fixed Frame is `world` (static tf publishes world→imu_link).

No commit (runtime verification only).

---

### Task 6: Commit the submodule delta and re-pin the superproject gitlink

Two-repo commit discipline: commit inside the submodule first, then update the superproject's gitlink pointer.

**Files:**
- Commit (submodule repo `.git/modules/imu`): the 8 files from Task 1.
- Commit (superproject): `src/imu` gitlink update.

**Interfaces:**
- Consumes: verified-working delta (build + acceptance + rviz all pass from Tasks 3-5).
- Produces: a re-pinned `src/imu` gitlink on `feat/rk3588-humble-port`; submodule branch `feat/damiao-driver`.

- [ ] **Step 1: Inspect submodule changes**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston/src/imu
git status --short
```
Expected: the 8 changed/new files (3 new under `src/drivers/damiao/`, 5 modified: `src/drivers/CMakeLists.txt`, `src/imu_driver.cpp`, `src/protocol/serial/serial_port.{hpp,cpp}`, `CMakeLists.txt`).

- [ ] **Step 2: Create the submodule branch and commit**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston/src/imu
git checkout -b feat/damiao-driver
git add -A
git commit -m "feat(damiao): add DM-IMU-L1 driver backend + serial write()

Reuse DAMIAO driver from jeston-main: DamiaoImuDriver (native quaternion
RID 0x04 + accel + gyro, sliding-window parse, CRC16-CCITT, init config
commands), IMUSerialPort::write() it depends on, factory DAMIAO branch,
and CMake link/install/export wiring. HIPNUC path unchanged.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
git log --oneline -1
```
Expected: a new commit on `feat/damiao-driver`; HEAD shown.

- [ ] **Step 3: Re-pin the superproject gitlink**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
git add src/imu
git status --short
```
Expected: `src/imu` shows as modified (gitlink changed from `2cabb7de` to the new submodule commit).

- [ ] **Step 4: Commit the gitlink update in the superproject**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
git commit -m "feat(imu): re-pin src/imu submodule to feat/damiao-driver

Brings the DAMIAO DM-IMU-L1 C++ backend into the build so imu_py exposes
imu_type='DAMIAO'. Build + damiao_imu_test.py + rviz all verified on
live hardware (DM-IMU-L1 @ /dev/dm_imu, 921600).

Submodule commit is local-only for now (pinned 2cabb7de was unreachable
on the public remote). Follow-up: push feat/damiao-driver to a fork and
repoint .gitmodules url.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
git log --oneline -3
```
Expected: superproject HEAD advances with the gitlink update commit.

---

### Task 7: Regression check + follow-up documentation

Confirm HIPNUC path is untouched (no behavior change for non-DAMIAO), and record the local-only submodule caveat for reproducibility.

**Files:**
- Verify: factory HIPNUC branch intact; no unrelated submodule changes.

**Interfaces:**
- Produces: regression confirmation; follow-up note for fork push.

- [ ] **Step 1: Confirm HIPNUC factory branch is intact**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston/src/imu
git show HEAD:src/imu_driver.cpp | grep -A2 'imu_type == "HIPNUC"'
```
Expected: the HIPNUC branch is unchanged (`return std::make_shared<HipnucIMUDriver>(...)`). Only a new `else if (imu_type=="DAMIAO")` branch was added.

- [ ] **Step 2: Confirm no unrelated files changed in the submodule commit**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston/src/imu
git show --stat HEAD
```
Expected: exactly the 8 files from Task 1, nothing else (no build/install/log artifacts — they're gitignored).

- [ ] **Step 3: Record the accel-units follow-up note if applicable**

If Task 4 Step 2 found `|lin_acc| ≈ 1.0` (module outputs g, not m/s²), append a note to the spec's Risks section:
```bash
cd /home/orange5plus/code/robodog_jeston
# Only if accel was in g: edit docs/superpowers/specs/2026-06-22-imu-damiao-cpp-rviz-design.md
# Risks -> note observed accel in g, needs x9.8 in damiao_imu_driver.cpp parse_normal_subpacket
```
If `|lin_acc| ≈ 9.8`, skip this step (driver assumption correct).

- [ ] **Step 4: Note the local-only submodule follow-up in memory**

The submodule `feat/damiao-driver` commit exists only locally. Other clones cannot fetch it until it's pushed to a reachable remote and `.gitmodules` url is repointed. This is a known follow-up, not a blocker for this machine. (No file change required — already documented in the spec's "子模块重 pin 策略" section and the superproject commit message.)

---

## Self-Review

**Spec coverage:**
- "组件（复用 jeston-main delta）" 6-file delta → Task 1 (copy) + Task 6 (commit). ✓
- "环境与启动接线" udev + imu_tools + build → Task 2 (udev/imu_tools) + Task 3 (build). ✓
- "测试与验收" damiao_imu_test.py + topic hz/echo + rviz + HiPNUC regression → Task 4 (acceptance) + Task 5 (rviz) + Task 7 (regression). ✓
- "单位与坐标系" → verified empirically in Task 4 (|q|~1, |a|~9.8, rad/s). ✓
- "子模块重 pin 策略" → Task 6. ✓
- "数据流/话题/QoS" → Task 5 (200Hz, RELIABLE, frame_id). ✓

**Placeholder scan:** No TBD/TODO; all steps have exact commands and expected output. TDD adapted (verbatim-reused code → compile gate + hardware acceptance gate) and stated honestly in Architecture.

**Type consistency:** Factory signature `create_imu(imu_id, interface_type, interface, imu_type, baudrate)` consistent across Tasks 1/3. `/imu` QoS (RELIABLE/KeepLast(10)) consistent across spec and Task 5. `imu_type=="DAMIAO"` string consistent.
