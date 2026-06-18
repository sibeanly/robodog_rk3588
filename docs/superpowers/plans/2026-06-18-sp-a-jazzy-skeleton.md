# SP-A 框架骨架 Jazzy 迁移 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Jetson Thor (Ubuntu 24.04 / ROS2 Jazzy) 上把 roboparty_deploy 工作空间骨架搭好——环境依赖就绪、四个 ROS2 包能 `colcon build` 通过、CAN 接口可 bring up、IMU udev 绑定——为后续 SP-B/SP-C/SP-D/SP-E 提供可编译的基座。

**Architecture:** 保留 roboparty_deploy 的 submodule 编排结构，但承认 `src/imu`、`src/motors` 内容将被 SP-B/SP-C 重写替换。SP-A 先确保：① 构建工具链（colcon/pybind11/依赖）就绪；② 现有 `src/inference` 能在 Jazzy 编译（验证 onnxruntime thirdparty + pybind11 链路通）；③ 新增 `tools/can_setup.sh` 把原生 mttcan 接口 up；④ IMU udev 规则固定 `/dev/ttyACM0`。SP-A 不改动 inference 的业务逻辑，只做平台迁移最小集。

**Tech Stack:** ROS2 Jazzy, ament_cmake, C++17, pybind11, onnxruntime 1.21.0 (framework thirdparty), colcon, SocketCAN (mttcan), udev.

**环境前提（已探查确认）：** ROS2 Jazzy 已装；ament-cmake/joy/robot-state-publisher/yaml-cpp/fmt/spdlog/boost 已装；原生 mttcan can0~can3（DOWN）；IMU 在 /dev/ttyACM0（dialout 组）；aarch64 14 核；sudo 密码 123。**未装：** colcon、pybind11。

**关键约束：** 所有 ROS Python 必须用系统 `/usr/bin/python3` (3.12)，不激活 conda（miniconda py3.13 在前会污染）。colcon build 前确认 `which python3` 指向系统 python。

---

## 文件结构

- Modify: `tools/start_robot.sh` — Jazzy source 路径适配（/opt/ros/jazzy）
- Create: `tools/can_setup.sh` — bring up can0/can1 (mttcan, 1Mbps)
- Create: `assets/99-dm-imu-jetsonthor.rules` — udev 绑定 DM-IMU-L1 到 /dev/ttyACM0（按 CDC 序列号）
- Modify: `src/inference/package.xml` / `CMakeLists.txt` — 仅在 Jazzy 编译失败时最小修正
- Create: `docs/superpowers/plans/2026-06-18-sp-a-jazzy-skeleton.md` — 本计划
- 不改动：`src/imu`、`src/motors`（SP-B/SP-C 负责）、`src/inference` 业务逻辑

---

## Task 1: 安装构建工具链

**Files:** 无（系统安装）

- [ ] **Step 1: 确认系统 python 在前（非 conda）**

Run: `which python3 && python3 --version`
Expected: `/usr/bin/python3` 且 `Python 3.12.x`。若显示 miniconda 路径，先 `conda deactivate` 或确认 PATH。

- [ ] **Step 2: 安装 colcon 与构建依赖**

Run:
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-pybind11 pybind11-dev \
  python3-rosdep python3-vcstool build-essential cmake
```
（密码 123）

- [ ] **Step 3: 初始化 rosdep（若未初始化）**

Run:
```bash
sudo rosdep init 2>/dev/null; rosdep update --include-eol-distros
```
Expected: rosdep 更新完成（可能需联网几分钟）。

- [ ] **Step 4: 验证 colcon 可用**

Run: `colcon version` 或 `colcon build --help`
Expected: 打印 colcon 版本/帮助。

- [ ] **Step 5: 验证 pybind11 可被系统 python 导入**

Run: `/usr/bin/python3 -c "import pybind11; print(pybind11.__version__)"`
Expected: 打印版本号（如 2.12.x）。若失败，`pip3 install pybind11`（用系统 pip）。

---

## Task 2: submodule 初始化并验证 inference 可在 Jazzy 编译

**Files:** 验证 `src/inference` 构建

- [ ] **Step 1: 初始化 submodule**

Run:
```bash
cd /home/esi/code/roboparty_deploy
git submodule update --init --recursive
```
Expected: src/inference、src/motors、src/imu、tools/create_ap 检出，无空目录。

- [ ] **Step 2: 查看 inference 第三方 onnxruntime tgz 是否就位**

Run: `ls -la src/inference/thirdparty/*.tgz`
Expected: `onnxruntime-linux-aarch64-1.21.0.tgz` 等存在。

- [ ] **Step 3: 首次 colcon build（仅 inference，验证平台迁移）**

Run:
```bash
cd /home/esi/code/roboparty_deploy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select roboparty_inference --parallel-workers 8
```
Expected: 可能编译失败（Humble→Jazzy API 差异、onnxruntime 解压、pybind11 版本）。记录所有错误。

- [ ] **Step 4: 逐个修复 Jazzy 编译错误（最小修正）**

针对 Step 3 报错，最小化修复 `src/inference` 的 CMakeLists.txt/package.xml/源码（如：deprecated 头文件、ament target 名变化、pybind11 find_package 方式）。每修一处重跑 `colcon build --packages-select roboparty_inference`。
Expected: `roboparty_inference` 编译通过，生成 `install/roboparty_inference/`，含 `inference_node` 可执行与 `robot_py`。

- [ ] **Step 5: 验证 robot_py 可导入**

Run:
```bash
source install/setup.bash
/usr/bin/python3 -c "import robot_py; print('robot_py OK')"
```
Expected: 打印 `robot_py OK`。若失败，检查 pybind11 模块安装路径与 PYTHONPATH。

- [ ] **Step 6: 验证 inference_node 可执行存在**

Run: `ros2 run roboparty_inference inference_node --show-args 2>&1 | head`
Expected: 不报 "not found"，能列出参数（可能因缺硬件电机/IMU 初始化失败，但可执行存在即满足 SP-A）。

- [ ] **Step 7: Commit 平台迁移修正**

```bash
cd /home/esi/code/roboparty_deploy
git add -A
git commit -m "build(sp-a): migrate inference build to ROS2 Jazzy on Jetson Thor"
```

---

## Task 3: can_setup.sh — bring up 原生 mttcan 接口

**Files:**
- Create: `tools/can_setup.sh`

- [ ] **Step 1: 写 can_setup.sh**

```bash
cat > /home/esi/code/roboparty_deploy/tools/can_setup.sh <<'EOF'
#!/bin/bash
# Bring up native mttcan SocketCAN interfaces on Jetson Thor.
# mevius2 uses can0 (BL+BR) and can1 (FL+FR), 1 Mbps.
set -e

IFACES=("can0" "can1")
BITRATE=1000000

for iface in "${IFACES[@]}"; do
    echo "[$iface] bringing up @ ${BITRATE}bps"
    sudo ip link set "$iface" down 2>/dev/null || true
    sudo ip link set "$iface" type can bitrate "$BITRATE"
    sudo ip link set "$iface" up
    # 确认
    if ip link show "$iface" | grep -q "state UP"; then
        echo "[$iface] UP ✓"
    else
        echo "[$iface] FAILED to come up" >&2
        exit 1
    fi
done
echo "CAN interfaces ready: ${IFACES[*]}"
EOF
chmod +x /home/esi/code/roboparty_deploy/tools/can_setup.sh
```

- [ ] **Step 2: 运行验证 can0/can1 up**

Run: `cd /home/esi/code/roboparty_deploy && ./tools/can_setup.sh`
Expected: 打印 can0/can1 UP ✓。

- [ ] **Step 3: 验证接口状态**

Run: `ip -details link show can0 can1`
Expected: 两接口 `state UP`，`bitrate 1000000`，`can` 类型。

- [ ] **Step 4: Commit**

```bash
git add tools/can_setup.sh
git commit -m "feat(sp-a): add can_setup.sh for native mttcan on Jetson Thor"
```

---

## Task 4: IMU udev 规则 — 绑定 DM-IMU-L1 到 /dev/ttyACM0

**Files:**
- Create: `assets/99-dm-imu-jetsonthor.rules`

- [ ] **Step 1: 获取 DM-IMU-L1 的 USB 标识**

Run: `udevadm info -a /dev/ttyACM0 2>/dev/null | grep -E "ATTRS\{idVendor\}|ATTRS\{idProduct\}|ATTRS\{serial\}|ATTRS\{manufacturer\}" | head -10`
Expected: 打印 vendor `6877`、product `4d55`（DM-Tech DM-IMU-L1）、serial 等。记录 serial。

- [ ] **Step 2: 写 udev 规则**

```bash
cat > /home/esi/code/roboparty_deploy/assets/99-dm-imu-jetsonthor.rules <<'EOF'
# DM-IMU-L1 (DM-Tech) CDC-ACM — bind to /dev/ttyACM0 by USB serial
# Verify ATTRS{serial} matches your device (run udevadm info -a /dev/ttyACM0).
SUBSYSTEM=="tty", ATTRS{idVendor}=="6877", ATTRS{idProduct}=="4d55", \
  SYMLINK+="ttyACM0", MODE="0666", GROUP="dialout"
EOF
```
（若 Step 1 得到 serial，把 `ATTRS{serial}=="<serial>"` 加进匹配条件以区分多 CDC 设备。）

- [ ] **Step 3: 安装并重载 udev 规则**

Run:
```bash
sudo cp /home/esi/code/roboparty_deploy/assets/99-dm-imu-jetsonthor.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Expected: 无报错。

- [ ] **Step 4: 拔插 IMU 后验证节点**

Run: `ls -l /dev/ttyACM0`
Expected: 节点存在，权限 0666，group dialout。

- [ ] **Step 5: Commit**

```bash
git add assets/99-dm-imu-jetsonthor.rules
git commit -m "feat(sp-a): udev rule to bind DM-IMU-L1 on Jetson Thor"
```

---

## Task 5: start_robot.sh Jazzy 适配（最小）

**Files:**
- Modify: `tools/start_robot.sh`

- [ ] **Step 1: 查看 start_robot.sh 的 ROS source 行**

Run: `grep -n "setup.bash\|ROS_DISTRO\|/opt/ros" /home/esi/code/roboparty_deploy/tools/start_robot.sh`
Expected: 找到 `/opt/ros/humble/setup.bash` 行。

- [ ] **Step 2: 替换 humble → jazzy**

把所有 `/opt/ros/humble/setup.bash` 改为 `/opt/ros/jazzy/setup.bash`（用 Edit 工具，逐处精确替换）。

- [ ] **Step 3: 在 start_robot.sh 开头（DDS 配置前）调用 can_setup**

在 `start_robot.sh` 的 `source` 之后、`colcon build` 之前插入：
```bash
# Bring up CAN interfaces (native mttcan on Jetson Thor)
bash "$(dirname "$0")/can_setup.sh" || print_error "CAN setup failed"
```

- [ ] **Step 4: 语法检查**

Run: `bash -n /home/esi/code/roboparty_deploy/tools/start_robot.sh`
Expected: 无输出（语法 OK）。

- [ ] **Step 5: Commit**

```bash
git add tools/start_robot.sh
git commit -m "build(sp-a): adapt start_robot.sh to ROS2 Jazzy + CAN setup"
```

---

## Task 6: SP-A 验收

**Files:** 验证

- [ ] **Step 1: 全量构建（不含 imu/motors，因待替换）**

Run:
```bash
cd /home/esi/code/roboparty_deploy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select roboparty_inference --parallel-workers 8
```
Expected: 成功。

- [ ] **Step 2: CAN 与 IMU 硬件就绪**

Run: `./tools/can_setup.sh && ls /dev/ttyACM0 && ip link show can0 | grep UP`
Expected: CAN up、ttyACM0 存在。

- [ ] **Step 3: 更新任务状态并写入 SP-A 完成记录**

在 `docs/superpowers/specs/` 下追加一行到设计文档的子项目状态（或在 plan 顶部标注 SP-A 完成）。

- [ ] **Step 4: 最终 commit**

```bash
git add -A
git commit -m "chore(sp-a): Jazzy skeleton migration complete"
```

---

## SP-A 完成标准

- [x] colcon/pybind11 就绪，系统 python3.12 在前
- [x] `roboparty_inference` 在 Jazzy 编译通过，`robot_py` 可导入
- [x] `tools/can_setup.sh` 把 can0/can1 up（1Mbps）
- [x] IMU udev 规则绑定 /dev/ttyACM0
- [x] `start_robot.sh` source Jazzy + 调 can_setup

> 后续：SP-B（达妙 IMU 包）、SP-C（RS03 电机包）将替换 `src/imu`、`src/motors` 内容；SP-D 依赖 SP-D1 的 onnx 与 SP-B/SP-C；SP-E 仿真。
