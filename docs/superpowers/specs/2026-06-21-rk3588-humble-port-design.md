# 设计文档：robodog_jeston 适配 RK3588 / Ubuntu 22.04 / ROS2 Humble（两阶段规划）

> 日期：2026-06-21
> 状态：待审阅
> 关联：需求 `2026-06-21-rk3588-humble-port-requirements.md`、计划 `2026-06-21-rk3588-humble-port-plan.md`（writing-plans 生成）

## 1. 现状与迁移结论

本仓库从原始 Humble/RK3588 真机基线（`~/code/roboto_origin/modules/roboparty_deploy`）迁移到 Jetson Thor（U24/Jazzy/py3.12/原生 mttcan），并在此期间加入 `sim/` MuJoCo 仿真栈与 mevius2 工具链。现迁回本开发机 RK3588/Humble/py3.10。经多路并行 gap-analysis（代码、Humble 基线、平台差异、mevius2 模型源）合并得出结论：

**纯 Python 仿真路径在 Humble/RK3588 上完全可行，且不需要任何 C++ colcon 构建或子模块初始化。真机 C++ 全链作为 Phase 2 后续阶段。**

依据：
- `sim/mujoco_bridge.py`、`sim/sim_inference_node.py`、`sim/launch/mujoco_sim.launch.py` 仅依赖 `rclpy` + 标准消息类型（apt 提供）+ `mujoco`/`onnxruntime`/`numpy`（pip 提供）
- 使用的 rclpy 子集（`Node`、`QoSProfile(depth, ReliabilityPolicy, DurabilityPolicy)`、`Publisher/Subscription`、`Timer`）在 Humble 与 Jazzy 间无 API 差异
- 全部 Python 代码无 3.12 专有语法（无 match/case、`X|None`、tomllib），3.10 直接可运行
- DDS profile（`assets/rt_fastdds_profile.xml`）与 Humble 基线逐字节一致，`rmw_fastrtps_cpp` 为 Humble 默认 RMW
- 模型源已明确：`policy_mevius2.onnx` 可从 `/home/orange5plus/code/mevius2/models/policy.pt`（TorchScript，34→12，4 层 ELU MLP）用既有 `tools/export_policy_onnx.py` 导出，无需 dummy 回退

因此 Phase 1 工作量集中在 **环境补全**、**ONNX 导出**、**路径/发行版修正** 与 **obs 复刻核对**，而非代码移植。

## 2. 两阶段划分

| | Phase 1（本轮） | Phase 2（后续） |
|---|---|---|
| 目标 | 纯 Python 仿真链 RK3588/Humble 跑通 + 离线测试 | 真机 C++ 全链 |
| 硬件 | 无真机 | 真机电机/CAN/IMU |
| 阻塞项 | 无 | 未推送子模块分支；真硬件 |
| 开工 | 立即 | 不开工，仅记录 |

本文档其余章节描述 Phase 1；Phase 2 见第 7 节。

## 3. 架构（Phase 1 仿真链）

```
              ┌───────────────────────────┐
              │  /usr/bin/python3 (3.10)  │  ← 系统 python == ROS python
              │  source /opt/ros/humble    │
              └───────────────────────────┘
   ┌────────────────────────┐        ┌──────────────────────────┐
   │ mujoco_bridge.py        │        │ sim_inference_node.py     │
   │ node: mujoco_sim_node   │        │ node: sim_inference_node  │
   │ 200Hz PD (kp50/kd2)     │        │ 50Hz policy (decim 4)     │
   │ load scene.xml+meshes   │        │ load policy_mevius2.onnx  │
   │                         │        │ obs(34) → action(12)      │
   │ pub /joint_states  BE   │───────▶│ sub /joint_states  BE     │
   │ pub /imu           BE   │───────▶│ sub /imu           BE     │
   │ sub /joint_targets BE  ◀────────│ pub /joint_targets BE     │
   │ pub /mujoco/base_pose   │        │                           │
   └────────────────────────┘        └──────────────────────────┘
        (无 C++ 子模块 / 无 imu_py / 无 motors_py / 无 CAN / 无真机)
```

两个节点经 `/joint_states`、`/imu`、`/joint_targets` 三个 BEST_EFFORT/VOLATILE 话题闭环。`sim_inference_node.py` 是 C++ `inference_node` 的 Python 等价替代，完全规避 C++ 子模块。

**模型语义对齐**：sim_inference_node 的 obs 构造、`dof_sym_sign`、ACTION_SCALE、DEFAULT_ANGLE 必须与 mevius2 原始 `mevius2_utils.py`/`parameters.py` 逐项一致，否则模型被喂 OOD 输入产生垃圾动作。Phase 1 包含对此的核对与必要修正（第 5 节 FR-5 / 测试 T5）。

## 4. 关键适配点

### 4.1 环境（Phase 1，修正现有 spec 过时假设）

> 修正：原 spec §3.1 假设 `/opt/ros/humble` 未装，实际**已装**。下表为真实现状。

| 项 | 现状 | 动作 |
|---|---|---|
| ROS2 Humble | **已装** `/opt/ros/humble` | 无需装；`source` 后 `import rclpy` 应可用（验收确认） |
| pip | 未装 | `apt install python3-pip` |
| numpy | 1.21.5 已有 | 不动 |
| mujoco | 未装 | `pip install --user mujoco==3.9.0` |
| onnxruntime | 未装 | `pip install --user onnxruntime==1.21.0` |
| joy/colcon | 待确认 | 若 `/opt/ros/humble` 不含则补装 `ros-humble-joy`、`python3-colcon-common-extensions` |
| 代理 | apt 未配 | `/etc/apt/apt.conf.d/95proxy` + pip env |
| torch/onnx（仅导出用） | 未装 | `pip install --user torch onnx`（一次性，走代理） |

**版本固定理由**：onnxruntime aarch64 cp310 wheel 存在至 1.23.0，**不存在** 1.27.0（Jazzy 测试报告用的版本）；1.21.0 与 `src/inference/thirdparty/onnxruntime-linux-aarch64-1.21.0.tgz` 对齐。mujoco aarch64 cp310 wheel 3.3.0–3.9.0 可用，固定 3.9.0。torch aarch64 cp310 CPU wheel 存在，仅供离线导出。

### 4.2 ONNX 导出（Phase 1 子流程）

主机无 torch/onnx/docker/conda，需先 `pip install --user torch onnx`，再用既有 `tools/export_policy_onnx.py`：

```bash
python3 tools/export_policy_onnx.py \
  --pt /home/orange5plus/code/mevius2/models/policy.pt \
  --out src/inference/models/policy_mevius2.onnx
```

工具内部：`torch.jit.load` → `torch.onnx.export`（opset 17，dynamic batch axis 0，input "obs"/output "action"）→ `onnx.checker.check_model` → pt-vs-onnx onnxruntime CPU 数值对齐（`np.allclose atol=1e-5 rtol=1e-4`），失败退出 1。导出完成后 torch/onnx 可留可删。

模型语义（sim_inference_node 必须复刻）：obs=34（base_ang_vel 3 + projected_gravity 3 + command 3 + dof_pos-default 12 + dof_vel 12 + is_standing 1，base_lin_vel 注释掉）；action=12；`dof_sym_sign=[1,1,1,-1,1,1,1,1,1,-1,1,1]` 作用于 dof_pos/dof_vel 输入 AND 输出；obs_scales lin_vel=2.0/ang_vel=0.25/dof_pos=1.0/dof_vel=0.05；ACTION_SCALE=0.2；ref = ACTION_SCALE*action + DEFAULT_ANGLE `[0,0.7,-1.2]×4`；joint order `BL,BR,FL,FR × (collar,hip,knee)`。

### 4.3 路径/发行版修正

| 文件:行 | 现 | 改 |
|---|---|---|
| `tools/start_robot.sh:146-147` | `/opt/ros/jazzy` | `/opt/ros/humble` |
| `scripts/run_damiao_imu.sh:5` | `cd /home/esi/code/roboparty_deploy` | `cd "$(dirname "$0")/.."` |
| `scripts/run_damiao_imu.sh:10` | `/opt/ros/jazzy` | `/opt/ros/humble` |
| `scripts/run_damiao_imu.sh:11` | `install_sp_d` | `install` |
| `scripts/damiao_imu_rviz.launch.py:5-6` | `jazzy` + `install_sp_d` | `humble` + `install` |
| `scripts/damiao_imu_node.py:10` | `install_sp_d` | `install` |
| `tools/export_policy_onnx.py:35` | `DEFAULT_PT=/home/esi/.../policy.pt` | 本机 `/home/orange5plus/code/mevius2/models/policy.pt` 或仓库相对 |
| `tools/export_policy_onnx.py:36` | `DEFAULT_OUT=/home/esi/.../policy_mevius2.onnx` | 仓库相对 `src/inference/models/policy_mevius2.onnx` |
| `sim/launch/mujoco_sim.launch.py:7` | 注释 `jazzy` | `humble` |
| `sim/mujoco_bridge.py:7` | 注释 `/home/esi/code/mevius2-master` | `/home/orange5plus/code/mevius2` |

### 4.4 清理

删除 `build_sp_b/c/d`、`install_sp_b/c/d`（Jazzy/py3.12 残留，`install_sp_d/setup.bash` 链 `/opt/ros/jazzy` + py3.12 hooks，py3.10 不可用且污染树）。`install/`（无 `_sp_`）目前不存在——仿真主路径不需要它，本轮不创建；上面脚本改 `install/` 仅为路径正确化，这些真机脚本本轮不执行。

### 4.5 不改动（明确排除）

- `sim/*.py`、`scripts/*.py` 业务逻辑（obs 复刻核对修正除外）
- `assets/rt_fastdds_profile.xml`（与基线一致）
- `tools/can_setup.sh`、udev 规则、RT 内核、`robot.yaml`（Phase 2）
- 子模块内容（仿真路径不依赖；不运行 `submodule update`）

## 5. 测试设计（Phase 1，离线无真机）

| 用例 | 验证 | 期望 | 依赖 |
|---|---|---|---|
| T1 import 冒烟 | `source humble` 后 `import rclpy,sensor_msgs,std_msgs,geometry_msgs,mujoco,ort,numpy` | 全成功，版本正确（mujoco 3.9.0 / ort 1.21.0） | FR-1 |
| T2 路径 grep | 仓库内（排除 docs 历史、.git、build*、install*）`jazzy`/`/home/esi`/`install_sp_d` | 干净 | FR-3/4 |
| T3 残留目录 | `build_sp_*`/`install_sp_*` | 不存在 | FR-4 |
| T4 ONNX 导出 | 跑 `export_policy_onnx.py` | 生成 onnx，pt-vs-onnx allclose 通过，in(1,34)/out(1,12) | FR-2 |
| T5 obs 复刻核对 | 比对 `sim_inference_node.py` 与 mevius2 `mevius2_utils.py`/`parameters.py` | obs_scales、dof_sym_sign（输入+输出）、ACTION_SCALE、DEFAULT_ANGLE、joint order 全一致；不一致则修正 | FR-5 |
| T6 bridge 无头 | `mujoco_bridge.py --no-viewer` | `/mujoco_sim_node` 上线 | T1 |
| T7 joint_states | `ros2 topic hz/echo /joint_states` | ~200Hz，12 关节名近 STANDBY | T6 |
| T8 imu | `ros2 topic hz /imu` | ~200Hz | T6 |
| T9 推理节点 | `sim_inference_node.py` | 加载 ONNX（打印 in/out 形状），`/sim_inference_node` 上线 | T4+T6 |
| T10 joint_targets | `ros2 topic hz/echo /joint_targets` | ~50Hz，12 维 | T9 |
| T11 接线 | `ros2 topic info -v` 三话题 | 各 1 pub + 1 sub，QoS BE/VOLATILE | T6+T9 |
| T12 真步态闭环 | bridge + inference 同跑，观察 `/joint_states` 轨迹 | 机器人保持站立/轻微步态（真模型，非零 action） | T4–T11 |

测试产物：测试报告文档（命名带 `2026-06-21`），记录每项结果、命令、输出摘录。T12 若发散则回 T5 复查 obs 复刻。

## 6. 与现有 spec 的修正点

- 原 §3.1 "ROS2 无 /opt/ros" → 错，**Humble 已装**
- 原 §3.1 "mujoco/onnxruntime/numpy 均未装" → numpy 1.21.5 已有
- 原 §3.4 模型来源风险 → 已明确从 mevius2 `policy.pt` 导出真模型，不再需 dummy 回退
- 原 design 引用 `2026-06-21-rk3588-humble-port-plan.md` 但不存在 → 本轮 writing-plans 生成

## 7. Phase 2 边界（记录，不开工）

Phase 1 完成后真机 C++ 全链作为后续阶段，本轮仅记录依赖与验收，不写任何真机代码。

**工作项**：① 推送 `feat/sp-d-mevius2` 子模块分支；② Humble/py3.10 colcon 重建三子模块（imu_py/motors_py/robot_py）；③ 重写 `can_setup.sh` 为 RK3588 USB-CAN（gs_usb）；④ mevius2 12 关节 RS03 `robot.yaml`；⑤ RT 内核 + limits.conf rtprio/memlock；⑥ udev 规则 KERNELS 核对；⑦ 真机 IMU 直测 + 电机闭环。

**阻塞项**：`feat/sp-d-mevius2` 分支未推送（colcon 重建前提）；真硬件在场（真机测试前提）。

**验收（待开工细化）**：colcon 产出 imu_py/motors_py/robot_py；`robot_py.RobotInterface` 加载 mevius2 robot.yaml 读 IMU、初始化电机；`start_robot.sh` 真机启动 inference + joy session；真机 IMU ~500Hz、12 关节可控、`/set_zeros`/`/start_inference` 可用。

**衔接**：Phase 1 产出的 `policy_mevius2.onnx`、路径修正、清理结果 Phase 2 直接复用；Phase 1 仿真闭环作为真机上线前回归基线。
