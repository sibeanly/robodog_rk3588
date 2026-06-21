# 需求文档：robodog_jeston 适配 RK3588 / Ubuntu 22.04 / ROS2 Humble（两阶段规划）

> 日期：2026-06-21
> 状态：待审阅
> 关联：设计文档 `2026-06-21-rk3588-humble-port-design.md`、计划 `2026-06-21-rk3588-humble-port-plan.md`（writing-plans 生成）
> 背景：robodog_jeston 从原始 Humble/RK3588 真机基线（`~/code/roboto_origin/modules/roboparty_deploy`）迁移到 Jetson Thor（Ubuntu 24.04 / ROS2 Jazzy / Python 3.12 / 原生 mttcan）并加入 `sim/` MuJoCo 仿真栈与 mevius2 工具链，现迁回本开发机：RK3588（Orange Pi 5 Plus 类）/ Ubuntu 22.04 / kernel 5.10.0-1012-rockchip / aarch64 / Python 3.10.12，目标 ROS 发行版为 **Humble**。

## 0. 两阶段划分

| | Phase 1（本轮） | Phase 2（后续） |
|---|---|---|
| 目标 | 纯 Python 仿真链在 RK3588/Humble 跑通 + 离线测试 + 文档 | 真机 C++ 全链 |
| 内容 | ONNX 导出、pip 依赖、路径/发行版修正、清理 build/install 残留、mujoco bridge + sim_inference 闭环离线验证 | colcon 重建三子模块（imu_py/motors_py/robot_py）、RK3588 USB-CAN can_setup、mevius2 robot.yaml、RT 内核/limits.conf、真机 IMU 直测 + 电机闭环 |
| 硬件 | 无真机 | 真机电机/CAN/IMU |
| 阻塞项 | 无（已全部分解） | 未推送的 `feat/sp-d-mevius2` 子模块分支；真硬件在场 |
| 开工 | 立即 | 本轮不开工，仅记录依赖与验收 |

**本文档需求以 Phase 1 为主，Phase 2 仅在第 7 节给出待开工项、阻塞项与验收（不细化，不开发）。**

## 1. Phase 1 项目目标

让本仓库已提交的纯 Python 仿真链（`sim/mujoco_bridge.py` + `sim/sim_inference_node.py` + `sim/launch/mujoco_sim.launch.py`）在本机 RK3588/Humble 上可直接运行，完成 mujoco 物理桥接、策略推理节点、话题/服务接线的离线验证；并从 `/home/orange5plus/code/mevius2/models/policy.pt` 导出真 `policy_mevius2.onnx`，实现真步态闭环。真机 C++ 驱动链（imu_py/motors_py/robot_py、USB-CAN、RT 内核、mevius2 robot.yaml）作为 Phase 2，本轮不涉及。

## 2. Phase 1 范围

### 2.1 在范围内

- ONNX 导出：从 mevius2 原始 TorchScript `policy.pt` 用 `tools/export_policy_onnx.py` 导出真 `policy_mevius2.onnx`（pt-vs-onnx 数值对齐）
- 环境搭建：本机补装 pip、mujoco 3.9.0、onnxruntime 1.21.0（Humble 已装、numpy 1.21.5 已有）；ONNX 导出临时装 torch+onnx
- 路径/发行版适配：把已提交脚本中硬编码的 `/opt/ros/jazzy`、`/home/esi/...`、`install_sp_d` 改为 `/opt/ros/humble` 与仓库相对路径
- 清理 Jetson 残留的 Jazzy/py3.12 构建产物（build_sp_* / install_sp_*）
- obs 复刻核对：比对 `sim_inference_node.py` 与 mevius2 `mevius2_utils.py`/`parameters.py`，确保 obs_scales、dof_sym_sign、ACTION_SCALE、DEFAULT_ANGLE、joint order 一致；不一致则修正
- 仿真离线测试：mujoco bridge 烟雾测试（无头 200Hz PD、12 关节、/imu）、sim_inference_node 结构测试、QoS/话题接线检查、真步态闭环观察
- 文档：需求、设计、开发计划、测试报告

### 2.2 不在范围内（Phase 2）

- 真机电机闭环、CAN bring-up、IMU 真机直测、RT 内核安装
- C++ 子模块（roboparty_inference/roboparty_imu/roboparty_motors）在 Humble/py3.10 上的 colcon 重建（imu_py/motors_py/robot_py）
- 推送/获取未提交的 `feat/sp-d-mevius2` 子模块分支
- mevius2 robot.yaml（12 关节 RS03 拓扑）重配、set_zero/motion_player 配置重生成
- Gazebo Harmonic 集成、多模态感知、训练新策略

## 3. Phase 1 功能需求

### FR-1 环境与依赖（安装）

- FR-1.1 确认 `/opt/ros/humble` 已存在；`source /opt/ros/humble/setup.bash` 后 `ros2` 可用，`python3 -c "import rclpy, sensor_msgs, std_msgs, geometry_msgs"` 全部通过（系统 python 3.10 即 ROS python）
- FR-1.2 `apt install python3-pip`（sudo 密码 1234）；若 `/opt/ros/humble` 不含 joy/colcon 则补装 `ros-humble-joy`、`python3-colcon-common-extensions`
- FR-1.3 pip `--user` 安装 `onnxruntime==1.21.0`、`mujoco==3.9.0`，可 import 且版本正确
- FR-1.4 代理 `http://127.0.0.1:7897` 用于 apt（`/etc/apt/apt.conf.d/95proxy`）与 pip（`http_proxy/https_proxy`）
- FR-1.5 ONNX 导出：临时 `pip install --user torch onnx`（aarch64 cp310 CPU wheel，走代理），导出后可保留或删除（不影响 ROS 运行）

### FR-2 ONNX 导出

- FR-2.1 用 `tools/export_policy_onnx.py` 从 `/home/orange5plus/code/mevius2/models/policy.pt` 导出到 `src/inference/models/policy_mevius2.onnx`
- FR-2.2 工具内部 pt-vs-onnx 数值对齐（`np.allclose atol=1e-5 rtol=1e-4`）通过；输入 `(1,34)`、输出 `(1,12)` float32
- FR-2.3 `tools/export_policy_onnx.py` 的 `DEFAULT_PT`/`DEFAULT_OUT` 默认路径修正为仓库相对 / 本机正确路径（不再 `/home/esi/...`）

### FR-3 路径与发行版适配

- FR-3.1 `tools/start_robot.sh:146-147`：`/opt/ros/jazzy` → `/opt/ros/humble`（含错误回显路径）
- FR-3.2 `scripts/run_damiao_imu.sh:5,10,11`：`/home/esi/...` → 仓库相对；`jazzy` → `humble`；`install_sp_d` → `install`（此脚本本轮不执行，但路径必须正确）
- FR-3.3 `scripts/damiao_imu_rviz.launch.py:5-6`、`scripts/damiao_imu_node.py:10`：`jazzy`/`install_sp_d` → `humble`/`install`
- FR-3.4 `tools/export_policy_onnx.py:35-36`：`/home/esi/...` 默认改为仓库相对 / 本机正确路径
- FR-3.5 注释性修正（`mujoco_sim.launch.py:7`、`mujoco_bridge.py:7`）：`jazzy` → `humble`、`/home/esi/code/mevius2-master` → `/home/orange5plus/code/mevius2`

### FR-4 清理与就绪

- FR-4.1 删除 `build_sp_b/c/d`、`install_sp_b/c/d`（Jazzy/py3.12 残留）
- FR-4.2 确保 `src/inference/models/policy_mevius2.onnx` 存在（FR-2 导出）
- FR-4.3 不运行 `git submodule update --init`（仿真路径无需子模块；当前 origin/master 检出足以放置 onnx）

### FR-5 obs 复刻核对

- FR-5.1 比对 `sim/sim_inference_node.py` 与 mevius2 `scripts/mevius2_utils.py`、`scripts/parameters.py`：obs_scales（lin_vel=2.0/ang_vel=0.25/dof_pos=1.0/dof_vel=0.05）、`dof_sym_sign=[1,1,1,-1,1,1,1,1,1,-1,1,1]`（作用于 dof_pos/dof_vel 输入 AND policy 输出）、ACTION_SCALE=0.2、DEFAULT_ANGLE=`[0,0.7,-1.2]×4`、joint order `BL,BR,FL,FR × (collar,hip,knee)`
- FR-5.2 若 `sim_inference_node.py` 与 mevius2 任一项不一致则修正至一致（属"让仿真真跑通"的必要部分，非业务重写）

### FR-6 仿真离线测试

- FR-6.1 mujoco bridge 无头运行：`/mujoco_sim_node` 上线，`/joint_states` ~200Hz 且 12 关节名 `[BL,BR,FL,FR]×[collar,hip,knee]` 近 STANDBY，`/imu` ~200Hz（BEST_EFFORT）
- FR-6.2 sim_inference_node 运行：启动加载 ONNX（打印 in/out 形状），`/sim_inference_node` 上线，以 50Hz 发布 `/joint_targets`（12 维）
- FR-6.3 接线验证：`/joint_states`、`/joint_targets`、`/imu` 各有 1 pub + 1 sub；QoS `BEST_EFFORT/VOLATILE` 匹配
- FR-6.4 真步态闭环：bridge + inference 同时跑，观察 `/joint_states` 轨迹，机器人保持站立/轻微步态（真模型，非零 action）

### FR-7 文档

- FR-7.1 需求、设计、开发计划、测试报告置于 `docs/superpowers/`，命名带日期 `2026-06-21`

## 4. 非功能需求

- **NFR-1 可移植**：aarch64（本机 RK3588）运行；脚本不再含主机特定绝对路径
- **NFR-2 不破坏 Jetson 可用性**：路径改为仓库相对 / 标准发行版路径，与 Jetson 侧不冲突
- **NFR-3 安全**：本轮无真机动作；mujoco viewer 在 Mali/Panfrost EGL 上若不可用则 `--no-viewer` 回退
- **NFR-4 一致性**：Python 语法保持 3.10 兼容（当前代码已无 3.12 专有语法，无需改动）
- **NFR-5 文档完备**

## 5. 约束

- 目标发行版 **ROS2 Humble**（非 Jazzy），Python 3.10（系统 python == ROS python）
- 本轮**仅仿真+离线测试**，无真机硬件
- 复用已提交的 Python 仿真栈，不重写业务逻辑（obs 复刻核对除外）
- 代理 `http://127.0.0.1:7897`；sudo 密码 1234

## 6. Phase 1 验收标准

- AC-1 ROS2 Humble 可用 + pip + mujoco/onnxruntime 安装可用（`import rclpy/mujoco/ort/numpy` 全部通过，版本正确）
- AC-2 `policy_mevius2.onnx` 从 mevius2 `policy.pt` 导出成功，pt-vs-onnx 数值对齐通过
- AC-3 已提交脚本无 `/opt/ros/jazzy`、`/home/esi`、`install_sp_d` 残留（grep 干净，docs 历史记录除外）
- AC-4 `build_sp_*`/`install_sp_*` 已删除
- AC-5 `sim_inference_node.py` obs 复刻与 mevius2 一致
- AC-6 mujoco bridge 无头运行，`/joint_states` ~200Hz、12 关节、`/imu` ~200Hz
- AC-7 sim_inference_node 加载 ONNX 并 50Hz 发布 `/joint_targets`，接线 1 pub+1 sub
- AC-8 真步态闭环：机器人保持站立/轻微步态（真模型）
- AC-9 测试报告文档完成，记录各项结果

## 7. Phase 2 待开工项（记录，不开工）

### 7.1 工作项

1. 推送未提交的 `feat/sp-d-mevius2` 子模块分支（`src/inference` 等当前 HEAD 偏离 superproject 记录 SHA）
2. 在 Humble/py3.10 用 colcon 重建三子模块：`imu_py`、`motors_py`、`robot_py`（C++ + pybind11）
3. 重写 `tools/can_setup.sh` 为 RK3588 USB-CAN（`gs_usb` 模块，非 Jetson 的 `mttcan`）
4. 重配 `src/inference/config/robot.yaml` 为 mevius2 12 关节 RS03 拓扑（替换当前 23-DOF RPO/DM 配置）
5. RT 内核安装（`assets/*.deb` 5.10 RT，或确认本机 kernel 5.10.0-1012-rockchip 是否已 RT）+ `/etc/security/limits.conf` rtprio/memlock
6. udev 规则确认（`assets/99-auto-up-devs-orangepi.rules` 的 `KERNELS` 匹配实际 USB-CAN/IMU 接线）
7. 真机 IMU 直测 + 电机闭环（`/init_motors`、`/set_zeros`、inference 启停）

### 7.2 阻塞项（本轮无法消除）

- `feat/sp-d-mevius2` 子模块分支未推送 → colcon 重建的前提
- 真硬件在场（电机/CAN/IMU 实物）→ 真机测试前提

### 7.3 验收（待开工后细化）

- `colcon build --symlink-install` 在 Humble/py3.10 成功产出 imu_py/motors_py/robot_py
- `robot_py.RobotInterface` 能加载 mevius2 robot.yaml 并读 IMU、初始化电机
- `./tools/start_robot.sh` 真机启动 inference_session + joy_session
- 真机 IMU ~500Hz、电机 12 关节可控、`/set_zeros`/`/start_inference` 服务可用

### 7.4 与 Phase 1 的衔接

Phase 1 产出的 `policy_mevius2.onnx`、路径修正、清理结果 Phase 2 直接复用；Phase 1 验证过的仿真闭环作为真机上线前的回归基线。
