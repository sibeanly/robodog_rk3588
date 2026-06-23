# basemevius2 Rough-Terrain 策略 MuJoCo 移植总结

**日期**: 2026-06-23
**分支**: feat/rk3588-humble-port
**目标**: 将 Isaac Lab (RSL-RL PPO) 训练的 mevius2 rough-terrain 策略 (`policy_21399.onnx`, 232→12) 移植到 MuJoCo 仿真，复现站立/行走/越障行为。

## 1. 背景

- 旧 mevius2 模型（34→12）已在 `robodog_jeston` 仓库的 Python sim 路径跑通（`sim/sim_inference_node.py` + `sim/mujoco_bridge.py`）。
- 用户新训练了 rough-terrain 策略 `policy_21399.onnx`（232→12，RSL-RL PPO actor，4 层 ELU MLP，无 obs 归一化，batch=1，opset 18）。训练框架在 `/home/orange5plus/code/basemevius2/`，任务 `RobotLab-Isaac-Velocity-Rough-Mevius2-Mevius2-v0`。
- 训练用 URDF: `/home/orange5plus/code/basemevius2/source/robot_lab/data/Robots/mevius2/mevius2_description/urdf/mevius2.urdf`。
- 黄金对照: `/home/orange5plus/code/basemevius2/onnx_script/deploy_mujoco.py` + `DEPLOY_MUJOCO.md`（用户在另一台开发机跑通的 sim2sim 部署）。

## 2. 新旧模型约定对比

| 维度 | 旧 mevius2 (34→12) | 新 basemevius2 (232→12) |
|---|---|---|
| 输入维度 | 34（无历史） | **232** |
| Obs 结构 | ang_vel×0.25, gravity_b, cmd×[2,2,0.25], (dof_pos-def)×DOF_SYM, dof_vel×0.05×DOF_SYM, is_standing | ang_vel×0.25, gravity_b, cmd×1.0, (dof_pos-def), dof_vel×0.05, **last_action(12)**, **height_scan(187) clip[-1,1]** |
| 关节顺序 | [BL,BR,FL,FR]×[collar,hip,knee] | **[FR,FL,BR,BL]**×[collar,hip,knee] |
| DOF_SYM | [1,1,1,-1,1,1,1,1,1,-1,1,1] | **全 +1**（无对称符号） |
| Action scale | 单一 0.2 × DOF_SYM | **每关节**: collar=0.125, hip=0.15, knee=0.30 |
| Action clip | ±100 | **±100**（训练值，RSL-RL 无 tanh，输出可达 ±5） |
| DEFAULT_ANGLE | [0,0.7,-1.2]×4 | 同 |
| PD / 频率 | kp=50, kd=2, sim 200Hz, policy 50Hz | 同 |
| 归一化 | 无 | 无 (actor_obs_normalization=False) |
| cmd scale | [2,2,0.25] | **1.0** |

## 3. 部署约定（单一真值表，sim 与 C++ 共用）

从训练源码推导（`rough_env_cfg.py`、`assets/mevius2.py`、onnx 签名、Isaac 真值）：

| 项 | 值 |
|---|---|
| 关节顺序（policy 原生） | `[FR,FL,BR,BL]×[collar,hip,knee]` |
| Obs 232 维 | `[0:3]`ang_vel×0.25 · `[3:6]`projected_gravity=R(q_wxyz)^T·[0,0,-1] · `[6:9]`cmd[vx,vy,wz]×1.0 · `[9:21]`(joint_pos−default)×1.0 · `[21:33]`joint_vel×0.05 · `[33:45]`last_action(原始onnx输出,初值0) · `[45:232]`height_scan(187) clip[-1,1] |
| Obs clip | 全部 ±100（height_scan 已 ±1） |
| cmd 范围 | vx∈[-0.8,0.8], vy∈[-0.5,0.5], wz∈[-0.8,0.8] |
| default | `[0.0, 0.7, -1.2]×4`（policy 顺序） |
| dof_sym_sign | 全 +1 |
| Action | `target = default + clip(onnx_out, ±100) × per_joint_scale`，scale=`[0.125,0.15,0.30]×4` |
| target 限位 | collar±0.7854, hip[-1.0472,2.6180], knee[-2.8508,-0.7812]（URDF） |
| PD | kp=50, kd=2，力矩 clip ±60（effort_limit），**每 200Hz 物理步重算** |
| 频率 | policy 50Hz（dt=0.02, decimation 4, sim dt 0.005） |
| height_scan | `base_z − ray_hit_z − 0.5`，clip[-1,1]，**动态 187 射线**（平地≈-0.018） |

## 4. 实现结构

```
sim/obs_math.py            # 纯 obs/action 数学（单一真值源，单元测试）
sim/test_obs_builder.py    # pytest 单测（11/11）
sim/sim_inference_node.py  # ROS2 节点：订阅 /joint_states /imu /mujoco/base_pose /mujoco/height_scan /cmd_vel，跑 onnx，发 /joint_targets
sim/mujoco_bridge.py       # ROS2 节点：mujoco 物理 + PD + 187 射线 height_scan，发 /joint_states /imu /mujoco/base_pose /mujoco/height_scan
sim/run_sim.sh             # 无头一键启动
sim/run_sim_viewer.sh      # 带可视化启动（前台 bridge+viewer，自动清理幽灵 IMU）
sim/_debug_closed_loop.py  # 独立闭环（无 ROS，对照测试）
sim/_debug_pd_hold.py      # 纯 PD hold 稳定性测试
sim/_debug_old_model.py    # 旧模型对照测试
```

## 5. 移植过程与踩坑记录（按时间顺序）

### 5.1 基础适配（Tasks 1-4）
- 拷贝 `policy_21399.onnx` 到 `src/inference/models/`。
- 提取纯 `obs_math.py`（TDD，11 单测）：`build_obs` / `action_to_targets`，含非单位四元数 gravity 测试、clip 测试、height_scan 测试。
- 重写 `sim_inference_node.py`：232 维 obs、per-joint action、`last_action` 状态、订阅 `/mujoco/base_pose` 取 base_z。
- `mujoco_bridge.py` 关节顺序改 `[FR,FL,BR,BL]`（与 policy 一致，按名映射）。

### 5.2 踩坑 1：STANDBY keyframe 蜷缩姿态 → 启动即翻
- **现象**: 机器人第一帧就底朝天（`gravity_b.z=+1`）。
- **根因**: 旧 STANDBY keyframe 是蜷缩姿态（`z=0.03, joints [-0.169,1.28,-2.87]`），新策略训练自站立姿态（`z=0.45, joints [0,0.7,-1.2]`），spawn 即 out-of-distribution。
- **修复**: keyframe 改站立姿态（`z=0.48, joints [0,0.7,-1.2]×4`，commit `00164cd`）。

### 5.3 踩坑 2：幽灵 IMU publisher 污染 /imu → 翻倒
- **现象**: 独立闭环（无 ROS）能站住，ROS 路径第一帧 `gravity_b.z=+1`（底朝天）。`head` 日志显示 `quat` 在 `[1,0,0,0]`（mujoco 正立）和 `[0.019,0.779,-0.625,0.043]`（~178°）间跳变。
- **根因**: `scripts/damiao_imu_node.py`（真实 IMU 驱动）进程残留，与 mujoco_bridge 共发 `/imu`。inference 用 best-effort 混收两源 → obs 在正立/底朝天间跳变 → 翻倒。IMU 硬件已拔但驱动进程未关。
- **修复**: 杀掉 `damiao_imu_node` 进程；`run_sim_viewer.sh` 启动前自动清理（commit `b49cc0d`）。`ros2 topic info /imu -v` 确认 Publisher count=1。
- **教训**: 调 sim 行为异常前，先查 `/imu`、`/joint_states` 的 publisher 数量。

### 5.4 踩坑 3：height_scan 填 0 → 步态衰减
- **现象**: 站立稳，但走路不前进（原地踏步）。
- **根因**: height_scan 填固定 0，丢失 `base_z-0.5` 的动态垂直信号（策略用它维持步态自持）。
- **修复**: 实现真实 187 射线 height_scan（commit `7d188cd`），`base_z - ray_hit_z - 0.5`，clip[-1,1]。

### 5.5 踩坑 4：height_scan 射线打到机器人自身腿 → 假深坑
- **现象**: height_scan min=-0.59（应为 -0.02），策略以为前方有深沟。
- **根因**: 射线未排除机器人 geom，打到移动的腿。
- **修复**: 机器人 geom 设 group 1，地形（floor+box）group 0，射线只测 group 0（commit `d0c3a34`）。

### 5.6 踩坑 5（核心 bug）：action clip ±1 → 截断步态，脚抬不起
- **现象**: 站立稳，走路前后摇摆不前进。
- **根因**: 误用 README "安全建议" 的 ±1 clip。训练实际用 `clip={".*":(-100,100)}`，RSL-RL actor 无 tanh，原始输出可达 ±5。clip ±1 把屈膝 -2.27 砍到 -1，脚抬不起 → 静蹲。
- **修复**: `CLIP_ACTION=100.0`（commit `7a9882e`）。**这是走路不前进的核心根因**。

### 5.7 踩坑 6：PD 力矩无限幅
- **根因**: 训练 DCMotorCfg `effort_limit=60`，我的 PD 未 clip 力矩。
- **修复**: `tau = clip(KP*(target-q)+KD*(-qvel), ±60)`（commit `7a9882e`）。

### 5.8 踩坑 7：wz 标量索引 → IndexError 杀线程
- **现象**: `run_sim_viewer.sh` 启动即崩 `IndexError: invalid index to scalar variable`。
- **根因**: `publish_height_scan` 里 `wz` 是标量，误写 `wz[i]`。
- **修复**: `pnt = np.array([wx[i], wy[i], wz])`（commit `2915cbc`）。

### 5.9 踩坑 8（核心 bug）：height_scan 网格点顺序错 → 越障失败
- **现象**: 平地能走，但越障翻倒。
- **根因**: 187 个网格点排列顺序与训练不一致。IsaacLab `GridPatternCfg` flatten 顺序是 `meshgrid(Y, X, indexing="ij")` + `stack([X.ravel(), Y.ravel()])`，**x 是内层（最快变化）索引**。我误用 `meshgrid(X, Y, indexing="ij")` 让 y 成内层 → 187 维地形向量被打乱。平地所有点值相同（顺序无关）故能走；越障时点序错 → 策略读到错乱地形图 → 翻倒。
- **修复**: 改用 `meshgrid(ys, xs, indexing="ij")` + `stack([XX.ravel(), YY.ravel()])`，与 `deploy_mujoco.py` GRID_B 完全一致（commit `37d05f9`）。**这是越障失败的核心根因**。

### 5.10 base_link CoM 探索
- MJCF 的 base CoM 被手改为 0.0566（URDF 是 0.0166）。试过改回 0.0166，新旧模型都不稳；恢复 0.0566（jeston-main 跑通旧模型的值）。CoM 不是步态问题主因（commit `845b1e7` → revert `f920e88`）。

## 6. 关键对照实验：旧模型 vs 新模型

同一 MJCF、同一机器、同一 PD：
- **旧模型**（34维，jeston-main 约定）：走路稳定，x: 0→2.56m，tilt 7-12°。
- **新模型**（232维）：修复前走不远（x→0.47 停，tilt 发散 25-29°）；修复后稳定。

这证明物理/PD/MJCF 正确，问题在新模型 obs/action 约定复现。

## 7. 最终验证结果

独立闭环（无 ROS，`sim/_debug_closed_loop.py` 逻辑）：
- **行走** vx=0.5：x: 0→+5.53m，z 稳定 0.434-0.449（训练目标 0.446），tilt 6-15°（正常步态）。
- **越障** 越过 x=0.8 box：tilt 越障时 8°，越过后收敛 1.2°，持续前进不倒。
- **单测**：11/11 通过。

ROS 路径（`run_sim_viewer.sh`）用同一份 `obs_math.py` + `mujoco_bridge.py`，逻辑一致。

## 8. 运行方法

```bash
# 可视化启动（自动清理幽灵 IMU）
cd /home/orange5plus/code/robodog_jeston
./sim/run_sim_viewer.sh

# 另开终端，行走
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'   # 前进
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'                    # 停止
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'  # 转向
```

命令范围：vx±0.8, vy±0.5, wz±0.8（超出自动 clip）。

## 9. Sim2Sim 移植检查清单

- [x] MuJoCo MJCF：STL mesh + 自由关节 + 力矩执行器 + IMU 传感器，关节顺序与策略一致。
- [x] obs shape `[1,232]` float32，顺序严格匹配 PolicyCfg。
- [x] base_ang_vel：机体系（gyro 传感器），×0.25。
- [x] projected_gravity：`R(q)^T @ [0,0,-1]`。
- [x] joint_pos：相对默认角。
- [x] previous_actions：保存上一帧 ONNX 输出，初始 0。
- [x] **action clip = ±100**（训练值，RSL-RL 无 tanh，**不要 ±1**）。
- [x] **height_scan = base_z - hit_z - 0.5**，clip[-1,1]，**动态射线**（非固定常量）。
- [x] **height_scan 187 点顺序：x 内层**（`meshgrid(Y,X,indexing="ij")` + `stack([X,Y])`）。
- [x] PD 每 200Hz 物理步重算（非仅 50Hz），力矩 clip ±60。
- [x] 控制频率 50Hz policy / 200Hz PD（decimation=4, sim_dt=5ms）。
- [x] 地形 geom group 0（worldbody 直接子节点），机器人 geom group 1，射线测 group 0。
- [x] 启动前清理 `/imu` 幽灵 publisher（`ros2 topic info /imu -v` 确认 count=1）。
- [x] STANDBY keyframe = 站立姿态（z=0.48, joints [0,0.7,-1.2]）。

## 10. 提交记录

| commit | 内容 |
|---|---|
| 1f26341 | 纯 obs_math + 单测 |
| 5ad0cfc | 重写 sim_inference_node 232 维 |
| 46551f8 | bridge 关节顺序 [FR,FL,BR,BL] |
| 00164cd | STANDBY keyframe → 站立姿态 |
| 3f62c5b | height_scan base_z-0.5 + base_pose 订阅 |
| 7d188cd | 真实 187 射线 height_scan |
| d0c3a34 | height_scan geomgroup 过滤 |
| 7a9882e | action clip 100 + effort 60 + geom group |
| 2915cbc | wz 标量索引 bug |
| 37d05f9 | height_scan 网格点顺序（x 内层）— 越障修复 |

## 11. 后续

- C++ 真机部署（Task 6-12）：去子模块化 src/inference（已完成）、C++ height_scan obs 源、per-joint action scale、新 config `inference_mevius2_rough.yaml`、launch 默认、模型加载验证。C++ 端需复用本总结的约定（尤其 action clip ±100、height_scan 公式与点序）。
- 可视化步态偏慢：可能是 viewer 渲染 / ROS DDS 开销，非硬件性能（独立闭环远快于实时）。可用 `deploy_mujoco.py`（无 ROS）对照确认。
