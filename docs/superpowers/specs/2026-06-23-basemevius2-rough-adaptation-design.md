# basemevius2 (Rough-Terrain) 模型适配设计

**日期**: 2026-06-23
**分支**: feat/rk3588-humble-port
**方案**: A — Sim 先行验证 → C++ 移植扩展，双路径独立

## 背景

1. 当前仓库已实现原 mevius2 模型推理 + mujoco 仿真（34→12 模型，仅 Python sim 路径跑）。
2. 用户在 `/home/orange5plus/code/basemevius2/` 用 Isaac Lab RSL-RL 重新训练了一个四足 rough-terrain 策略，导出为 `policy_21399.onnx`（232→12）。训练框架与 obs/action 约定在 `basemevius2/source/robot_lab/.../rough_env_cfg.py`、`assets/mevius2.py`、`onnx_script/README.md`。
3. C++ 推理目标：Jetson Thor 真机部署，复用 `/home/orange5plus/code/robodog_jeston-main/src/inference/` 的 mevius2 C++ 基座（已含 is_standing obs 源、dof_sym_sign、inference_mevius2.yaml、mevius2 robot.yaml）。
4. 决策：去子模块化 `src/inference`，并入主仓库 `sibeanly/robodog_jeston`，所有 C++ 改动在主仓库内 commit。

## 新模型与旧模型对比

| 维度 | 旧 mevius2 (当前 sim) | 新 basemevius2 (policy_21399) |
|---|---|---|
| 输入维度 | 34 (无历史) | **232** |
| Obs 结构 | ang_vel×0.25, gravity_b, cmd×[2,2,0.25], (dof_pos-def)×DOF_SYM, dof_vel×0.05×DOF_SYM, is_standing | ang_vel×0.25, gravity_b, cmd×1.0, (dof_pos-def), dof_vel×0.05, **last_action(12)**, **height_scan(187) clip[-1,1]** |
| 关节顺序 | [BL,BR,FL,FR]×[collar,hip,knee] | **[FR,FL,BR,BL]**×[collar,hip,knee] |
| DOF_SYM | [1,1,1,-1,1,1,1,1,1,-1,1,1] | **全 +1** |
| Action scale | 单一 0.2 × DOF_SYM | **每关节**: collar=0.125, hip=0.15, knee=0.30, 输出 clip[-1,1] |
| DEFAULT_ANGLE | [0,0.7,-1.2]×4 | [0,0.7,-1.2]×4 (相同) |
| PD / 频率 | kp=50, kd=2, sim 200Hz, policy 50Hz | kp=50, kd=2, sim 0.005×decimation4=50Hz (相同) |
| 归一化 | 无 | 无 (actor_obs_normalization=False) |
| cmd scale | [2,2,0.25] | **1.0** |
| URDF | assets/mujoco/mevius2_mujoco.xml | basemevius2/.../mevius2.urdf — 关节限位与现有 MJCF 一致 |

## 部署约定（单一真值表，sim 与 C++ 共用）

从训练源码推导（`rough_env_cfg.py`、`assets/mevius2.py`、onnx 签名 232→12、`rsl_rl_ppo_cfg.py` actor_obs_normalization=False）：

| 项 | 值 |
|---|---|
| 关节顺序（policy 原生） | `[FR,FL,BR,BL]×[collar,hip,knee]` |
| Obs 232 维 | `[0:3]`ang_vel×0.25 · `[3:6]`projected_gravity · `[6:9]`cmd[vx,vy,wz]×1.0 · `[9:21]`(joint_pos−default)×1.0 · `[21:33]`joint_vel×0.05 · `[33:45]`last_action(原始12,初值0) · `[45:232]`height_scan(187)×1.0 clip[-1,1] |
| Obs clip | 全部 ±100（height_scan 已 ±1） |
| cmd 范围 | vx∈[-0.8,0.8], vy∈[-0.5,0.5], wz∈[-0.8,0.8] |
| default | `[0.0, 0.7, -1.2]×4`（policy 顺序） |
| dof_sym_sign | 全 +1（新模型训练无 sym sign） |
| Action | onnx 输出 12 → clip[-1,1] → `target = default + clip(action) × per_joint_scale`，scale=`[0.125,0.15,0.30]×4` |
| target 限位 | collar±0.7854, hip[-1.0472,2.6180], knee[-2.8508,-0.7812]（URDF） |
| PD | kp=50, kd=2（全关节） |
| 频率 | policy 50Hz（dt=0.02, decimation 4, sim dt 0.005） |
| 归一化 | 无 |
| height_scan | 平地/无感知 → 填 187 个 0（README 明示，仅损失上下台阶能力） |

**真机电机映射**（robot.yaml 电机数组顺序 `[BL,BR,FL,FR]`）：
- `usd2urdf`（policy[i]→motor idx）= `[9,10,11, 6,7,8, 3,4,5, 0,1,2]`
- `motor_sign` / `motor_zero_offset` / kp / kd：沿用 jeston-main 的 mevius2 `robot.yaml`（不变）
- `joint_default_angle` 按 motor 顺序填 `[0,0.7,-1.2]×4`

## 实现步骤

### 第 1 阶段：Python sim 适配（先行验证）

**`sim/sim_inference_node.py`**：
- obs 改为 232 维；重写 `build_obs()`：去掉 is_standing，加 last_action 状态 + height_scan=0。
- action 后处理：per-joint scale + clip[-1,1] + URDF 限位。
- 更新常量：`DEFAULT_ANGLE=[0,0.7,-1.2]×4`，per-joint scale，cmd scale=1.0，dof_sym 全+1。
- policy 顺序 `[FR,FL,BR,BL]`。
- ONNX 路径指向 `policy_21399.onnx`。

**`sim/mujoco_bridge.py`**：
- `JOINT_NAMES` 改为 `[FR,FL,BR,BL]×[collar,hip,knee]`（与 policy 顺序一致），使 `/joint_states`、`/joint_targets` 数组顺序 = policy 顺序，`sim_inference_node` 无需重排。bridge 内部按 `mj_name2id` 映射到 mujoco body 顺序，不受影响。
- 同步更新 STANDBY/初始 PD hold 的关节顺序。
- 拷贝 `policy_21399.onnx` 到 `src/inference/models/`（sim 引用同一路径）。

**验证**：`./sim/run_sim.sh` 起仿真，发 `cmd_vel`，确认站立 + 行走。这是约定正确性的地面真值（sim 无 motor_sign/零偏复杂度）。

### 第 2 阶段：去子模块化 src/inference

- `git submodule deinit -f src/inference`
- 删除 `.gitmodules` 中 `src/inference` 条目
- `git rm` 子模块（含 `.git/modules/src/inference`）
- 把 jeston-main 的 `src/inference` 目录拷入作为普通目录
- `git add src/inference .gitmodules`
- 此后所有 C++ 改动在主仓库内 commit，无需推 Roboparty/roboparty_inference 子模块。

### 第 3 阶段：C++ 移植 jeston-main mevius2 基座

第 2 阶段拷入的 jeston-main `src/inference` 已含：
- `is_standing` obs 源（`obs_manager.cpp:54`）
- `dof_sym_sign` 参数 + 应用（dof_pos/dof_vel/action）
- 动态输出 batch dim 处理（`[-1,12]`）
- `inference_mevius2.yaml`、mevius2 `robot.yaml`（RobStride03 电机，2×CAN，DAMIAO IMU）
- launch 默认指向 mevius2

此阶段无额外代码改动，验证 `colcon build --symlink-install` 通过。

### 第 4 阶段：C++ 扩展适配新模型

**4a. 每关节 action scale**：
- `action_scale_` 由 `double` 改为 `std::vector<double>`。
- action 映射：`act_[usd2urdf_[i]] = clamp(out[i], ±clip_actions) × dof_sym_sign_[i] × action_scale_vec_[i] + joint_default_angle_[usd2urdf_[i]]`。
- config 向后兼容：yaml 标量 → 广播为全关节；list → 逐关节。

**4b. height_scan obs 源**：
- `obs_manager.cpp` 注册表加 `{"height_scan", &get_height_scan_obs}`。
- 实现：零填充，大小由 config `height_scan_size` 决定（187）。平地无感知，填 0。

**4c. last_action**：jeston-main 已有该 obs 源 ✓，直接用。

**4d. 新 config `inference_mevius2_rough.yaml`**：
- `model_names: ["policy_21399.onnx"]`，`joint_num: 12`，`decimation: 4`，`dt: 0.005`
- obs layout: `"ang_vel:3, gravity_b:3, cmd_vel:3, dof_pos:12, dof_vel:12, last_action:12, height_scan:187"`，`frame_stacks: [1]`（总 232）
- `usd2urdf: [9,10,11, 6,7,8, 3,4,5, 0,1,2]`
- `dof_sym_sign: [1.0]×12`
- `action_scale: [0.125,0.15,0.30, 0.125,0.15,0.30, 0.125,0.15,0.30, 0.125,0.15,0.30]`
- `joint_default_angle: [0,0.7,-1.2]×4`
- `clip_actions: 1.0`（action clip[-1,1]，复用现有 clamp 逻辑）
- scales: `ang_vel:0.25, lin_vel:1.0, dof_pos:1.0, dof_vel:0.05`
- `height_scan_size: 187`
- `clip_cmd: [-0.8,0.8,-0.5,0.5,-0.8,0.8]`

**4e. 模型文件**：`policy_21399.onnx` 拷到 `src/inference/models/`（第 1 阶段已拷，sim 与 C++ 共用）。

**4f. launch**：`inference.launch.py` 默认指向 `inference_mevius2_rough.yaml`。

**4g. action clip 语义**：新模型 action 需 clip[-1,1]。用 `clip_actions: 1.0` 复用 `inference_node.cpp` 现有 `std::clamp(out, -clip_actions_, clip_actions_)` 逻辑（原 ±100 对应旧模型，现 1.0 对应新模型）。

## 错误处理 / 测试

- C++ `setup_model` 已校验输入 size，配置错配会直接 throw（总 232）。
- 验证链：sim 站立行走（第 1 阶段）→ C++ `colcon build` 通过（第 3 阶段）→ 真机上机（用户验证）。
- 无单元测试框架；以 sim 行为 + 编译通过为准。

## 风险

- **约定错配**（最大风险）：由 sim 先行验证兜底。
- **真机 motor_sign/零偏与 sim 约定一致性**：sim 不涉及；真机上机复用 jeston-main 已验证的 `robot.yaml`，必要时微调零偏。
- **height_scan=0 的分布偏移**：README 明示平地可接受；若站立不稳可后续在 mujoco 里做真实 raycast。
- **去子模块化不可逆**：完成后 src/inference 不再跟踪上游 Roboparty/roboparty_inference；需确认这是期望的（用户已确认）。

## 不做（YAGNI）

- 方案 B（sim 共用 C++ config）：双路径独立，约定表注释交叉文档化即可。
- C++ 真实 height_scan raycast：平地填 0 足够。
- is_standing obs 源保留但新 config 不使用。
- Unitree FR/FL/RR/RL 约定：新模型用 FR/FL/BR/BL，不改名。
