# 测试报告：robodog_jeston RK3588/Humble 仿真适配 Phase 1

> 日期：2026-06-21
> 状态：通过
> 关联：需求 `2026-06-21-rk3588-humble-port-requirements.md`、设计 `2026-06-21-rk3588-humble-port-design.md`、计划 `2026-06-21-rk3588-humble-port-plan.md`
> 环境：RK3588 / Ubuntu 22.04 / kernel 5.10.0-1012-rockchip / aarch64 / Python 3.10.12 / ROS2 Humble

## 1. 验收标准（AC-1..AC-9）

| AC | 内容 | 结果 | 证据 |
|---|---|---|---|
| AC-1 | ROS2 Humble + pip + mujoco/onnxruntime 安装可用 | ✅ | `import rclpy/mujoco/ort/numpy` 全通过；mujoco 3.9.0 / onnxruntime 1.21.0 / numpy 2.2.6 |
| AC-2 | policy_mevius2.onnx 从 mevius2 policy.pt 导出，数值对齐通过 | ✅ | export 脚本 `RESULT: PASS`，allclose(atol=1e-5,rtol=1e-4) 10/10 OK，in [batch,34] out [batch,12] |
| AC-3 | 脚本无 jazzy//home/esi/install_sp_d 残留（grep 干净） | ✅ | grep 排除 docs/.git/build*/install* 后 exit=1（无匹配） |
| AC-4 | build_sp_*/install_sp_* 已删除 | ✅ | ls exit=2（不存在）；已加 .gitignore 防复发 |
| AC-5 | sim_inference_node.py obs 复刻与 mevius2 一致 | ✅ | 逐项核对 12 项常量全一致（见 §3 T7），零改动 |
| AC-6 | mujoco bridge 无头运行，/joint_states ~200Hz 12关节，/imu ~200Hz | ✅ | /joint_states 202.1Hz，/imu 199.9Hz，12 关节名正确 |
| AC-7 | sim_inference_node 加载 ONNX 50Hz 发 /joint_targets，接线 1 pub+1 sub | ✅ | /joint_targets 49.7Hz 12维；三话题各 1pub+1sub BEST_EFFORT/VOLATILE |
| AC-8 | 真步态闭环：机器人保持站立/轻微步态 | ✅ | base_pose 稳定 z=+0.443 qw=0.998，3秒无塌陷/翻倒，act 非零 |
| AC-9 | 测试报告文档完成 | ✅ | 本文档 |

## 2. 测试用例结果（T1..T12）

| 用例 | 验证 | 结果 | 输出摘录 |
|---|---|---|---|
| T1 import 冒烟 | rclpy/sensor_msgs/std_msgs/geometry_msgs/mujoco/ort/numpy | ✅ | `all import OK 3.9.0 1.21.0` |
| T2 路径 grep | jazzy//home/esi/install_sp_d（排除 docs 等） | ✅ | exit=1 无匹配 |
| T3 残留目录 | build_sp_*/install_sp_* | ✅ | exit=2 不存在 |
| T4 ONNX 导出 | export_policy_onnx.py 生成 onnx + 对齐 | ✅ | `RESULT: PASS`，199.7 KiB，Gemm×4+Elu×3 |
| T5 obs 复刻核对 | sim_inference_node.py vs mevius2 12 项常量 | ✅ | 全一致，零改动 |
| T6 bridge 无头 | mujoco_bridge.py --no-viewer | ✅ | `/mujoco_sim_node` 上线，200Hz 12关节 |
| T7 joint_states | hz + 关节名 + 近 STANDBY | ✅ | 202.1Hz，[BL,BR,FL,FR]×[collar,hip,knee]，pos≈[0.169,1.28,-2.87] |
| T8 imu | hz + 内容 | ✅ | 199.9Hz，frame_id=imu_link，qw≈0.9999 |
| T9 推理节点 | 加载 ONNX + 50Hz 上线 | ✅ | `loaded ONNX in=[batch,34] out=[batch,12]`，`sim_inference_node @ 50Hz` |
| T10 joint_targets | hz + 12 维 | ✅ | 49.7Hz，data len=12，sample≈[-0.02,0.68,-1.08] |
| T11 接线 | 三话题 1 pub+1 sub，QoS 一致 | ✅ | 全 BEST_EFFORT/VOLATILE，各 1pub+1sub |
| T12 真步态闭环 | bridge+inference 同跑，base_pose 稳定 | ✅ | base z=+0.443 qw=0.998，3秒稳定，act 非零（真模型） |

> 注：计划 T1..T12 与需求 FR-1..FR-7 / AC-1..AC-9 映射见设计文档第 5 节。

## 3. T5 obs 复刻核对明细

逐项比对 `sim/sim_inference_node.py` 与 `/home/orange5plus/code/mevius2/scripts/mevius2_utils.py`、`parameters.py`：

| 项 | sim_inference_node.py | mevius2 来源 | 一致 |
|---|---|---|---|
| obs 维度 34 | build_obs 拼接 3+3+3+12+12+1 | mevius2_utils.py:95-102 | ✅ |
| SCALE_ANG_VEL | 0.25 (:59) | ang_vel=0.25 | ✅ |
| SCALE_LIN_VEL | 2.0 (:60) | lin_vel=2.0 | ✅ |
| SCALE_DOF_POS | 1.0 (:61) | dof_pos=1.0 | ✅ |
| SCALE_DOF_VEL | 0.05 (:62) | dof_vel=0.05 | ✅ |
| DOF_SYM | [1,1,1,-1,1,1,1,1,1,-1,1,1] (:57-58) | mevius2_utils.py:68-71 | ✅ |
| DOF_SYM 作用于输入 | :168,169 | :73-74 | ✅ |
| DOF_SYM 作用于输出 | action*DOF_SYM*ACTION_SCALE (:185) | :112-116 | ✅ |
| ACTION_SCALE | 0.2 (:63) | parameters.py:81 | ✅ |
| DEFAULT_ANGLE | [0,0.7,-1.2]×4 (:55-56) | :60-65 | ✅ |
| joint order | [BL,BR,FL,FR]×(collar,hip,knee) (:46-51) | :46-51 | ✅ |
| CLIP_OBS/ACTION | 100/100 (:64-65) | :19-20 | ✅ |

结论：零改动，sim_inference_node 已正确复刻 mevius2 观测/动作语义。

## 4. 环境最终状态

| 项 | 值 |
|---|---|
| ROS2 | Humble（/opt/ros/humble，已预装） |
| Python | 3.10.12（/usr/bin/python3，== ROS python） |
| pip | 22.0.2 |
| mujoco | 3.9.0（--user） |
| onnxruntime | 1.21.0（--user） |
| numpy | 2.2.6（被 pip 升级，高于 spec 1.21.5，可用） |
| torch | 2.12.1+cu130（--user，仅导出用；CUDA 构建但 cuda.is_available()=False，本机无 Nvidia GPU；导出完可删） |
| onnx | 1.22.0（--user，仅导出用） |
| apt 代理 | /etc/apt/apt.conf.d/95proxy → 127.0.0.1:7897 |

## 5. 已提交改动（feat/rk3588-humble-port）

- `9a3c804` docs(sp): 两阶段 spec（sim now, real-hw later）
- `1aa4980` docs(sp): Phase 1 实现计划
- `8de955f` fix(sp): 路径/distro 残留修正（7 文件：export_tool/start_robot/run_damiao_imu/damiao_imu_rviz/damiao_imu_node/mujoco_sim.launch/mujoco_bridge）
- 删除 build_sp_*/install_sp_* + .gitignore 加 build_*/install_*（Jazzy/py3.12 残留，曾被误提交）
- `policy_mevius2.onnx`：导出成功但**位于子模块 src/inference 工作区内（未跟踪）**，不在父仓库管理范围；sim 节点按路径可加载，功能不受影响

## 6. 已知事项 / 偏离

- **numpy 升级到 2.2.6**：pip 装 mujoco/onnxruntime 时自动满足依赖升级，高于 spec 固定的 1.21.5。已验证 mujoco 3.9.0 / onnxruntime 1.21.0 在 numpy 2.x 下 import 与运行均正常。
- **policy_mevius2.onnx 在子模块内未提交**：`src/inference/models/` 属子模块 src/inference（detached f265c0a）。本轮明确不动子模块内容，onnx 留在子模块工作区未跟踪，sim 节点按硬编码路径可加载。若需版本化该 onnx，需在子模块仓库内提交（属 Phase 2 子模块治理）。
- **torch 为 CUDA 构建但本机无 Nvidia GPU**：RK3588 是 Rockchip NPU 不是 CUDA。导出只用 CPU（torch.jit.load + torch.onnx.export），cuda.is_available()=False，功能正常。理想应装 CPU-only torch（--index-url .../cpu），但已装版本可用且导出已完成，无需重装。运行推理用 onnxruntime CPUExecutionProvider，不依赖 CUDA。
- **ros2 topic hz/echo 对 BEST_EFFORT 话题 hang**：Humble 已知行为，echo 需 `--qos-reliability best_effort`，hz 工具不可靠。本报告频率数据改用 Python rclpy 订阅者实测，更准确。

## 7. 后续阶段（Phase 2，未开工）

真机 C++ 全链，阻塞于未推送的 `feat/sp-d-mevius2` 子模块分支与真硬件在场。详见需求文档第 7 节。Phase 1 产出的 policy_mevius2.onnx、路径修正、清理结果 Phase 2 直接复用；Phase 1 仿真闭环作为真机上线前回归基线。

## 8. 结论

**Phase 1（仿真 + 离线测试）全部通过。** AC-1..AC-9 全部 ✅，T1..T12 全部 ✅。纯 Python 仿真链在 RK3588/Humble/py3.10 上跑通真步态闭环：mujoco bridge 200Hz 物理 + sim_inference 50Hz 策略，经 /joint_states、/imu、/joint_targets 三话题 BEST_EFFORT 闭环，机器人保持站立（base z=0.443 qw=0.998）。无真机动作，符合本轮范围。
