# mevius2 四足机器人 ROS2 Jazzy 迁移 — 测试报告

> 日期：2026-06-18
> 状态：完成
> 关联：头脑风暴/需求/设计 `docs/superpowers/specs/2026-06-18-*.md`，SP-A/SP-E 计划 `docs/superpowers/plans/`

## 总览

将 `roboparty_deploy`（ROS2 Humble/人形双足）迁移为 mevius2 四足机器狗（ROS2 Jazzy/Jetson Thor，12 DoF，RobStride03 电机，达妙 DM-IMU-L1）。5 个子项目（SP-A/B/C/D1/D/E）全部完成，仿真闭环验证通过（站立 + 行走）。

| 子项目 | 内容 | 状态 |
|---|---|---|
| SP-A | 框架骨架 Jazzy 迁移 | ✅ |
| SP-B | 达妙 DM-IMU-L1 驱动 + 真机直测 | ✅ |
| SP-C | RobStride03 电机驱动（方案C） | ✅ |
| SP-D1 | policy.pt → ONNX 导出 + 数值对齐 | ✅ |
| SP-D | mevius2 本体 + 推理适配（obs/配置） | ✅ |
| SP-E | mujoco 仿真 + 测试报告 | ✅ |

---

## SP-A 框架骨架 Jazzy 迁移

**完成项：**
- 构建工具链：安装 colcon、python3-pybind11、rosdep（系统 python3.12）
- `tools/can_setup.sh`：bring up 原生 mttcan SocketCAN，can0（BL+BR）+ can1（FL+FR），1 Mbps。已验证两接口 `state UP`、`bitrate 1000000`
- `assets/99-dm-imu-jetsonthor.rules`：udev 规则按 USB serial（DMIMU20250212）绑定 DM-IMU-L1 到 `/dev/dm_imu` 符号链接（→ ttyACM0），权限 0666，dialout 组
- `tools/start_robot.sh`：source `/opt/ros/jazzy/setup.bash`，build 前调 can_setup
- 第三方依赖验证：onnxruntime aarch64 1.21.0 + yaml-cpp 0.9.0 在 Jazzy 可 FetchContent（73s 下载解压，libonnxruntime.so 17MB 就位）

**环境探查结论：**
- CAN：Jetson Thor 原生 `nvidia,tegra264-mttcan`，4 路 can0~can3（纯 SocketCAN，无需 USB-CAN 适配器/udev-USB 绑定）
- IMU：`/dev/ttyACM0`（CDC-ACM），dialout 组
- 架构 aarch64，14 核
- **Python 陷阱**：miniconda py3.13 在 PATH 前，但 ROS2 Jazzy 用系统 python3.12。所有 ROS Python（colcon/节点/pybind/mujoco）必须用 `/usr/bin/python3`（3.12），不激活 conda。colcon build 需 `unset PYTHONPATH PYTHONHOME` + `PATH=/usr/bin:...` + `-DPython3_EXECUTABLE=/usr/bin/python3`

---

## SP-B 达妙 DM-IMU-L1 IMU 驱动

**完成项：**
- `src/imu/src/drivers/damiao/`（hpp/cpp/CMakeLists），镜像 hipnuc 驱动结构，工厂注册 `"DAMIAO"`
- `src/imu/src/protocol/serial/serial_port`：新增 `write()` 方法（发配置命令用）
- C++ 驱动解析 80 字节复合上行帧（4 子包×20）+ CRC16-CCITT（poly 0x1021, init 0xFFFF），后台线程 + 单位转换（gyro °/s→rad/s，quat 保持 w,x,y,z）
- `roboparty_imu` 在 Jazzy 编译通过，`imu_py` 可导入

**真机直测（/dev/dm_imu @ 921600）：**
- 频率：**1000 Hz**（帧间隔 1.000ms）
- 四元数：有效单位四元数 |q|=1.0000，实时更新（非 identity）
- lin_acc：|a| = **9.9 m/s²** → 确认 m/s² 单位（非 g）✓
- gyro：静止 ~**0.0007 rad/s** → 确认 rad/s 单位（若未转换 °/s 会是 ~0.01-0.05，大 50×）✓

**修复的 2 个 dm-imu-viewer 参考实现潜在 bug：**
1. **四元数 CRC 覆盖 20 字节而非 16**：23 字节 quat 子包 CRC 覆盖 `[0..19]`（header+4 floats），19 字节子包覆盖 `[0..15]`。规则 `CRC covers [0:crc_offset]`。viewer 两个都用 16 且从不走 quat 路径（其 configure_default 禁用 quat），有潜在 bug。用真实包暴力验证修复。
2. **四元数使能命令**：该模块 quat 被关闭（之前官方 ROS1 驱动运行过 turn_off_quat+save 到 flash）。从通道参数模式 `0x13+reg_id` 推导使能命令 `AA 01 17 0D`（accel 0x14/gyro 0x15/euler 0x16/quat 0x17），发送后 reg=0x04 包出现在流中。指出 Python viewer 的 `AA 01 04 0D` 实为 disable-accel（另一 bug）。

**注意：** 80 字节帧无温度字段（19+19+19+23=80），`get_temperature()` 返回 0.0。

---

## SP-C RobStride03 电机驱动（方案C）

**完成项：**
- `src/motors/src/drivers/robstride/`（hpp/cpp/CMakeLists），移植 EDULITE_A3 `RobstrideCanDriver` 协议，外层套 roboparty `motor_driver` 抽象
- 工厂注册 `"RobStride03"`/`"RS03"`
- RS03 私有协议（29 位扩展 ID，5 个 16 位字段）：comm 0x01 MIT（torque 在 ExtID bits23-8，p/v/kp/kd 大端 uint16）、0x02 状态解析、0x03 enable、0x04 disable+清故障、0x06 set_zero、0x07 set_id、0x11 read_param、0x16 save
- 16 位线性映射，RS03 限位 P=±4π, V=±20（mevius2 降额）, KP=0..5000, KD=0..100, T=±60
- `roboparty_motors` 在 Jazzy 编译通过，`motors_py` 可导入，工厂识别 RS03（测试到 CAN socket bind 停，零执行机构动作）

**设计要点：**
- SocketCAN 抽象**无需改动**——原生支持 29 位扩展 ID（CAN_EFF_FLAG），RX 按 `(motor_id<<8)|master_id` demux，无 master_id_offset（区别于达妙）
- sign/offset 按框架惯例：motor_sign 在上层 robot_interface 应用，motor_zero_offset 在驱动层应用（与 dm 一致）

---

## SP-D1 policy.pt → ONNX 导出

**完成项：**
- `tools/export_policy_onnx.py`：torch.jit.load + torch.onnx.export（opset 17）
- `src/inference/models/policy_mevius2.onnx`：199.7 KiB，输入 `obs [batch,34]` → 输出 `action [batch,12]`，批次维动态
- actor MLP 34→256→128→64→12（ELU），float32，无 RNN

**数值对齐：**
- max-abs-diff = **2.29e-5**（合成 U[-1,1] 输入，输出幅值达 ~73）
- 标准 `np.allclose(atol=1e-5, rtol=1e-4)` 全 10 样本通过
- 相对误差 ~1e-5（~10 ppm），恒定跨输入尺度（scale sweep 验证：max-abs-diff 精确跟踪 max|out|，是正确的 float32 跨后端 BLAS 抖动特征，非导出 bug）
- 1e-5 绝对阈值在输出幅值 73 时是 sub-ulp 物理不可达；真实观测尺度 |out|≲23 时满足

**环境：** lerobot docker `lerobot0.4.4:v4.0`（aarch64 主机无 torch wheel，conda lerobot 也无 torch），torch 2.9 JetPack 25.09，onnx 1.18，onnxruntime 1.23。opset 17，IR v8。onnx.checker 通过。

---

## SP-D mevius2 本体 + 推理适配

**完成项：**
- obs_manager 补丁（复用现有 DSL，3 处小补丁）：
  - 新增 `is_standing` obs source（`float(norm(cmd[:3])<0.03)`）
  - `dof_sym_sign` 支持：dof_pos/dof_vel 乘 sym_sign（作用于 usd2urdf 重映射后索引），action 乘 sym_sign
  - 注册 `is_standing` 到 obs source 表
- `config/inference_mevius2.yaml`：34 维 obs layout、joint_num=12、frame_stacks=1（无帧堆叠）、decimation=4/dt=0.005（50Hz policy/200Hz PD）、action_scale=0.2、obs_scales_lin_vel=2.0、obs_scales_ang_vel=0.25、obs_scales_dof_vel=0.05、dof_sym_sign=[1,1,1,-1,1,1,1,1,1,-1,1,1]、usd2urdf 恒等、joint_default_angle=[0,0.7,-1.2]×4、joint_limits（从 URDF）、gravity_z_upper=-0.5
- `config/robot.yaml`：12 关节（motor_id [10,11,12,7,8,9,4,5,6,1,2,3] 按 [BL,BR,FL,FR] 物理 CAN_ID）、can0(BL+BR)/can1(FL+FR)、RobStride03、DAMIAO IMU /dev/dm_imu、kp=50/kd=2、motor_sign=MOTOR_DIR、motor_zero_offset、close_chain 空、urdf2motor 恒等
- `launch/inference.launch.py` 指向 inference_mevius2.yaml
- 整链（imu+motors+inference）Jazzy 编译通过，`robot_py` 导入 OK，inference_node 加载 ONNX + 解析 mevius2 obs 配置，到达 "Press 'X'" 空闲提示（无电机指令）

**关键修复：**
- **动态 ONNX 输出 batch 维 -1→1**：mevius2 ONNX 输出 shape `[-1,12]`，原 setup_model 只解析输入 dim，导致 CreateTensor 崩溃。通用修复，静态 shape 人形模型不受影响。
- **close_chain `type=="mevius2"`→nullptr**：四足无闭链，否则抛 "Unknown close_chain type"。

**关键决策（需硬件验证）：**
- **motor_zero_offset = MOTOR_OFFSET_ANGLE × MOTOR_DIR**（非原始值）。roboparty 驱动 `joint=(raw+offset)`+上层 `*motor_sign`；mevius2 `joint=raw*dir+offset`。等价需 `offset_rp = offset_mevius*dir`。数学证明成立（BL_hip: dir=-1,offset=+5.027,raw=0 → 两者均得 +5.027）。**真机驱动前需对照已知姿态验证关节角。**

---

## SP-E mujoco 仿真

**完成项：**
- `sim/mujoco_bridge.py`：rclpy + mujoco 3.9.0 仿真节点。加载 `assets/mujoco/scene.xml`，keyframe 0 重置，200Hz 物理步进，PD（tau=50·(target-pos)+2·(0-vel)）驱动 actuator，发布 `/joint_states`（12 关节 [BL,BR,FL,FR]）+ `/imu`（base_link 四元数 wxyz→xyzw + body_gyro_sensor body-frame 角速度），订阅 `/joint_targets`。按关节名映射（mujoco 模型关节名 FR,FL,BR,BL 顺序，bridge 统一按名映射到 [BL,BR,FL,FR]）。
- `sim/sim_inference_node.py`：rclpy + onnxruntime 1.27.0 推理节点（`providers=['CPUExecutionProvider']` CPU 推理，消除 GPU 探测 warning）。复刻 obs_manager 数学构建 34 维 obs，50Hz 策略，`target=DEFAULT+0.2·(action·sym)`，发布 `/joint_targets`。
- `sim/launch/mujoco_sim.launch.py`

**仿真结果（核心验收）：**

1. **站立**：零 cmd 启动 → gravity_b=[0,0,-1.00]（水平站立），stand=1，target≈DEFAULT_ANGLE，base xyz 稳定（z=0.030，qw=1.0）。**机器人稳定站立不倒** ✓
2. **行走**：`ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:{x:0.3}}'` → cmd=[0.6,0,0]（×SCALE_LIN_VEL），stand=0，策略输出周期性摆动（步态），base x 前进：0.11→0.21→0.45→0.70→**1.02 m**（~0.3-0.4 m/s），gravity_b 小幅摆动 z≈-1（未摔倒）。**机器人响应指令前进行走** ✓

**验证的链路：** mujoco 物理引擎 → /joint_states+/imu → sim_inference_node（obs 构建 + ONNX 推理）→ /joint_targets → mujoco PD 驱动。全闭环通。

**环境：** mujoco 3.9.0 + onnxruntime 1.27.0（系统 python3.12，`pip install --user --break-system-packages`）。onnxruntime GPU 探测 warning（/sys/class/drm/card）无害——自动回退 CPUExecutionProvider，符合 onnxruntime CPU 推理设计。

---

## 已知问题 / TODO

1. **esi 用户 rtprio 限制 0**：inference_node 的 SCHED_FIFO 调用失败。测试时用 sudo 运行，或按 CLAUDE.md 在 `/etc/security/limits.conf` 设 rtprio/memlock 后重新登录。1000Hz IMU 解析在普通调度下已达成。
2. **motor_zero_offset 硬件验证待做**：SP-D 用了 `MOTOR_OFFSET_ANGLE×MOTOR_DIR` 变换，真机驱动前需对照已知姿态验证关节角（数学已证明等价，但需实测确认）。
3. **DAMIAO IMU 四元数 sense / 安装方向**：`get_gravity_b_obs` 用 w-first 四元数做 R(q)ᵀ·[0,0,-1]，需真机确认水平时 gravity_b≈[0,0,-1]、gyro 轴向匹配 body frame。extrinsic_R 当前为单位阵（按 IMU 安装方向调整）。
4. **Gazebo Harmonic 集成后置**：本阶段用 mujoco 验证策略/步态，gz ros_gz 集成未做（设计文档列为后续）。
5. **真机电机执行未测试**：本阶段仅仿真。真机步态调试、零点标定为后续阶段。
6. **mevius2 的 -2π·dir 多圈包裹修正未复制**（xiaomimotor_lib）：RS03 位置范围 ±4π，±π 包裹基本 no-op，边界附近需注意。
7. **多模态感知（livox/FAST_LIO/elevation）不在范围**。
8. **TensorRT 量化未做**（mevius2 policy 小 MLP，onnxruntime CPU 足够 50Hz；`/home/esi/code/docs` TRT 量化文档列为未来选项）。

---

## 提交记录（本地，未推送 GitHub）

main 分支 6 个 commit（领先 origin/main）：
1. `8df50b2` SP-A 框架骨架
2. `fc0e331` SP-D1 ONNX 导出脚本+报告
3. `b0ba384` SP-D1 inference submodule 指针（onnx）
4. `f14a8a6` SP-C RobStride03 电机驱动
5. `d5b9d48` SP-B 达妙 IMU + 真机测试
6. `ee1eebb` SP-D mevius2 本体+推理适配

三个 submodule（inference/motors/imu）在本地分支（feat/sp-d-mevius2 / feat/sp-c-robstride / feat/sp-b-damiao-imu），未推送上游。

## 结论

迁移完成。以 roboparty_deploy 框架为基础，IMU 替换为达妙 DM-IMU-L1（真机 1000Hz 验证）、电机替换为 RobStride03（方案C 复用 EDULITE_A3 驱动）、本体替换为 mevius2 四足（12 关节，policy.pt→ONNX，obs_manager 适配），在 ROS2 Jazzy 仿真环境中验证了 RL 步态策略（站立 + 行走）。IMU 驱动经真机直测确认正确。
