# 需求文档：mevius2 四足机器人 ROS2 Jazzy 部署框架

> 日期：2026-06-18
> 状态：待审阅
> 关联：头脑风暴 `2026-06-18-brainstorming-mevius2-migration.md`、设计文档 `2026-06-18-mevius2-migration-design.md`

## 1. 项目目标

以 `roboparty_deploy` 集成框架为蓝本，构建一个面向 mevius2 四足机器狗（12 DoF，RobStride03 电机，达妙 DM-IMU-L1）的 ROS2 Jazzy 部署框架，能在仿真环境中运行 RL 步态策略，并能独立验证 IMU 驱动与 ROS 适配。

## 2. 范围

### 2.1 在范围内

- 框架从 ROS2 Humble/U22 迁移到 ROS2 Jazzy/U24.04（Jetson Thor）
- 替换 IMU 驱动为达妙 DM-IMU-L1（C++ + pybind11，保持 `imu_py` 接口）
- 替换电机驱动为 RobStride03（C++ + pybind11，保持 `motors_py`/`robot_py` 接口）
- 替换本体为 mevius2 四足（12 关节，`robot.yaml` 重配，policy.pt→ONNX，obs_manager 适配）
- 仿真：mujoco（策略/步态验证）+ Gazebo Harmonic（ROS2 集成演示）
- IMU 真机直测：独立验证达妙 IMU 驱动 + ROS2 节点正确性
- 文档：头脑风暴、需求、设计、开发、测试报告

### 2.2 不在范围内

- 多模态感知：livox 激光雷达、FAST_LIO 里程计、elevation_mapping 高程图
- 真机电机闭环步态调试（仿真先行，真机标定/调试后续阶段）
- 训练新策略（复用 mevius2 现有 `policy.pt`）
- WiFi 热点工具（create_ap，与运动控制无关）

## 3. 功能需求

### FR-1 框架骨架（ROS2 Jazzy 迁移）

- FR-1.1 所有包 `package.xml`/`CMakeLists.txt` 兼容 ROS2 Jazzy（ament_cmake，C++17）
- FR-1.2 `start_robot.sh` 适配 Jazzy（source 路径、DDS、screen 会话），或提供 Jazzy 版启动脚本
- FR-1.3 FastDDS 共享内存 profile 在 Jazzy 验证可用
- FR-1.4 udev 规则适配 Jetson Thor 的 CAN/IMU USB 端口
- FR-1.5 `colcon build --symlink-install` 在 Jazzy 成功，生成 `imu_py`/`motors_py`/`robot_py`

### FR-2 达妙 IMU 驱动包（roboparty_imu）

- FR-2.1 C++ 驱动解析达妙 80字节复合上行帧（4子包×20）+ CRC16-CCITT 校验
- FR-2.2 通过 USB CDC-ACM 串口（921600）读取，后台线程 + 主线程轮询
- FR-2.3 单位转换：gyro °/s→rad/s；四元数 (w,x,y,z)→(x,y,z,w)
- FR-2.4 pybind11 生成 `imu_py.IMUDriver.create_imu()`，接口与原 roboparty_imu 一致
- FR-2.5 ROS2 节点发布 `sensor_msgs/Imu`（frame_id=imu_link），参数化 port/baud/frame_id/freq
- FR-2.6 **真机直测**：`ros2 topic echo /imu` 能稳定收到数据，频率达标（≥500Hz 可配），姿态变化正确

### FR-3 RobStride03 电机驱动包（roboparty_motors，方案C）

- FR-3.1 移植 EDULITE_A3 `RobstrideCanDriver`（C++，RS03 私有协议 29位扩展ID 5个16位字段，SocketCAN 1Mbps，带硬件过滤/发送互斥/重试/软启动）
- FR-3.2 外层套 roboparty `motor_driver` 抽象（`RobStrideMotorDriver`）：enable/disable、MIT 控制（p_ref/v_ref/kp/kd/tau_ff）、读状态、set_zero、改 ID、读写参数、NONE/MIT/POS/SPD 模式
- FR-3.3 按 ExtID bits15-8 解复用多电机反馈
- FR-3.4 驱动侧应用 motor_sign（乘）与 motor_zero_offset（加减）
- FR-3.5 pybind11 生成 `motors_py.MotorDriver.create_motor()`，**接口与原 roboparty_motors 完全一致**，inference_node/robot_py 依赖链不变
- FR-3.6 ROS2 服务面：`/init_motors`、`/deinit_motors`、`/clear_errors`、`/set_zeros`、`/reset_joints`、`/refresh_joints`、`/read_joints`（与原框架一致）
- FR-3.7 `SimMotorBackend` 实现 MotorDriver 接口，对接仿真 bridge（mock 模式），真机/仿真可切换

### FR-4 mevius2 本体 + 推理

- FR-4.1 `robot.yaml` 配 12 关节：motor_id(1-12)、can0(BL+BR)/can1(FL+FR)、motor_type=RobStride03、kp=50/kd=2、motor_sign=MOTOR_DIR、motor_zero_offset=MOTOR_OFFSET_ANGLE、close_chain 空、urdf2motor 恒等
- FR-4.2 `policy.pt` 导出为 `policy_mevius2.onnx`，验证 (1,34)→(1,12) 数值与原 .pt 一致（独立子 agent，参考飞书文档 + `lerobot/openpi/smolvla_on_thor`）
- FR-4.3 obs_manager 适配 34维布局：ang_vel·0.25、gravity_b=R(q)ᵀ·[0,0,-1]、cmd·[2,2,0.25]、(dof_pos−default)·sym、dof_vel·0.05·sym、is_standing
- FR-4.4 action 适配：`target=DEFAULT_ANGLE+0.2·(action·sym)`，URDF 限位保护
- FR-4.5 `inference_mevius2.yaml`：joint_num=12、action_scale=0.2、frame_stacks=1、joint_default_angle、clip_cmd、joint_limits
- FR-4.6 推理节点 50Hz，PD 200Hz，安全：IMU 失效或倾角>53° 时 kp=kd=0
- FR-4.7 复用 `mevius2_dae.urdf` 作为机器人描述

### FR-5 仿真（自写 bridge，与方案C配套）

- FR-5.1 真机/仿真共用 `MotorDriver`/`RobotInterface` 接口，`sim_mode` 切换 `RobStrideMotorDriver`/`SimMotorBackend`，inference_node 无感
- FR-5.2 **mujoco bridge**（Python rclpy+mujoco）：加载 scene.xml，发布 `/joint_states`+`/imu`（仿真 IMU），订阅 `/joint_targets`，200Hz 物理步进 + PD。验证策略步态（站立/行走）
- FR-5.3 **Gazebo Harmonic bridge**：gz sim + ros_gz 桥接 joint_states/imu/joint_targets，PD 增益 50/2。验证 ROS2 全链路
- FR-5.4 仿真 IMU 来自仿真器，**不接入真机 IMU**

### FR-6 集成与启动

- FR-6.1 `start_robot.sh`（或 Jazzy 版）能拉起推理节点 + joy + 仿真（可选）
- FR-6.2 服务控制面与原框架一致：`/start_inference`、`/stop_inference`、`/read_imu` 等
- FR-6.3 手柄（joy）控制：站立/行走切换、cmd_vel

## 4. 非功能需求

- **NFR-1 可移植**：aarch64（Jetson Thor）+ x86_64 均可编译
- **NFR-2 实时性**：策略 50Hz、PD 200Hz 抖动可接受；FastDDS 共享内存低延迟
- **NFR-3 安全**：电机指令经限位保护；IMU 失效降级；仿真先行，真机动作前确认
- **NFR-4 一致性**：`robot.yaml` 单一真相源；obs/action 关节顺序 [BL,BR,FL,FR] 全链路一致
- **NFR-5 文档**：开发文档、测试报告完备
- **NFR-6 可测**：驱动单元测试、IMU 直测脚本、仿真步态测试

## 5. 约束

- ROS2 Jazzy（非 Humble）
- C++17，ament_cmake
- 复用 roboparty 框架的 C++ ONNX 推理范式与 pybind11 驱动结构
- 复用 mevius2 现有 policy.pt（不重训）
- Jetson Thor 硬件（sudo 密码 123）

## 6. 验收标准

- AC-1 `colcon build` 在 Jazzy 成功，无错误
- AC-2 真机达妙 IMU：`ros2 topic echo /imu` 稳定输出，频率/姿态正确（FR-2.6）
- AC-3 仿真（mujoco）：机器人能站立、响应 cmd_vel 行走（FR-5.1）
- AC-4 仿真（gz）：ROS2 全链路通，joint_states/imu/cmd 桥接正常（FR-5.2）
- AC-5 服务控制面：`/init_motors`→`/start_inference`→`/stop_inference` 流程在仿真中可用
- AC-6 测试报告文档完成，记录各项测试结果
