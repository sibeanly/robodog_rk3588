# 头脑风暴记录：roboparty_deploy → mevius2 四足机器人迁移

> 日期：2026-06-18
> 状态：设计阶段（待用户审阅）
> 关联：需求文档 `2026-06-18-mevius2-migration-requirements.md`、设计文档 `2026-06-18-mevius2-migration-design.md`

## 1. 项目背景

将 `roboparty_deploy`（ROS2 Humble / Ubuntu 22.04 / rk3588 域控，人形双足）迁移适配为：

- **平台**：Jetson Thor，Ubuntu 24.04，ROS2 Jazzy（当前开发环境即此）
- **IMU**：HiPNUC → 达妙 DM-IMU-L1（已接入 OS）
- **电机**：达妙 CAN → RobStride03
- **本体**：人形双足 → mevius2 四足机器狗（12 DoF）

参考代码库：
- 框架基础：`/home/esi/code/roboparty_deploy`
- 达妙 IMU：`/home/esi/code/dm-imu-master`（驱动/文档）、`dm-imu-viewer{,-cpp}`（纯 Python/C++ 可视化参考）
- RobStride03：`/home/esi/code/robstride/RobStride`、mevius2 `xiaomimotor_lib.py`
- mevius2 本体：`/home/esi/code/mevius2-master`

## 2. 探索发现（关键接缝）

### 2.1 roboparty 框架原生推理方案（已确认）

roboparty 推理层是 **C++ + ONNX Runtime**：
- `inference_node.cpp` 用 `Ort::Session` 加载 `.onnx`（`src/inference_node.cpp:65`），`session->Run()` 推理（`:302`）
- 第三方内置 onnxruntime 1.21.0（aarch64 + x64），自带 `models/*.onnx`
- `obs_manager.cpp` 按配置 `obs_layouts` 字符串拼装观测，`frame_stacks` 做帧堆叠
- 不用 libtorch / Python 推理；训练侧 PyTorch/IsaacGym → 导出 ONNX → C++ 部署

### 2.2 mevius2 policy.pt 精确布局（已通过 agent 深挖确认）

- **网络**：actor-only MLP `Linear(34→256)·ELU·Linear(256→128)·ELU·Linear(128→64)·ELU·Linear(64→12)`，无 RNN，float32
- **obs（34维）**：`ang_vel(3)·0.25 | gravity_b(3) | cmd(3)·[2,2,0.25] | (dof_pos−default)(12)·sym | dof_vel(12)·0.05·sym | is_standing(1)`，clip ±100
- **action（12维）**：`target = DEFAULT_ANGLE + 0.2·(action·sym)`，URDF 限位保护
- **四元数顺序 (x,y,z,w)**；`gravity_b = R(q)ᵀ·[0,0,-1]`
- **关节顺序 [BL,BR,FL,FR]×[collar,hip,knee]**，与 CAN 顺序一致，无需重映射
- **时序**：策略 50Hz，PD/CAN 200Hz，decimation 4，kp=50/kd=2，`tau=50·(target−pos)−2·vel`

### 2.3 达妙 DM-IMU-L1 协议

- USB CDC-ACM 虚拟串口，921600，80字节复合上行帧（4子包×20）+ CRC16-CCITT
- 输出：accel(m/s²)、gyro(**°/s**，需转 rad/s)、euler(°)、quaternion(**w,x,y,z**，需转 x,y,z,w)
- 已有 ROS2 Humble 驱动 + 纯 C++ viewer 作为干净参考

### 2.4 RobStride03 协议

- SocketCAN 1Mbps，**29位扩展ID 私有协议**（非 MIT 11位），5个16位字段
- TX ExtID = `(0x01<<24)|(tau_u16<<8)|motor_id`，Data 8字节 BE：p_ref/v_ref/kp/kd
- RX ExtID bits15-8 = 源 motor_id（按此解复用），**无 master_id_offset**（与达妙不同）
- 驱动侧应用 MOTOR_DIR(乘) 与 MOTOR_OFFSET_ANGLE(加减)

## 3. 决策记录（已与用户锁定）

| 决策点 | 选择 | 理由 |
|---|---|---|
| 推理加载方式 | C++ ONNX Runtime（路线1）| "以框架为基础"，沿用原生 `inference_node`+onnxruntime+`obs_manager` |
| 驱动语言结构 | C++ + pybind11 | 与推理层 C++ 一致，保持 `inference_node→robot_lib→driver_lib` 依赖链 |
| **电机层（方案C）** | **复用 EDULITE_A3 `RobstrideCanDriver` + 套 roboparty `motor_driver` 抽象 + motors_py** | 驱动复用成熟实现（过滤/互斥/重试/软启动），外层接口与原框架一致，inference_node 依赖链不变；不走 ros2_control 插件 |
| **仿真（自写 bridge）** | **真机/仿真共用 MotorDriver/RobotInterface 接口，sim_mode 切换后端；mujoco + gz bridge** | 真机/仿真同代码路径，inference_node 无感；mujoco 做策略验证，gz 做 ROS2 集成 |
| 迁移范围 | 仅运动控制 | 多模态感知（livox/FAST_LIO/elevation）暂不在范围 |
| **IMU 定位** | **真机达妙 IMU 仅用于验证驱动+ROS 适配，不接入仿真数据** | 仿真 IMU 由仿真器提供；真机 IMU 独立做驱动正确性直测 |
| **ONNX 导出（独立子agent）** | 参考飞书文档 + `lerobot/openpi/smolvla_on_thor`，独立 Agent 执行 | lerobot 已带 pytorch；导出+数值对齐验证作为独立任务 |

## 4. 方案探讨与权衡

### 4.1 推理层（已选路线1）

- **路线1（选）**：`policy.pt` → 导出 ONNX → 沿用 `inference_node`+onnxruntime。obs 构建按 mevius2_utils 用 C++ 重写进 obs_manager。
  - 优点：最贴合框架，复用成熟的 obs_layouts/frame_stacks 机制、服务控制面、`robot_py` 链路
  - 风险：① `.pt→.onnx` 导出需数值对齐验证；② obs_layouts 字符串机制能否表达 mevius2 的 34维布局（含 is_standing、sym_sign）——可能需要扩展 obs_manager
- 路线2（Python torch 直加载）：obs 构建最不易错，但偏离框架 C++ 推理，破坏依赖链 —— 否决

### 4.2 驱动层（已选 C++ + pybind11）

- 新建 `roboparty_imu`（达妙）和 `roboparty_motors`（RobStride03）替换原 submodule 内容，**保持对外 C++ 接口与 pybind 绑定名一致**（`imu_py.IMUDriver`、`motors_py.MotorDriver`、`robot_py.RobotInterface`），上层 inference_node/robot_py 无需改 API
- 电机驱动：C++ 重写 RS03 私有协议（参考 `robstride_ros_sample/src/motor_cfg.cpp` 的 raw SocketCAN + `xiaomimotor_lib.py` 的协议逻辑），套用原 `motor_driver.hpp` 基类抽象
- IMU 驱动：C++ 重写达妙 80字节帧解析（参考 `dm-imu-viewer-cpp`），套用原 imu 接口

### 4.3 本体迁移

- `robot.yaml` 改为 12 关节：motor_id、can0/can1 映射、motor_type=RobStride03、kp/kd=50/2、motor_sign=MOTOR_DIR、motor_zero_offset=MOTOR_OFFSET_ANGLE、close_chain 置空（四足无平行四连杆）、urdf2motor 恒等
- 新增 `inference_mevius2.yaml`：obs_layouts 改 34维、joint_num=12、action_scale=0.2、joint_default_angle=DEFAULT_ANGLE、frame_stacks=1（mevius2 无帧堆叠）、usd2urdf 恒等、clip_cmd
- URDF：复用 `mevius2-master/models/mevius2_dae.urdf`

### 4.4 仿真

- **mujoco**：复用 `mevius2-master/models/scene.xml` + `mevius2_mujoco.xml`，写一个 mujoco 仿真节点（C++ 或 Python bridge）发布 `/joint_states` + `/imu`，订阅推理输出。用于策略/步态验证。
- **Gazebo Harmonic**：为 mevius2 做 URDF→SDF + ros_gz 桥接 + 控制器插件，发布 joint_states/imu，订阅 cmd。用于 ROS2 集成演示。
- **IMU 直测**：独立 launch 真机达妙 IMU 节点，`ros2 topic echo /imu` 验证驱动+ROS 适配，不进仿真回路。

## 5. 子项目分解与顺序

| 子项目 | 内容 | 依赖 |
|---|---|---|
| SP-A 框架骨架迁移 | ROS2 Jazzy 构建系统、start_robot.sh、can_setup、udev、装 colcon+pybind11 | — |
| SP-B 达妙 IMU 包 | C++ 驱动 + imu_py + ROS2 节点 + 真机直测，替换 roboparty_imu | A |
| SP-C RobStride03 电机包 | 方案C：移植 EDULITE_A3 RobstrideCanDriver + 套 motor_driver 抽象 + motors_py + ROS2 服务 | A |
| SP-D1 policy.pt→ONNX（独立子agent） | 参考飞书文档 + lerobot/openpi/smolvla_on_thor，导出 + 数值对齐验证 | A（可并行 B/C） |
| SP-D mevius2 本体+推理 | robot.yaml(12关节)、obs_manager(Mevius2ObsBuilder)、SimMotorBackend、inference_mevius2.yaml | D1, B, C |
| SP-E 仿真+集成测试 | mujoco bridge、gz bridge、IMU 直测、步态测试、测试报告 | D |

SP-A 先行；SP-B/SP-C/SP-D1 三路并行；SP-D 依赖 D1+B+C；SP-E 依赖 D。

## 6. 待确认/风险项

1. **policy.pt → ONNX 导出**：需在有 torch 的环境执行 `torch.jit.load` + `torch.onnx.export`，验证导出后 `(1,34)→(1,12)` 数值与原 `.pt` 一致。当前开发机是否有 torch 待确认。
2. **obs_manager 扩展**：mevius2 的 is_standing 标志、dof_sym_sign、gravity_b 计算可能超出原 obs_layouts 字符串机制，需评估是扩展 DSL 还是直接 C++ 硬编码该策略的 obs。
3. **RS03 V_MAX**：mevius2 降额到 20 rad/s，官方 50。迁移用哪个待定（倾向沿用 mevius2 的 20 以保证策略一致）。
4. **Jetson Thor CAN 接口**：是否有原生 SocketCAN 控制器，还是 USB-CAN 适配器（mevius2 用 USB-CAN + udev）。需确认硬件拓扑。
5. **ROS2 Jazzy 差异**：joy 包、ros_gz 版本、FastDDS 共享内存 profile 在 Jazzy 的兼容性。

## 7. 下一步

1. 用户审阅本头脑风暴 + 需求 + 设计文档
2. 进入 `writing-plans` 技能，为 SP-A 起编写实现计划
3. 按 SP-A→B/C→D→E 顺序实现，用 workflows 并行化独立子任务
