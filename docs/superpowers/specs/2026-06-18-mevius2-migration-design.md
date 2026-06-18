# 设计文档：mevius2 四足机器人 ROS2 Jazzy 部署框架

> 日期：2026-06-18
> 状态：待审阅
> 关联：头脑风暴 `2026-06-18-brainstorming-mevius2-migration.md`、需求 `2026-06-18-mevius2-migration-requirements.md`

## 1. 设计总则

保留 roboparty_deploy 的"集成编排层 + 独立子模块"架构与 C++ ONNX 推理范式，仅替换三个底层实现：IMU 驱动、电机驱动、本体配置/策略。对外 C++ 接口与 pybind 绑定名保持不变，使上层 `inference_node`/`robot_py` 改动最小。

```
┌─────────────────────────────────────────────────────────┐
│ inference_node (C++, ONNX Runtime)  ← 改 obs_manager/配置 │
│   robot_py.RobotInterface  ← 复用,配置改 12 关节           │
├─────────────────────────────────────────────────────────┤
│ roboparty_motors (C++ + motors_py)  ← 重写 RS03 协议       │
│ roboparty_imu    (C++ + imu_py)     ← 重写达妙协议         │
├─────────────────────────────────────────────────────────┤
│ SocketCAN (RS03) | USB CDC-ACM (达妙 IMU) | 仿真器         │
└─────────────────────────────────────────────────────────┘
```

## 2. 模块设计

### 2.1 roboparty_imu（达妙 DM-IMU-L1）

**接口（与原一致）**：
```cpp
class IMUDriver {
public:
    static std::shared_ptr<IMUDriver> create_imu(
        const std::string& imu_type, const std::string& interface,
        const std::string& interface_type, int baudrate, int imu_id);
    bool init(); bool deinit();
    // 返回四元数(x,y,z,w)、角速度(rad/s)、线加速度(m/s²)、温度
    IMUData read();
};
```
pybind11 导出 `imu_py.IMUDriver`。

**实现要点**：
- 串口：raw termios 或 boost.asio（参考 `dm-imu-viewer-cpp`），921600，8N1
- 帧解析：80字节复合帧 = 4子包×20，同步头匹配，CRC16-CCITT(poly 0x1021, init 0xFFFF)校验
- 单位转换：gyro °/s→rad/s（×π/180）；quaternion (w,x,y,z)→(x,y,z,w)
- 后台读线程填充双缓冲，主线程 `read()` 取最新
- ROS2 节点 `imu_node`：参数 port/baud/frame_id/freq，发布 `sensor_msgs/Imu`（best-effort QoS），可配置输出频率

**真机直测（FR-2.6）**：独立 launch 仅起 `imu_node`，`ros2 topic hz /imu` 验证频率，`ros2 topic echo /imu` 验证姿态（手动翻转 IMU 看四元数/重力方向变化）。

### 2.2 roboparty_motors（RobStride03）

**接口（与原一致）**：
```cpp
class MotorDriver {
public:
    static std::shared_ptr<MotorDriver> create_motor(
        int motor_id, const std::string& iface_type, const std::string& can_iface,
        const std::string& motor_type, int motor_model, int master_id_offset,
        double motor_zero_offset);
    void set_control_mode(MotorControlMode_e mode); // NONE/MIT/POS/SPD
    void motor_mit_cmd(double f_p, double f_v, double f_kp, double f_kd, double f_t);
    void motor_pos_cmd(double pos, double spd, bool ignore_limit);
    void motor_spd_cmd(double spd);
    void lock_motor(); void unlock_motor(); void set_motor_zero();
    void refresh_motor_status();
    double get_motor_pos/spd/current/temperature();
};
```
pybind11 导出 `motors_py.MotorDriver`。

**实现要点（方案C：复用 EDULITE_A3 驱动 + 套 roboparty 接口）**：
- **复用 EDULITE_A3 的 `RobstrideCanDriver`**（C++，raw SocketCAN `PF_CAN/SOCK_RAW/CAN_RAW`，1Mbps，29位扩展ID私有协议）。该驱动已具备：`CAN_RAW_FILTER` 硬件过滤（只收 comm_type=2 反馈帧）、发送互斥锁、10ms 超时 + 2 次重试、8KB 发送缓冲、三阶段软启动（清故障→Kp=0 软使能→位置保持）。源：`EDULITE_A3/el_a3_ros/el_a3_hardware/src/robstride_can_driver.cpp`，zread 文档 `9-robstride私有协议解析` / `17-rsa3hardwareinterface`。
- RS03 私有协议（29位扩展ID）：
  - TX control：ExtID=`(0x01<<24)|(tau_u16<<8)|motor_id`，Data BE = p_ref/v_ref/kp/kd（各16位）
  - RX status：按 ExtID bits15-8 解复用源 motor_id，Data BE = pos/vel/torque/temp
  - 线性映射 `physical = (raw/65535)*(max-min)+min`（signed [-X,X] / unsigned [0,X]）
  - RS03 限位：P=±4π, V=±20(mevius2降额), KP=0..5000, KD=0..100, T=±60
- **外层套 roboparty `motor_driver` 抽象**：把 `RobstrideCanDriver` 适配为 `MotorDriver` 子类（`RobStrideMotorDriver`），实现 `create_motor`/`motor_mit_cmd`/`motor_pos_cmd`/`lock/unlock`/`set_motor_zero`/`refresh_motor_status`/`get_motor_*` 等。RS03 的使能/失能/清故障映射到 comm_type 3/4，set_zero 映射到 comm_type 6，模式切换映射到 param 0x7005。motor_sign(乘)与 motor_zero_offset(加减) 在适配层 TX/RX 应用。master_id_offset 参数保留但 RS03 不用（传 0），保持接口签名一致。
- pybind11 导出 `motors_py.MotorDriver`，**接口与原 roboparty_motors 完全一致** → inference_node/robot_py 依赖链不变。
- ROS2 服务节点 `motors_node`：`/init_motors`、`/deinit_motors`、`/clear_errors`、`/set_zeros`、`/reset_joints`、`/refresh_joints`、`/read_joints`
- **mock 仿真后端**：`MotorDriver` 抽象新增一个 `MockMotorDriver`（或 `SimMotorBackend`）实现，接口与真机一致，内部不碰 CAN，而是对接仿真 bridge（见 §2.5）。`robot_py.RobotInterface` 增加 `sim_mode` 参数切换真机/仿真后端，inference_node 完全无感。

### 2.3 roboparty_inference（mevius2 适配）

**模型**：`policy.pt`→`policy_mevius2.onnx`（导出脚本，torch.jit.load + torch.onnx.export，验证 (1,34)→(1,12)）。

**obs_manager 改造**（关键）：
原框架用 `obs_layouts` 字符串 DSL 拼装观测。mevius2 的 34维布局含 is_standing 标志、dof_sym_sign、gravity_b 计算，超出纯字符串表达。**方案**：为 mevius2 策略新增一个 C++ obs 构建路径（`Mevius2ObsBuilder`），硬编码该策略的 obs 逻辑，由 `inference_mevius2.yaml` 的 `obs_layout: "mevius2"` 触发。保留原 DSL 路径不动。

obs 构建（C++）：
```
ang_vel[3]      = imu.gyro(rad/s, body frame) × 0.25
gravity_b[3]    = R(quat_xyzw)ᵀ · [0,0,-1]
cmd[3]          = [lx,ly,yaw] × [2.0, 2.0, 0.25]
dof_pos[12]     = (joint_pos - DEFAULT_ANGLE) × sym_sign
dof_vel[12]     = joint_vel × 0.05 × sym_sign
is_standing[1]  = float(norm(cmd) < 0.03)
clip ±100
```
- 四元数顺序 (x,y,z,w)
- sym_sign = [1,1,1, -1,1,1, 1,1,1, -1,1,1]
- 关节顺序 [BL,BR,FL,FR]×[collar,hip,knee]，与 CAN/motor 顺序一致，无重映射

**action 处理**：
```
action = policy(obs)            // (1,12)
action = action × sym_sign
action = clip(action, ±100)
target = DEFAULT_ANGLE + 0.2 × action   // ACTION_SCALE=0.2
// URDF 限位保护：若实际角度越限则钳位
// PD: tau = 50·(target-pos) - 2·vel  (kp=50, kd=2, v_ref=0, tau_ff=0)
//     实际下发 motor_mit_cmd(target, 0, 50, 2, 0)
```

**时序**：策略 50Hz，PD/CAN 200Hz，decimation 4。安全：IMU 失效(>0.1s)或 R[2,2]<0.6 时 kp=kd=0。

**配置文件**：
- `config/robot.yaml`：12 关节电机拓扑 + kp/kd/sign/offset（见 §2.4）
- `config/inference_mevius2.yaml`：model_names=["policy_mevius2.onnx"], obs_layout:"mevius2", joint_num:12, action_scale:0.2, frame_stacks:1, joint_default_angle, clip_cmd, joint_limits
- launch `inference.launch.py` 的 `configs` 列表指向 `inference_mevius2.yaml`

### 2.4 robot.yaml（mevius2 12 关节）

```yaml
imu:
    imu_type: "DAMIAO"
    imu_interface_type: "serial"
    imu_interface: "/dev/dm_imu"                 # udev 符号链接 → ttyACM0 (DM-IMU-L1 CDC-ACM)
    baudrate: 921600
motors:
    motor_id: [10,11,12, 7,8,9, 4,5,6, 1,2,3]   # 按 [BL,BR,FL,FR] 顺序填物理 CAN_ID
    motor_interface_type: ["can","can"]
    motor_interface: ["can0","can1"]            # 原生 mttcan SocketCAN
    motor_num: [6, 6]                            # can0=BL+BR(idx0-5), can1=FL+FR(idx6-11)
    motor_type: ["RobStride03","RobStride03"]
    motor_model: [0,0,0,0,0,0,0,0,0,0,0,0]
    master_id_offset: 0                          # RS03 不用
    motor_zero_offset: [ <12个 MOTOR_OFFSET_ANGLE> ]
robot:
    type: "mevius2"
    kp: [50.0 ×12]
    kd: [2.0 ×12]
    motor_sign: [ <12个 MOTOR_DIR> ]             # [1,-1,-1, 1,1,1, -1,-1,-1, -1,1,1]
    close_chain_motor_idx: []                    # 四足无闭链
    urdf2motor: [0..11]                          # 恒等
    extrinsic_R: [1,0,0, 0,1,0, 0,0,1]           # 按 IMU 安装方向调整
```

> 注意 motor_id 数组顺序：roboparty 框架按 motor_id 数组索引驱动，obs/action 也按此索引。mevius2 关节顺序 [BL,BR,FL,FR]，对应 CAN_ID=[10,11,12,7,8,9,4,5,6,1,2,3]。因此 `motor_id` 数组应按 [BL,BR,FL,FR] 顺序填入对应物理 CAN ID：`[10,11,12, 7,8,9, 4,5,6, 1,2,3]`，can0 持有 BL+BR（idx0-5），can1 持有 FL+FR（idx6-11）。`motor_num=[6,6]`。

### 2.5 仿真（自写 bridge 路线，与方案C配套）

**设计原则**：真机与仿真**共用同一套 `MotorDriver`/`RobotInterface` 接口**，仅切换后端。inference_node 不感知是真机还是仿真——这是 roboparty 框架"配置驱动"哲学的延续，也避免仿真与真机下发路径不一致。仿真 IMU 由仿真器提供，**不接入真机 IMU**（真机 IMU 仅独立直测，见 §2.1）。

**SimBackend 抽象**：`RobotInterface(sim_mode=true)` 使用 `SimMotorBackend`（实现 `MotorDriver` 接口），其 `read()`/`apply_action()` 通过 ROS2 topic 与仿真 bridge 进程交互：
- 订阅仿真器发布的 `/joint_states`（12关节 pos/vel/effort）+ `/imu`（仿真 IMU）回填状态
- 把推理目标（关节角度）发布到 `/joint_targets`，由 bridge 转成仿真器控制量
- PD 在仿真器侧执行（`tau=50·(target-pos)-2·vel`），与真机电机固件 PD 一致

**mujoco bridge**（FR-5.1，策略/步态验证）：
- 独立 Python 进程（rclpy + mujoco，用系统 python3.12 装 `mujoco`），复用 mevius2 mujoco 逻辑
- 加载 `mevius2-master/models/scene.xml`（复制到 `assets/mujoco/`）
- 200Hz 物理步进，发布 `/joint_states`+`/imu`，订阅 `/joint_targets`，在 mujoco 内 PD 驱动 actuator
- 暴露 `--sim` 启动：inference_node 以 sim_mode 启动，SimBackend 接 mujoco bridge

**Gazebo Harmonic bridge**（FR-5.2，ROS2 集成演示）：
- 独立进程：gz sim 加载 mevius2 URDF/SDF，`ros_gz_bridge` 桥接 `/joint_states`、`/imu`、`/joint_targets`
- 控制器：自定义 gz 插件或 ros2_control JointGroupPositionController，PD 增益 50/2（与真机一致）
- 与 mujoco bridge 共用同一 SimBackend 接口，差异仅在 bridge 内部后端
- **不走 gz ros2_control 硬件接口插件**——那会引入与方案C不一致的下发路径（真机走 motors_py，仿真走控制器话题）。保持真机/仿真同接口。

**IMU 直测**（FR-2.6，独立）：
- `ros2 launch roboparty_imu damiao_imu_test.launch.py` 仅起 imu_node
- 不进仿真回路，纯验证驱动+ROS 适配

## 3. 数据流

### 3.1 仿真回路（mujoco / gz 共用）
```
sim_bridge ──/joint_states──> [SimMotorBackend.read] ──> inference_node (obs)
          └──/imu────────────> [SimIMU]
inference_node ──/joint_targets──> sim_bridge ──(PD in sim)──> actuator
joy_node ──/joy──> inference_node (cmd_vel, mode)
```
inference_node 通过 RobotInterface(sim_mode=true) 读状态/下发目标，与真机代码路径一致，仅后端不同。

### 3.2 真机回路（后续阶段）
```
RobStrideMotorDriver (CAN) ──> [RobotInterface(sim_mode=false)] ──> inference_node
imu_node ──/imu──> inference_node
joy_node ──/joy──> inference_node
```

## 4. 文件结构（新增/改动）

```
roboparty_deploy/
├── src/
│   ├── imu/            ← 重写为达妙驱动（替换 submodule 内容）
│   │   ├── src/damiao_imu_driver.{cpp,hpp}
│   │   ├── src/imu_node.cpp
│   │   ├── src/pybind_module.cpp
│   │   └── launch/damiao_imu_test.launch.py
│   ├── motors/         ← 方案C：复用 EDULITE_A3 驱动 + 套 roboparty 接口
│   │   ├── src/drivers/robstride/
│   │   │   ├── robstride_can_driver.{cpp,hpp}   ← 移植自 EDULITE_A3
│   │   │   └── robstride_motor_driver.{cpp,hpp} ← 适配 motor_driver 抽象
│   │   ├── src/sim/
│   │   │   └── sim_motor_backend.{cpp,hpp}      ← mock 后端，对接 sim bridge
│   │   ├── src/motors_node.cpp
│   │   └── src/pybind_module.cpp
│   └── inference/      ← 适配 mevius2
│       ├── models/policy_mevius2.onnx     ← 导出生成（独立子agent，见 §5 SP-D1）
│       ├── config/robot.yaml              ← 改 12 关节
│       ├── config/inference_mevius2.yaml  ← 新增
│       ├── src/obs/                       ← 新增
│       │   └── mevius2_obs_builder.{cpp,hpp}
│       └── launch/inference.launch.py     ← configs 指向 mevius2
├── sim/                 ← 仿真 bridge（自写，与方案C配套）
│   ├── mujoco_bridge.py     ← rclpy + mujoco，策略/步态验证
│   └── gz_bridge/           ← gz sim + ros_gz，ROS2 集成演示
├── assets/
│   ├── mevius2_dae.urdf                  ← 复制自 mevius2
│   ├── mujoco/scene.xml, mevius2_mujoco.xml  ← 复制自 mevius2
│   └── 99-auto-up-devs-jetsonthor.rules  ← 新 udev（IMU ttyACM0 绑定）
├── tools/
│   ├── start_robot.sh                    ← Jazzy 适配 + can_setup
│   └── export_policy_onnx.py             ← policy.pt→onnx（参考 lerobot/openpi/smolvla_on_thor）
└── docs/superpowers/specs/               ← 本系列文档
```

## 5. 实现顺序与 workflows 用法

| 阶段 | 内容 | 并行化 |
|---|---|---|
| SP-A | 框架骨架 Jazzy 迁移（构建系统/start_robot/can_setup/udev/装 colcon+pybind11） | 串行基础 |
| SP-B | 达妙 IMU 驱动 + 节点 + 真机直测 | 与 SP-C/SP-D1 并行 workflow |
| SP-C | RS03 电机驱动（移植 EDULITE_A3 RobstrideCanDriver + 套 motor_driver 抽象 + motors_py） + 节点 | 与 SP-B/SP-D1 并行 workflow |
| SP-D1 | **policy.pt→ONNX 导出（独立子 agent）**：参考飞书文档 + `lerobot/openpi/smolvla_on_thor` 代码，导出并数值对齐验证 | 与 SP-B/SP-C 并行 |
| SP-D | obs_manager(Mevius2ObsBuilder) + robot.yaml + inference_mevius2.yaml + SimMotorBackend | 依赖 D1，B/C |
| SP-E | mujoco bridge + gz bridge + IMU 直测 + 步态测试 + 测试报告 | 依赖 D |

**policy.pt→ONNX 导出（SP-D1）作为独立子 agent 执行**：
- **部署后端：onnxruntime CPU（roboparty 原生）**，不走 TensorRT。mevius2 policy 是 4 层 MLP（34→256→128→64→12, ELU），无 attention/mask/in-place 复杂操作，标准 `torch.onnx.export` 即可，不需要 openpi_on_thor 的 TRT 兼容补丁。
- **导出环境：lerobot docker 容器（lerobot0.4.4:v4.0，含 torch）或本地 conda lerobot 环境（也有 pytorch）**，二选一。docker 启动脚本见用户提供的 jetson-ai-lab openpi_on_thor 流程；参考代码 `/home/esi/code/lerobot/openpi/openpi_on_thor/pytorch_to_onnx.py`（导出结构参考）与 `smolvla_on_thor/smolvla_export_onnx.py`。导出脚本本身与 ROS 无关，可在任意有 torch 的环境运行；产出 onnx 后拷入 `src/inference/models/`。
- 参考文档：https://my.feishu.cn/wiki/QG58wsEU7irHSYkTWlIcLD3znIh（飞书，可能需登录）；https://www.jetson-ai-lab.com/tutorials/openpi_on_thor/
- 任务：在 lerobot docker 内将 `mevius2-master/models/policy.pt`（TorchScript actor MLP）导出为 `policy_mevius2.onnx`，opset 17，输入名 "obs" (1,34) float32 → 输出名 "action" (1,12) float32，dynamic_axes 批次维。产出 onnx 拷到 `src/inference/models/`。
- **数值对齐验证**：用主机系统 python3.12 + `onnxruntime`（pip 装，纯 CPU 包）+ `torch`（或直接对比 .pt 在 docker 内、onnx 在主机）跑同随机输入，max abs diff < 1e-5。若主机系统 python 无 torch，则在 docker 内同时跑 .pt 与 onnxruntime 验证（docker 内 onnxruntime 可 pip 装）。
- 产出 `tools/export_policy_onnx.py`（可在 docker 内运行）+ `policy_mevius2.onnx` + 对齐报告 `docs/superpowers/specs/2026-06-18-onnx-export-report.md`。
- 由独立 Agent 执行（非主线程），完成后产物供 SP-D 使用。inference_node 沿用框架自带 onnxruntime C++ lib（aarch64），无需改推理后端。

实现阶段用 Workflow 编排：SP-B/SP-C/SP-D1 三路并行 agent；SP-D 的 obs 构建用 adversarial verify（C++ obs 输出 vs Python mevius2_utils 参考输出对齐）；SP-E 仿真测试用 loop-until-dry 跑步态。

## 6. 风险与缓解

| 风险 | 缓解 |
|---|---|
| policy.pt→ONNX 数值不一致 | SP-D1 独立子 agent，导出脚本内置数值对齐测试；参考 lerobot/openpi/smolvla_on_thor 已验证的导出流程 |
| obs_manager DSL 不支持 mevius2 布局 | 新增硬编码 `Mevius2ObsBuilder` 路径，不动原 DSL |
| RS03 V_MAX 取值 | 沿用 mevius2 的 20 rad/s 保证策略一致，可配 |
| EDULITE_A3 驱动移植适配 | RobstrideCanDriver 协议层通用，仅适配 motor_driver 接口；保留其软启动/过滤/重试 |
| 真机/仿真接口不一致 | SimMotorBackend 与 RobStrideMotorDriver 实现同一 MotorDriver 抽象，inference_node 无感 |
| Jazzy ros_gz/joy 版本 | SP-A 验证依赖可用性，必要时锁定版本 |
| C++ obs 与 Python 参考不一致 | adversarial verify：同一 (quat,gyro,joint,cmd) 输入，C++ obs 与 mevius2_utils 输出逐元素比对 |
| conda py3.13 污染 ROS 构建 | 所有 ROS Python 强制用系统 /usr/bin/python3 (3.12)，不激活 conda |

## 6.1 未来选项（当前不纳入）

**TensorRT 量化加速**：`/home/esi/code/docs/` 下有完整的 SmolVLA Jetson Thor TRT 量化文档（`00-overview` ~ `05-pipeline`，含 fp16/fp8/nvfp4 标定管线）。当前阶段**不走 TRT**——mevius2 policy 是 4 层 MLP（参数量极小），onnxruntime CPU 推理已足够 50Hz 实时性，TRT 量化收益甚微且引入工具链复杂度。若未来策略规模增大或需 GPU 加速，可参考该批文档走 TRT 路线（导出 ONNX 后 `build_engine.sh` 转引擎 + 标定），届时需把 inference_node 从 onnxruntime C++ 切换为 TRT C++ API。此为后续阶段事项，SP-A~E 不涉及。

## 7. 环境探查结论（已查清）

实施前对 Jetson Thor 开发机（Ubuntu 24.04 / ROS2 Jazzy）的探查结果：

| 项 | 结论 | 对设计的影响 |
|---|---|---|
| **CAN 接口** | 原生 mttcan 控制器（`nvidia,tegra264-mttcan`），4 路 can0~can3，纯 SocketCAN，当前 DOWN | RS03 驱动直接用 raw socket；**无需 USB-CAN 适配器、无需 USB udev 绑定**；仅需 can_setup 把 can0/can1 `ip link set type can bitrate 1000000` + up。mevius2 用 2 路，富余 2 路 |
| **IMU 设备** | `/dev/ttyACM0`（DM-IMU-L1 是 CDC-ACM，非 ttyUSB），用户在 dialout 组 | `robot.yaml` 的 `imu_interface` = `/dev/ttyACM0`；udev 规则可固定为 ttyACM0 或按序列号绑定 |
| **ROS2** | Jazzy 已装（`/opt/ros/jazzy`），ament-cmake、joy、robot-state-publisher、yaml-cpp、fmt、spdlog、boost 均就绪 | SP-A 构建系统基础具备 |
| **colcon** | **未安装** | 需 `sudo apt install python3-colcon-common-extensions` |
| **pybind11** | **未安装** | 需 `pip install pybind11`（或 apt `python3-pybind11`），与系统 Python 3.12 对齐 |
| **torch/onnx** | **未安装** | SP-D 需 `pip install torch onnx` 导出 policy.pt→ONNX（注意用系统 python3.12 而非 conda 3.13） |
| **onnxruntime (C++)** | 框架第三方自带 libonnxruntime 1.21.0 tgz | C++ 推理不依赖 python onnxruntime；aarch64 tgz 已在 thirdparty |
| **mujoco** | **未安装** | SP-E mujoco bridge 需 `pip install mujoco mujoco-python-viewer` |
| **Gazebo Harmonic** | **未安装**（ros-gz 缺） | SP-E gz 集成需 `sudo apt install ros-jazzy-ros-gz` + gz sim |
| **Python 环境** | miniconda Python 3.13 在前；ROS2 Jazzy 用系统 Python 3.12 | **colcon build 与 ROS 节点必须用系统 /usr/bin/python3 (3.12)**，避免 conda 3.13 污染（ament_python 包、pybind11 绑定均依赖 3.12） |
| **架构/核数** | aarch64，14 核 | 编译并行 `colcon build --parallel-workers`；aarch64 onnxruntime tgz 已就绪 |

## 8. 锁定的设计选择

基于环境探查与讨论，以下设计选择锁定：

1. **CAN 拓扑**：原生 mttcan SocketCAN，can0=BL+BR，can1=FL+FR。can_setup 在 start_robot.sh 内 bring up（bitrate 1M）。无需 USB udev 绑定 CAN；IMU udev 绑定 ttyACM0（可选，按 CDC 序列号）。
2. **电机层（方案C）**：复用 EDULITE_A3 的 `RobstrideCanDriver`（C++，成熟：硬件过滤/互斥/重试/软启动），外层套 roboparty `motor_driver` 抽象 + `motors_py` pybind，**接口与原框架完全一致**，inference_node/robot_py 依赖链不变。不走 ros2_control 硬件接口插件路线（避免与 roboparty 接口链路冲突）。
3. **obs_manager**：新增 `Mevius2ObsBuilder` 硬编码路径（`obs_layout: "mevius2"` 触发），不扩展原 DSL。C++ obs 输出与 mevius2_utils Python 参考逐元素对齐验证（adversarial verify）。
4. **仿真（自写 bridge）**：真机/仿真共用 `MotorDriver`/`RobotInterface` 接口，`sim_mode` 切换 `RobStrideMotorDriver`/`SimMotorBackend` 后端，inference_node 无感。mujoco bridge（Python rclpy+mujoco，系统 py3.12）做策略/步态验证；gz bridge 做 ROS2 集成演示，两者共用 SimBackend。仿真 IMU 来自仿真器，不接真机 IMU。
5. **RS03 V_MAX**：沿用 mevius2 的 20 rad/s（保证策略一致），driver 内可配。
6. **policy.pt→ONNX（独立子 agent，SP-D1）**：参考飞书文档 https://my.feishu.cn/wiki/QG58wsEU7irHSYkTWlIcLD3znIh + `/home/esi/code/lerobot/openpi/smolvla_on_thor` 代码，在 pytorch 环境（lerobot 框架已装）导出 `policy.pt`→`policy_mevius2.onnx`，(1,34)→(1,12)，内置数值对齐测试（逐元素误差 < 1e-5）。由独立 Agent 执行，产出 `tools/export_policy_onnx.py` + onnx + 对齐报告。
7. **Python 环境**：所有 ROS 相关 Python（colcon、节点、pybind、mujoco bridge）统一用系统 `/usr/bin/python3` (3.12)，不激活 conda。ONNX 导出（SP-D1）用 lerobot 自带 torch 环境。
8. **参考源**：roboparty zread（架构）、dm-imu zread（IMU 协议）、mevius2 zread（obs/PD/协议/关节命名，交叉验证）、EDULITE_A3 zread（RobstrideCanDriver 复用 + 软启动/mock 模式）、robstride RobStride 仓库（协议官方参考）、lerobot/openpi/smolvla_on_thor + 飞书文档（ONNX 导出）。
