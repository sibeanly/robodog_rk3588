# DM-IMU-L1 C++ 驱动适配 + rviz 可视化 设计

- 日期：2026-06-22
- 分支：`feat/rk3588-humble-port`
- 范围决策：完整 C++ `imu_py` 路线（用户选定）
- 驱动来源：**复用 jeston-main 现成 DAMIAO 驱动**（`/home/orange5plus/code/robodog_jeston-main/src/imu/src/drivers/damiao/`），不自行编写
- 解析策略：现成驱动实现为**滑窗 RID 缓冲解析 + 初始化配置指令**，且解析**原生四元数帧（RID 0x04, 23 字节）** + accel + gyro（即原方案 A 的超集，等价于方案 C）

## 背景与问题

- DM-IMU-L1 已真实接入：`/dev/ttyACM0`（idVendor 6877 / idProduct 4d55 / serial DMIMU20250212）。
- `src/imu` 子模块 pin 的提交 `2cabb7de`（带 DAMIAO 驱动的 Roboparty 私有 fork）在公开 remote 上不存在；本地 `.git/modules/imu` 实际是公开 master `baab9a3d`（仅有 HIPNUC 后端，无 DAMIAO）。
- **最新代码树** `/home/orange5plus/code/robodog_jeston-main/`（非 git 仓库，submodule 内联展开）已包含完整的 DAMIAO C++ 驱动及配套改动，相对公开 master 的 delta 自洽、可直接复用（详见"jeston-main delta"节）。
- 现有原型 `scripts/damiao_imu_node.py` 调 `imu_py.IMUDriver.create_imu(imu_type="DAMIAO")`；一旦把 delta 移入子模块并 build，即可运行。
- 运行时依赖缺失：`/dev/dm_imu` 软链未安装、`ros-humble-imu-tools` 未装、`imu_py` 未 build（`install/setup.bash` 不存在）。
- 参考资源：vendor 纯 Python `dm_imu` 包（`/home/orange5plus/code/dm-imu/02.例程/ROS2-humble例程/`）+ vendor ROS1 C++ 驱动（`02.例程/ROS1-noetic例程/src/dm_imu/`）。

## 目标（本轮交付）

1. 在 roboparty_imu 子模块内新增 DAMIAO C++ 后端，colcon build 产出 `imu_py`。
2. `scripts/damiao_imu_test.py` 验收通过（|q|~1、|a|~9.8 m/s²、ang_vel SI、500-1000Hz）。
3. `/imu` 话题发布 + rviz 可视化（IMU Axes 随设备姿态实时跟随）。
4. 安装 udev 规则与运行时依赖，子模块本地重 pin。

## 非目标（Phase 2 / follow-up）

- 改 `src/inference/config/robot.yaml` 的 `imu_type` 让推理 C++ RobotInterface 走 DAMIAO（需推理节点 build + 真机整机联调，属 Phase 2）。
- 把 `feat/damiao-driver` 分支 push 到外部 fork 并改指 `.gitmodules` url（可复现性 follow-up，不阻塞本机运行）。

## 架构总览

在 roboparty_imu 子模块里新增 **DAMIAO 后端**，与 HIPNUC 后端并列，经 `imu_type` 工厂切换。复用现有 `IMUSerialPort`（异步 RX 回调）与 `IMUDriver` 抽象基类。`imu_py` 经 colcon build 产出后，现有 `scripts/damiao_imu_node.py` 原型直接运行，发布 `/imu`，rviz 可视化。

```
robot.yaml / scripts/damiao_imu_node.py
        │  imu_py.IMUDriver.create_imu(imu_type="DAMIAO", ...)
        ▼
IMUDriver::create_imu  ──► DamiaoImuDriver  (新增)
        │                       │ 复用 IMUSerialPort (异步 RX)
        │                       │ RID 缓冲解析 + CRC16 + 配置指令
        │                       ▼
        │                  quat_/ang_vel_/lin_acc_  (shared_mutex 缓存)
        │ ◄── get_quat/get_ang_vel/get_lin_acc ───
        ▼
/imu (sensor_msgs/Imu)  ──► rviz (rviz_imu_plugin/Imu)
```

## 组件（复用 jeston-main 现成 delta，移植入 src/imu 子模块）

jeston-main 的 `src/imu` 相对公开 master `baab9a3d` 的 delta 自洽，共 6 处（pybind_module.cpp 无需改动——`get_quat`/`get_ang_vel`/`get_lin_acc`/`get_temperature` 是基类继承、已绑定）：

| # | 类型 | 文件 | 变更 |
|---|---|---|---|
| 1 | 新增 | `src/drivers/damiao/{CMakeLists.txt,damiao_imu_driver.cpp,damiao_imu_driver.hpp}` | 整套驱动 |
| 2 | 改 | `src/drivers/CMakeLists.txt` | +`add_subdirectory(damiao)` |
| 3 | 改 | `src/imu_driver.cpp` | 工厂 +DAMIAO 分支 |
| 4 | 改 | `src/protocol/serial/serial_port.hpp` | +`ssize_t write(const uint8_t*, size_t)` 声明 |
| 5 | 改 | `src/protocol/serial/serial_port.cpp` | +`write()` 实现（阻塞直到写完，EINTR 重试） |
| 6 | 改 | 顶层 `CMakeLists.txt` | `damiao_imu` 加入 `target_link_libraries` / `install(TARGETS)` / `ament_export_libraries`（3 处） |

实现方式：把 jeston-main 的这 6 处直接复制覆盖到本仓库 `src/imu` 子模块工作树（公开 master `baab9a3d`），在子模块内提交到本地分支 `feat/damiao-driver`，superproject 重 pin gitlink。

### `DamiaoImuDriver` 行为（复用现成实现，要点记录）

- ctor: `DamiaoImuDriver(imu_id, interface_type, interface, baudrate)`；仅支持 `interface_type=="serial"`，否则抛异常。`IMUSerialPort::open(interface, baudrate)` → `set_serial_callback(serial_rx_cbk)` → `configure_imu()` 发配置指令。
- `serial_rx_cbk`（IMUSerialPort 后台 RX 线程）：字节追加到 `rx_buf_`（`shared_mutex` 加锁，缓冲上限 `MAX_BUF_SIZE=4096`），调 `parse_buffer()`。
- `parse_buffer`：滑窗找 `0x55 0xAA` 帧头；按 `reg` 字节分流——19 字节普通帧（accel/gyro/euler）走 `parse_normal_subpacket`，23 字节四元数帧走 `parse_quat_subpacket`；CRC 失败则丢弃帧头字节重同步。
- CRC16-CCITT（poly 0x1021, init 0xFFFF，表驱动）：普通帧 CRC 覆盖 `frame[0:16]`（含帧头，官方 STM32/ROS1 默认），失败回退 `frame[2:16]`（去帧头，ROS2 兜底）；四元数帧 CRC 覆盖 `frame[0:20]`。
- getter：`get_quat()` 返回原生四元数缓存 `{w,x,y,z}`（无需 euler 推导）；`get_ang_vel()` 返回 rad/s；`get_lin_acc()` 返回 m/s²；`get_temperature()` 返回 0（80 字节帧无温度）。
- `~DamiaoImuDriver`：`serial_->close()`（RX 线程由 IMUSerialPort 析构 join）。

### 配置指令序列（构造时发，`send_command` 每条重复 5 次、间隔 10ms）

`enter_setting(AA 06 01 0D)` → `on_accel(AA 01 14 0D)` → `on_gyro(AA 01 15 0D)` → `on_euler(AA 01 16 0D)` → `on_quat(AA 01 17 0D)` → `set_1000Hz(AA 02 01 00 0D)` → `save(AA 03 01 0D)` → `exit_setting(AA 06 00 0D)`。**保留 quat 输出**（解析 RID 0x04 原生四元数）。

## 帧格式（USB）

普通帧（accel/gyro/euler），19 字节：

| 偏移 | 字段 | 说明 |
|---|---|---|
| [0:2] | `0x55 0xAA` | 帧头 |
| [2] | slave `0x01` | 设备 ID |
| [3] | RID | 0x01=accel(m/s²), 0x02=gyro(deg/s), 0x03=euler(deg) |
| [4:8]/[8:12]/[12:16] | 3×float32 LE | 数据 |
| [16:18] | CRC16 LE | CCITT 0x1021, init 0xFFFF, 覆盖 [0:16] |
| [18] | `0x0A` | 帧尾 |

四元数帧（RID 0x04），23 字节：[4:20] 为 4×float32(w,x,y,z)，CRC 覆盖 [0:20]，帧尾 `0x0A` 于 [22]。

## 单位与坐标系（关键契约，现成驱动已满足）

输出须匹配现有管线契约（与 HiPNUC 一致、与 `damiao_imu_test.py` 验收一致）：

- `get_quat()` → `{w,x,y,z}` 单位四元数（**原生板载 EKF 四元数**，非 euler 推导——精度优于原方案 A）。
- `get_ang_vel()` → `{x,y,z}` **rad/s**（驱动把 DM 的 deg/s ×0.0174532925，同 HiPNUC `DEG_TO_RAD`）。
- `get_lin_acc()` → `{x,y,z}` **m/s²**（驱动按 m/s² 直存——假定模块已输出 SI；静止时 |a|≈9.8，与 `damiao_imu_test.py` 验收一致）。
- `get_temperature()` → 0（80 字节帧无温度字段）。
- `frame_id` = `imu_link`（与现有原型、sim bridge 一致）。

## 数据流 / 话题 / QoS

- `damiao_imu_node.py`（现有原型，不改逻辑）：200Hz 轮询 `imu.get_quat/ang_vel/lin_acc` → 发布 `/imu`（`sensor_msgs/Imu`，**RELIABLE/KeepLast(10)/Volatile**，frame_id `imu_link`，填 orientation+angular_velocity+linear_acceleration）。
- rviz：`rviz_imu_plugin/Imu` 显示订阅 `/imu` Reliable（与发布端 QoS 匹配，避免 "error subscribing"）；Fixed Frame `world`，配 `static_transform_publisher world→imu_link`。

## 环境与启动接线

需安装/启用的运行时依赖（当前都缺）：

1. **udev 规则**：`sudo cp assets/99-dm-imu-jetsonthor.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger` → 产出 `/dev/dm_imu`。
2. **rviz IMU 插件**：`sudo apt install ros-humble-imu-tools`（提供 `rviz_imu_plugin/Imu` 显示类）。
3. **build**：`colcon build --symlink-install`（在 superproject 根），产出 `imu_py`。
   - 注：C++ 驱动走 `IMUSerialPort` 原生 termios，**不依赖 pyserial**；pyserial 仅在用 vendor 纯 Python dm_imu 调试时需要（本轮不涉及）。

启动（复用现有原型，不新建脚本）：

- `./scripts/run_damiao_imu.sh`（起 IMU 节点 + 静态 tf）+ 另开 `rviz2 -d assets/damiao_imu.rviz`。
- 或 `ros2 launch scripts/damiao_imu_rviz.launch.py`（一键起节点+tf+rviz）。

## 子模块重 pin 策略

记录的 gitlink `2cabb7de` 不可达。做法：

1. 把 jeston-main 的 6 处 delta 复制覆盖到 `src/imu` 子模块工作树（当前为公开 master `baab9a3d`）。
2. 在 `.git/modules/imu` 内新建分支 `feat/damiao-driver`，提交这些改动。
3. superproject `git add src/imu` 更新 gitlink 到新提交，提交到当前 `feat/rk3588-humble-port` 分支。
4. `.gitmodules` url 暂留公开 remote（克隆用）；新提交仅本地存在——为可复现，后续需 push `feat/damiao-driver` 到用户控制的 fork 并把 `.gitmodules` url 改指该 fork（记为 follow-up，不阻塞本机运行）。

## 测试与验收

- **单元级**：`scripts/damiao_imu_test.py`（现有）——创建 DAMIAO IMU、读 100 样本、验 |q|~1、|a|~9.8 m/s²、ang_vel 静止≈0/倾转 1-3 rad/s（非 60-180 deg/s）、频率 500-1000Hz。
- **话题级**：`ros2 topic hz /imu`（应 ~200Hz）、`ros2 topic echo /imu --once`（字段非零、quat 归一）。
- **可视化级**：rviz 中 IMU Axes 随设备姿态实时跟随；倾转设备确认轴向正确。
- **回归**：HiPNUC 路径不受影响（工厂 `else if` 不动 HIPNUC 分支；仅当 `imu_type=="DAMIAO"` 走新驱动）。

## 风险

- **USB 浮点单位未文档化**：PDF 只明确 euler 为度、gyro 为 deg/s；accel 浮点单位未标。现成驱动按 m/s² 直存（假定 SI），由 `damiao_imu_test.py` 实测验收（静止 |a|≈9.8 即 SI 正确；若 ≈1.0 则仍是 g，需加 ×9.8）。
- **四元数帧 CRC 覆盖范围**：23 字节 quat 帧 CRC 覆盖 [0:20]，与 19 字节帧的 [0:16] 不同；现成驱动已按 jeston-main 实测设定并带去帧头回退。
- **CRC 是否含帧头**：现成驱动默认含帧头（`frame[0:16]`）并有去帧头回退，与 vendor 双参考一致。
- **配置指令写 flash**：`save` 指令写非易失存储，init 略慢（configure 序列 ~0.6s + exit 后等 300ms）；属一次性配置，符合 vendor 既有做法。
- **子模块提交仅本地**：本机可运行，但其他克隆者拉不到 DAMIAO 驱动，直到 follow-up push fork + 改 `.gitmodules`。
- **/imu 发布者冲突**：本轮仅 `damiao_imu_node` 发布 `/imu`，不与 inference/sim bridge 同时运行（三者按运行时择一），无冲突。
- **`write()` 依赖**：DAMIAO 驱动依赖 `IMUSerialPort::write()`，该方法是 jeston-main delta 的一部分（公开 master 没有），必须连同 serial_port.{hpp,cpp} 一起移植，否则编译失败。
