# DM-IMU-L1 C++ 驱动适配 + rviz 可视化 设计

- 日期：2026-06-22
- 分支：`feat/rk3588-humble-port`
- 范围决策：完整 C++ `imu_py` 路线（用户选定），DAMIAO 驱动**自行编写**
- 解析策略：方案 A — RID 缓冲解析 + 初始化配置指令

## 背景与问题

- DM-IMU-L1 已真实接入：`/dev/ttyACM0`（idVendor 6877 / idProduct 4d55 / serial DMIMU20250212）。
- `src/imu` 子模块 pin 的提交 `2cabb7de`（带 DAMIAO 驱动的 Roboparty 私有 fork）在公开 remote 上不存在；本地 `.git/modules/imu` 实际是公开 master `baab9a3d`（仅有 HIPNUC 后端，无 DAMIAO）。
- 现有原型 `scripts/damiao_imu_node.py` 调 `imu_py.IMUDriver.create_imu(imu_type="DAMIAO")`，但 DAMIAO 后端从未存在 → 无法运行。
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

## 组件（新增文件，均在 src/imu 子模块内）

镜像 `src/drivers/hipnuc/` 结构：

| 文件 | 作用 |
|---|---|
| `src/drivers/damiao/damiao_imu_driver.hpp` | `class DamiaoImuDriver : public IMUDriver`，镜像 HipnucIMUDriver 接口 |
| `src/drivers/damiao/damiao_imu_driver.cpp` | 构造：开串口→发配置指令→`set_serial_callback`；回调内缓冲解析；getter 返回缓存 |
| `src/drivers/damiao/dm_crc.h` / `dm_crc.c` | CRC16 CCITT(0x1021, init 0xFFFF) 表驱动（取自 vendor `bsp_crc.cpp` / `dm_crc.py`，与 PDF 附录四一致） |
| `src/drivers/damiao/CMakeLists.txt` | 仿 hipnuc：`add_library(damiao_imu STATIC ...)`，链 `imu_protocol` |

修改的现有文件（子模块内）：

- `src/drivers/CMakeLists.txt`：加 `add_subdirectory(damiao)`。
- `src/imu_driver.cpp`：`#include "drivers/damiao/damiao_imu_driver.hpp"` + `else if (imu_type=="DAMIAO") return std::make_shared<DamiaoImuDriver>(...)`。
- 顶层 `CMakeLists.txt`：`target_link_libraries(imu PUBLIC ... damiao_imu)`（在 `hipnuc_imu` 旁）。

### `DamiaoImuDriver` 关键成员/行为（镜像 HipnucIMUDriver）

- ctor: `DamiaoImuDriver(imu_id, interface_type, interface, baudrate)`；`interface_type=="serial"` 时 `IMUSerialPort::open(interface, baudrate)` → 发配置指令序列 → `set_serial_callback([this](data,len){serial_rx_cbk(data,len);})`。
- `serial_rx_cbk`：把字节追加到 `std::vector<uint8_t> buf_`（加锁），调用 `parse_frames_()` 扫描所有完整 19 字节帧。
- `parse_frames_`：找 `0x55 0xAA`；不足 19 字节则保留尾部等下次；校验尾字节 `0x0A` + RID∈{0x01,0x02,0x03} + CRC16（默认含帧头 `frame[0:16]`，失败回退 `frame[2:16]`）；按 RID 写 `lin_acc_/ang_vel_/euler_`。
- getter：`get_quat()` 由缓存的 euler(deg)→quat(ZYX) 实时换算返回；`get_ang_vel()`/`get_lin_acc()` 返回 SI 单位缓存。
- `~DamiaoImuDriver`：关串口、join RX 线程（由 IMUSerialPort 析构处理）。

### 配置指令序列（构造时发，每条重复 5 次、间隔 10ms，取自 vendor ROS1 driver）

`enter_setting(AA 06 01 0D)` → `on_accel(AA 01 14 0D)` → `on_gyro(AA 01 15 0D)` → `on_euler(AA 01 16 0D)` → `off_quat(AA 01 07 0D)` → `set_1000Hz(AA 02 01 00 0D)` → `save(AA 03 01 0D)` → `exit_setting(AA 06 00 0D)`。

## 帧格式（USB，19 字节）

| 偏移 | 字段 | 说明 |
|---|---|---|
| [0:2] | `0x55 0xAA` | 帧头 |
| [2] | device ID | 设备 ID |
| [3] | RID | 0x01=accel, 0x02=gyro, 0x03=euler(deg) |
| [4:8] / [8:12] / [12:16] | 3×float32 LE | 数据 |
| [16:18] | CRC16 LE | CCITT 0x1021, init 0xFFFF |
| [18] | `0x0A` | 帧尾 |

`VALID_RIDS = {0x01, 0x02, 0x03}`。四元数帧 RID 0x04（23 字节）本轮不开（off_quat）。

## 单位与坐标系（关键契约）

输出须匹配现有管线契约（与 HiPNUC 一致、与 `damiao_imu_test.py` 验收一致）：

- `get_quat()` → `{w,x,y,z}` 单位四元数（euler ZYX 推导）。
- `get_ang_vel()` → `{x,y,z}` **rad/s**（DM 若输出 deg/s 则 ×0.01745329，同 HiPNUC `DEG_TO_RAD`）。
- `get_lin_acc()` → `{x,y,z}` **m/s²**（若输出 g 则 ×9.8，同 HiPNUC `GRA_ACC`；静止时 |a|≈9.8）。
- `get_temperature()` → ℃（若硬件提供，否则 0）。

DM-IMU-L1 USB 浮点单位 PDF 未明确标注（仅明确 euler 为度）。默认按 "g/deg·s⁻¹ → SI" 转换（与 HiPNUC 一致），由 `scripts/damiao_imu_test.py` 验收确认；若实测硬件本就输出 SI，则去掉转换因子。`frame_id` = `imu_link`（与现有原型、sim bridge 一致）。

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

1. 在 `.git/modules/imu` 里新建分支 `feat/damiao-driver`（基于 master `baab9a3d`），提交新增 DAMIAO 文件。
2. superproject `git add src/imu` 更新 gitlink 到新提交，提交到当前 `feat/rk3588-humble-port` 分支。
3. `.gitmodules` url 暂留公开 remote（克隆用）；新提交仅本地存在——为可复现，后续需 push `feat/damiao-driver` 到用户控制的 fork 并把 `.gitmodules` url 改指该 fork（记为 follow-up，不阻塞本机运行）。

## 测试与验收

- **单元级**：`scripts/damiao_imu_test.py`（现有）——创建 DAMIAO IMU、读 100 样本、验 |q|~1、|a|~9.8 m/s²、ang_vel 静止≈0/倾转 1-3 rad/s（非 60-180 deg/s）、频率 500-1000Hz。
- **话题级**：`ros2 topic hz /imu`（应 ~200Hz）、`ros2 topic echo /imu --once`（字段非零、quat 归一）。
- **可视化级**：rviz 中 IMU Axes 随设备姿态实时跟随；倾转设备确认轴向正确。
- **回归**：HiPNUC 路径不受影响（工厂 `else if` 不动 HIPNUC 分支；仅当 `imu_type=="DAMIAO"` 走新驱动）。

## 风险

- **USB 浮点单位未文档化**：PDF 只明确 euler 为度，accel/gyro 浮点单位未标。默认按 g/deg·s → SI 转换，由 `damiao_imu_test.py` 实测验收；若实测为 SI 则去转换因子。
- **CRC 是否含帧头**：vendor Python 默认含帧头（`frame[0:16]`）并有去帧头回退；本实现同样做双校验兜底，与参考一致。
- **配置指令写 flash**：`save` 指令写非易失存储，init 略慢（~0.2s）；属一次性配置，符合 vendor 既有做法。
- **子模块提交仅本地**：本机可运行，但其他克隆者拉不到 DAMIAO 驱动，直到 follow-up push fork + 改 `.gitmodules`。
- **/imu 发布者冲突**：本轮仅 `damiao_imu_node` 发布 `/imu`，不与 inference/sim bridge 同时运行（三者按运行时择一），无冲突。
