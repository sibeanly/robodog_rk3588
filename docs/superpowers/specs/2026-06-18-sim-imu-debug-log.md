# 仿真与 IMU 可视化调试记录

> 日期：2026-06-18
> 范围：SP-E mujoco 四足仿真 + SP-B 达妙 IMU 真机 rviz 可视化的调试过程
> 关联：测试报告 `2026-06-18-test-report.md`、设计文档 `2026-06-18-mevius2-migration-design.md`

本文档记录仿真与 IMU 可视化从启动到成功的完整 debug 过程，包括每个问题的现象、根因、修复，供后续维护参考。

---

## 一、四足机器人 mujoco 仿真调试

### 1.1 目标
在 ROS2 Jazzy + mujoco 中运行 mevius2 RL 步态策略，验证机器人站立 + 行走。链路：mujoco 物理 → /joint_states + /imu → sim_inference_node（ONNX 推理）→ /joint_targets → mujoco PD 驱动。

### 1.2 依赖安装问题

**现象**：SP-E agent 写好了 `sim/mujoco_bridge.py`、`sim/sim_inference_node.py`，但卡住无法运行——mujoco 和 onnxruntime 未安装。

**根因**：aarch64 Jetson Thor 上系统 python3.12 受 PEP 668 保护（externally-managed），`pip install --user` 被拒。

**修复**：加 `--break-system-packages`（`--user` 装到 ~/.local，本就不碰系统目录，绕过 PEP 668 拦截无实际风险）：
```bash
export http_proxy=http://127.0.0.1:7897 https_proxy=http://127.0.0.1:7897
/usr/bin/python3 -m pip install --user --break-system-packages mujoco onnxruntime
```
结果：mujoco 3.9.0 + onnxruntime 1.27.0 装好。

**onnxruntime GPU 探测 warning（无害）**：
```
[W:onnxruntime:Default, device_discovery.cc:283 GetGpuDevices] Failed to detect devices under "/sys/class/drm/card1"
```
根因：onnxruntime 启动时探测 GPU（用于可选 CUDA EP），Jetson 的 card1/card3 是显示接口非计算 GPU，读不到 vendor。**这是期望行为**——探测失败自动回退 CPUExecutionProvider，符合 onnxruntime CPU 推理设计。修复：sim_inference_node.py 显式 `providers=['CPUExecutionProvider']`，既消除 warning 又强制 CPU。

### 1.3 代码 bug：sim_inference_node 缺 sys import

**现象**：sim_inference_node.py 启动即 NameError。

**根因**：`main()` 用了 `rclpy.init(args=sys.argv)` 但文件头没 `import sys`。

**修复**：补 `import sys`。

### 1.4 mujoco 可视化窗口崩溃（Force quit / 无响应）

**现象**：`ros2 launch mujoco_sim.launch.py use_viewer:=true` 启动后，mujoco 可视化窗口弹出但卡死，系统提示 "Force quit or wait"。但终端日志显示仿真物理 + 推理正常（gravity_b=[0,0,-1] 站立、base xyz 稳定）——即仿真逻辑正常，仅 viewer 窗口冻结。

**根因**：第三方库 `mujoco-python-viewer`（基于 GLFW）的 `MujocoViewer.render()` 在子线程渲染，而 GLFW 窗口要求主线程渲染——线程冲突导致窗口冻结。bridge 的设计是 `rclpy.spin` 在主线程、sim_loop（含 viewer.render）在子线程，违反了 GLFW 约束。

**第一次尝试（失败）**：`test_mujoco_viewer.py` 用第三方 mujoco-python-viewer，API 不匹配报 `AttributeError: 'MujocoViewer' object has no attribute 'is_running'`。

**修复**：改用 **mujoco 官方** `mujoco.viewer.launch_passive`：
- `launch_passive` 立即返回，渲染在**独立线程**，不阻塞主线程 spin
- 主循环只调 `viewer.sync()` 同步数据（非阻塞）
- 官方 API 有 `is_running()` 方法

test_mujoco_viewer.py 验证官方 viewer 能正常显示（窗口弹出，渲染 5 秒正常）。

### 1.5 官方 viewer 退出时 Segmentation fault

**现象**：test_mujoco_viewer.py 运行 5 秒正常显示，退出时 `Segmentation fault (core dumped)`。

**根因**：`launch_passive` 的 viewer 线程在 Jetson（EGL/GLFW 线程清理）退出阶段 segfault。这是退出清理阶段的问题，**运行中可视化正常**。

**修复**：bridge 的 `main()` 退出时**不调 `viewer.close()`**（它触发 segfault），设 `viewer=None` 后直接 destroy_node + shutdown，让 OS 自然回收 viewer 线程。退出时即使 segfault 也无害（仿真已停，数据已存）。

### 1.6 ros2 launch 包名错误

**现象**：`ros2 launch mujoco_sim.launch.py` 报 `'mujoco_sim.launch.py' is not a valid package name`。

**根因**：`ros2 launch` 第一个参数是**包名**，sim 目录未作为 ROS 包安装，不是包。

**修复**：直接跑两个 Python 脚本，不用 launch 包机制：
```bash
/usr/bin/python3 sim/mujoco_bridge.py &     # 带 viewer（DISPLAY=:1 时自动开）
/usr/bin/python3 sim/sim_inference_node.py &
```

### 1.7 后台启动不可靠

**现象**：用 `setsid nohup ... &` 或 harness `run_in_background` 后台启动 bridge/inference，经常静默失败（output 空、进程不在）。

**根因**：非交互后台 shell 的 `source` 环境变量未持久传递，且 GUI 程序（mujoco viewer）在 detached 后台缺 X11 上下文。

**修复**：viewer 需前台/有 tty 的终端跑。最终让用户在终端前台跑两个脚本，稳定。

### 1.8 仿真最终验证（成功）

启动 bridge + inference 后：
- **站立**：零 cmd → gravity_b=[0,0,-1.00]（水平站立），stand=1，target≈DEFAULT_ANGLE，base xyz 稳定（z=0.030, qw=1.0）。机器人稳定站立不倒 ✓
- **行走**：`ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear:{x:0.3}}'` → cmd=[0.6,0,0]（×SCALE_LIN_VEL=2.0），stand=0，策略输出周期摆动（步态），base x 前进 0.11→0.21→0.45→0.70→1.02m（~0.3-0.4 m/s），gravity_b z≈-1（未摔倒）✓

---

## 二、达妙 IMU 真机 rviz 可视化调试

### 2.1 目标
达妙 DM-IMU-L1（/dev/dm_imu）真机数据经 ROS2 发布 `/imu`，rviz2 可视化 IMU 方向坐标轴。

### 2.2 IMU 节点 _pub_count 竞态崩溃

**现象**：damiao_imu_node.py 启动后报 `AttributeError: 'DamiaoImuNode' object has no attribute '_pub_count'`，_read_loop 崩溃，一直 "no IMU data yet"。

**根因**：`__init__` 里 `_read_loop` 线程启动**后**才初始化 `self._pub_count = 0`，线程先跑到 `self._pub_count += 1` 时属性还不存在。

**修复**：把 `self._pub_count = 0` 移到线程启动**前**。

### 2.3 IMU 发布速率失控（13000 Hz）

**现象**：`ros2 topic hz /imu` 显示 13841 Hz，CPU 占满。

**根因**：`_read_loop` 无 sleep，Python while 循环全速轮询发布。达妙硬件 1000Hz，但 Python 循环不限速 → ~13kHz 数据洪流。

**修复**：_read_loop 加 200Hz 限速（`next_t += period; time.sleep(...)` 模式）。200Hz 足够 rviz 可视化和状态显示。

### 2.4 rviz2 无 IMU display 插件

**现象**：rviz2 的 Add → By display type 列表里找不到 Imu 类型；"show unvisualizable topic" 提示 /imu 不可可视化。

**根因**：`ros-jazzy-rviz-default-plugins` 的插件目录 `/opt/ros/jazzy/lib/rviz_default_plugins/` 为空（后证实是正常打包方式——插件在单个 `librviz_default_plugins.so`，不在子目录），且系统没装 IMU 专用显示插件。

**修复**：装 imu_tools（含 rviz_imu_plugin）：
```bash
sudo apt install ros-jazzy-imu-tools
```
装后 `librviz_imu_plugin.so` 就位，提供 `rviz_imu_plugin/Imu` display。

### 2.5 rviz2 配置 Fixed Frame 不生效（base_link）

**现象**：`rviz2 -d assets/damiao_imu.rviz` 启动后 Fixed Frame 仍是 base_link（配置里写的是 world），Global Status 报 "frame base_link does not exist"。

**根因**：我手写的 .rviz 配置结构错误——把 `Visualization Manager` 嵌在 `Panels[0]` 下。标准 rviz2 配置里 `Visualization Manager` 是**顶级 key**（与 `Panels`、`Window Geometry` 平级）。嵌套在 Panels 下导致 rviz2 忽略整个 Visualization Manager，Fixed Frame 用默认 base_link，IMU display 也不加载。

**验证**：对比 `/opt/ros/jazzy/share/rviz_common/default.rviz`，确认顶级结构为 `Panels:` / `Visualization Manager:` / `Window Geometry:`。

**修复**：重写配置，`Visualization Manager` 提到顶级，含 Grid + IMU display，Fixed Frame=world。

### 2.6 IMU display 订阅失败 "error subscribing"（核心问题）

**现象**：Fixed Frame=world 修好后，手动 Add Imu display 选 /imu，仍报 "error subscribing topic"。`ros2 topic info /imu -v` 显示 Subscription count: 0（rviz 在跑却没订阅上）。

**排查过程**：
- 排除 DDS/RMW 隔离：rviz2 和 damiao_imu_node 都 source /opt/ros/jazzy/setup.bash，默认 rmw_fastrtps_cpp，domain 0，RMW 一致。`ros2 node list` 互不可见时才怀疑 DDS，但实际 /imu CLI echo 正常，说明发布端 OK。
- 排除 QoS 不匹配：IMU 节点最初用 BEST_EFFORT，rviz 默认 Reliable → 改 IMU 节点为 RELIABLE（`QoSProfile(reliability=RELIABLE)`）。但 QoS 改了仍 error，说明不是 QoS。
- **rviz2 stderr 日志（关键证据）**：
  ```
  [ERROR] [rviz2]: PluginlibFactory: The plugin for class 'rviz_imu_plugins/Imu' failed to load.
  Error: ... the class rviz_imu_plugins/Imu with base class type rviz_common::Display does not exist.
  Declared types are ... rviz_imu_plugin/Imu rviz_imu_plugin/Mag
  ```

**根因**：配置文件里 IMU display 的插件类名写错——`rviz_imu_plugins/Imu`（**复数 plugins**），实际注册的是 `rviz_imu_plugin/Imu`（**单数 plugin**）。pluginlib 解析不了复数名，Imu display 根本没加载，所以从没创建 /imu 订阅。

**确认确切类名**：从插件描述文件取：
```bash
cat /opt/ros/jazzy/share/rviz_imu_plugin/plugin_description.xml
# <class name="rviz_imu_plugin/Imu" type="rviz_imu_plugin::ImuDisplay" base_class_type="rviz_common::Display">
```
包名是 `rviz_imu_plugin`（单数）。

**修复**：配置一行改动 `Class: rviz_imu_plugins/Imu` → `Class: rviz_imu_plugin/Imu`（commit 5537e0f）。

**验证**：修复后 `ros2 node list` 含 /rviz，`/imu` Subscription count: 1（订阅者 rviz，QoS RELIABLE+VOLATILE 匹配），rviz2 启动日志无错误。rviz 窗口显示红色坐标轴，翻转 IMU 实时跟随 ✓

### 2.7 rviz_default_plugins 空目录的澄清（非缺陷）

调试中一度怀疑 `/opt/ros/jazzy/lib/rviz_default_plugins/` 空是缺陷。后证实：此打包方式下插件在单个 `librviz_default_plugins.so`，子目录本就不存在；插件 xml 和 ament index 都在，rviz2 日志能枚举所有 `rviz_default_plugins/*` 类（Grid/TF/MoveCamera/Orbit 等）正常加载。**不是缺陷，无需重装**。

---

## 三、调试经验总结

| 问题类别 | 典型现象 | 根因模式 | 通用解法 |
|---|---|---|---|
| 依赖安装 | pip 被 PEP 668 拒 | externally-managed | `--user --break-system-packages` |
| 后台启动 | setsid/nohup 静默失败 | 非交互 shell 环境不持久 + GUI 缺 X11 上下文 | GUI/节点前台终端跑，或 harness `run_in_background` |
| GUI 卡死 | viewer 窗口 Force quit | 渲染线程冲突（GLFW 须主线程） | 用官方 `mujoco.viewer.launch_passive`（渲染独立线程） |
| 退出 segfault | viewer 退出崩溃 | EGL/GLFW 线程清理 | 不调 close()，让 OS 回收 |
| ROS2 频率失控 | topic hz 异常高 | 循环无 sleep | 加 period 限速 |
| rviz 配置不生效 | Fixed Frame 用默认值 | Visualization Manager 嵌套位置错 | 必须顶级 key |
| rviz 订阅失败 | error subscribing / sub count 0 | 插件类名拼错（单复数） | 从 plugin_description.xml 取确切类名 |
| QoS 不匹配 | 订阅不上 | pub Best-Effort / sub Reliable | 统一为 RELIABLE（或 display 设 Best Effort） |
| 误判缺陷 | 以为目录空是 bug | 打包方式差异 | 先查 ament index + 插件 xml |

**关键教训**：
1. rviz2 订阅失败时，先看 `rviz2 2>&1` 的 stderr 日志（pluginlib 错误直接显示），比猜 QoS/DDS 高效。
2. 插件类名必须从 `plugin_description.xml` 的 `<class name="...">` 取，不能凭记忆（单复数易错）。
3. rviz2 配置的 `Visualization Manager` 必须是顶级 key，不是 Panels 子项。
4. mujoco 可视化用官方 `launch_passive`，不用第三方 `mujoco-python-viewer`（线程模型不对）。
5. 非交互后台环境启动 GUI 程序不可靠，让用户前台跑最稳。

---

## 四、最终可用命令

### 四足仿真（mujoco + 可视化）
```bash
cd /home/esi/code/roboparty_deploy
conda deactivate 2>/dev/null; unset PYTHONPATH PYTHONHOME
export PATH=/usr/bin:$PATH; export DISPLAY=:1
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 sim/mujoco_bridge.py &          # mujoco 物理桥 + 可视化窗口
/usr/bin/python3 sim/sim_inference_node.py &     # ONNX 策略推理
# 行走：
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
# 停止：
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
```

### IMU 真机 rviz 可视化
```bash
# 终端1：IMU 节点 + tf
bash scripts/run_damiao_imu.sh
# 终端2：rviz2
cd /home/esi/code/roboparty_deploy
conda deactivate 2>/dev/null; unset PYTHONPATH PYTHONHOME
export PATH=/usr/bin:$PATH; export DISPLAY=:1
source /opt/ros/jazzy/setup.bash
rviz2 -d assets/damiao_imu.rviz
```

### 相关文件
- `sim/mujoco_bridge.py` — mujoco 物理桥（官方 launch_passive viewer）
- `sim/sim_inference_node.py` — ONNX 推理节点（CPUExecutionProvider）
- `sim/test_mujoco_viewer.py` — mujoco viewer 独立测试
- `scripts/damiao_imu_node.py` — 达妙 IMU ROS2 发布节点（RELIABLE QoS, 200Hz 限速）
- `scripts/run_damiao_imu.sh` — IMU 节点 + tf 启动脚本
- `assets/damiao_imu.rviz` — rviz2 配置（Fixed Frame world, IMU display `rviz_imu_plugin/Imu`）
