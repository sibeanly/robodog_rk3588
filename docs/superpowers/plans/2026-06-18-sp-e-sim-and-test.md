# SP-E 仿真与集成测试 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox (`- [ ]`).

**Goal:** 在 ROS2 Jazzy 仿真环境中运行 mevius2 RL 步态策略验证(mujoco 优先),完成 IMU 真机直测,产出测试报告。

**Architecture:** 自写 bridge 路线(与 SP-C 方案C配套)。真机/仿真共用 MotorDriver/RobotInterface 接口;mujoco bridge(Python rclpy+mujoco)发布 /joint_states + /imu,订阅 /joint_targets,200Hz PD 驱动;inference_node 通过 SimMotorBackend 读状态/下发目标。仿真 IMU 来自仿真器,不接真机 IMU。

**Tech Stack:** ROS2 Jazzy, Python 3.12 (系统), mujoco, mujoco-python-viewer, rclpy, sensor_msgs/JointState, sensor_msgs/Imu。gz Harmonic(ros-jazzy-ros-gz)做集成演示(可选/后置)。

**前提(SP-A/B/C/D1/D 已完成):** onnx 就位、imu/motors/inference 三包在 Jazzy 编译通过、inference_node 能加载 mevius2 配置。`assets/mujoco/{scene.xml,mevius2_mujoco.xml,meshes/}` 已复制。

---

## 文件结构
- Create: `sim/mujoco_bridge.py` — mujoco 仿真节点(Python rclpy + mujoco)
- Create: `sim/README.md` — 仿真使用说明
- Create: `sim/launch/mujoco_sim.launch.py` — 拉起 mujoco_bridge + inference_node(sim_mode)
- Modify: `src/inference` — 新增 SimMotorBackend(实现 MotorDriver 接口,订阅/发布 topic);或 inference_node 增加 sim_mode 直接读 /joint_states+/imu、写 /joint_targets。**优先最小改动:inference_node 加 sim_mode 分支**,不碰 motors 包。
- Create: `docs/superpowers/specs/2026-06-18-test-report.md` — 测试报告
- (可选后置) `sim/gz_bridge/` — Gazebo Harmonic 集成

---

## Task 1: mujoco bridge(Python rclpy + mujoco)

**Files:** Create `sim/mujoco_bridge.py`

- [ ] **Step 1: 装系统 python mujoco**

Run: `/usr/bin/python3 -m pip install --user mujoco mujoco-python-viewer` (若失败设代理 http://127.0.0.1:7897)
验证: `/usr/bin/python3 -c "import mujoco; print(mujoco.__version__)"`

- [ ] **Step 2: 写 mujoco_bridge.py**

节点 `mujoco_sim_node`:
- 加载 `/home/esi/code/roboparty_deploy/assets/mujoco/scene.xml`(用 from_xml_path)
- `mj_resetDataKeyframe(model, data, 0)` 初始姿态
- 关节名映射:mevius2 [BL,BR,FL,FR]×[collar,hip,knee],用 `mj_name2id` 取 qpos/qvel 地址
- 200Hz 循环(dt=0.005):
  - 读 12 关节 qpos/qvel → 发布 `/joint_states`(sensor_msgs/JointState,name=12关节名)
  - 读 base_link 姿态(quat)+ 角速度 → 发布 `/imu`(sensor_msgs/Imu,frame_id=imu_link)
  - 订阅 `/joint_targets`(sensor_msgs/JointState 或 Float32MultiArray 12维)→ PD:`tau=kp*(target-pos)+kd*(0-vel)` (kp=50,kd=2) → 写 `data.ctrl[actuator_idx]`
  - `mj_step(model, data)`
  - 可选:mujoco_viewer 实时渲染(可按 D 关闭每步渲染)
- 仿真 IMU:从 mujoco body `base_link` 的 `data.qpos`(前7:pos3+quat4)取四元数,`data.qvel`(前6:lin3+ang3)取角速度,转 (w,x,y,z)。重力 gravity_b = R(q)ᵀ·[0,0,-1] 不在 bridge 算(inference_node 的 get_gravity_b_obs 自己算)。

- [ ] **Step 3: 单独启动 bridge 验证**

Run: `/usr/bin/python3 sim/mujoco_bridge.py`
Expected: mujoco viewer 窗口出现,机器人保持站立姿态;`ros2 topic echo /joint_states` 有 12 关节数据;`ros2 topic hz /joint_states` ≈200Hz。

---

## Task 2: inference_node sim_mode 分支

**Files:** Modify `src/inference/src/inference_node.cpp` + `robot_interface.cpp`(最小)

- [ ] **Step 1: 加 sim_mode 参数**

`inference_mevius2.yaml` 加 `sim_mode: true`。inference_node 声明该参数。

- [ ] **Step 2: sim_mode 下状态来源改为 topic 订阅**

sim_mode=true 时:
- 订阅 `/joint_states` → 填 `joint_pos_buffer_`/`joint_vel_buffer_`(按关节名映射到 [BL,BR,FL,FR] 索引)
- 订阅 `/imu` → 填 `quat_buffer_`(转 w-first)/`ang_vel_buffer_`
- 不创建真机 motors/imu,不调 init_motor

sim_mode=false 时:原行为(真机 RobotInterface)。

- [ ] **Step 3: sim_mode 下指令输出改为 topic 发布**

action → target(经 sym/scale/default)→ 发布 `/joint_targets`(Float32MultiArray 12维,顺序 [BL,BR,FL,FR]),不调 motor_mit_cmd。

- [ ] **Step 4: 编译 + 验证**

`colcon build --packages-select roboparty_inference --build-base build_sp_e --install-base install_sp_e --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`

---

## Task 3: mujoco 仿真闭环测试

**Files:** `sim/launch/mujoco_sim.launch.py`

- [ ] **Step 1: launch 拉起 bridge + inference_node(sim_mode)**

- [ ] **Step 2: 启动并观察**

Run: `ros2 launch mujoco_sim.launch.py`
Expected: mujoco 中机器人站立;`ros2 topic pub /cmd_vel` 给前进指令 → 机器人迈步(步态)。

- [ ] **Step 3: 验证站立 + 行走**

- 站立:不给 cmd → 机器人保持站立
- 行走:`ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'` → 观察迈步
- 记录:joint_states 轨迹、imu 姿态、是否跌倒

---

## Task 4: IMU 真机直测(已有脚本,归档结果)

- [ ] **Step 1: 运行 damiao_imu_test.py,记录频率/姿态/单位结果**

---

## Task 5: 测试报告

**Files:** `docs/superpowers/specs/2026-06-18-test-report.md`

- [ ] 记录:SP-A 构建链、SP-B IMU 直测(1000Hz/rad·s/m·s²)、SP-C 电机驱动编译、SP-D1 ONNX 对齐(max-abs 2.29e-5)、SP-D 整链编译+配置加载、SP-E mujoco 站立/行走。
- [ ] 记录已知问题:rtprio 限制、motor_zero_offset 变换待硬件验证、gz 集成后置。

## 验收
- mujoco 中机器人能站立、响应 cmd_vel 行走
- IMU 真机直测通过
- 测试报告文档完成
