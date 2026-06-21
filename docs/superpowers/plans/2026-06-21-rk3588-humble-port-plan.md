# RK3588/Humble 仿真适配 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 robodog_jeston 的纯 Python 仿真链在 RK3588 / Ubuntu 22.04 / ROS2 Humble / Python 3.10 上跑通真步态闭环（Phase 1，无真机）。

**Architecture:** 复用已提交的 `sim/mujoco_bridge.py` + `sim/sim_inference_node.py` 两节点 BEST_EFFORT 话题闭环；从 mevius2 原始 TorchScript `policy.pt` 导出真 `policy_mevius2.onnx`；补装 pip/mujoco/onnxruntime（Humble/numpy 已有）；修正脚本里 `/opt/ros/jazzy`、`/home/esi`、`install_sp_d` 残留并清理 Jazzy 构建产物。不改业务逻辑。

**Tech Stack:** ROS2 Humble, Python 3.10, mujoco 3.9.0, onnxruntime 1.21.0, torch+onnx（仅导出用），Fast DDS (rmw_fastrtps_cpp)。

## Global Constraints

- 目标发行版 **ROS2 Humble**（`/opt/ros/humble`，已装），Python 3.10（系统 python == ROS python，`/usr/bin/python3`）
- pip 安装用 `--user`，版本固定：`mujoco==3.9.0`、`onnxruntime==1.21.0`；导出用 `torch`、`onnx`（最新 cp310 aarch64 CPU wheel）
- 代理 `http://127.0.0.1:7897` 用于 apt + pip；sudo 密码 `1234`
- 本轮**仅仿真+离线测试，无真机**；不动子模块内容、不运行 `submodule update`、不动 `robot.yaml`/`can_setup.sh`/udev/RT 内核（Phase 2）
- 业务逻辑零改动：`sim/*.py`、`scripts/*.py` 除注释/路径修正外不重写
- 验收 grep 范围**排除 `docs/`、`.git`、`build*`、`install*`**（历史记录保留）
- 提交信息带 `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`；git 已在仓库本地配置 user.name=Claude Code / user.email=claude@anthropic.com

---

## File Structure

修改/创建的文件：

- **环境（不落盘代码）**：`/etc/apt/apt.conf.d/95proxy`（apt 代理）、`~/.pip` 或 env vars（pip 代理）、`--user` 安装的 mujoco/onnxruntime/torch/onnx
- `tools/export_policy_onnx.py:35-36` — 修正 `DEFAULT_PT`/`DEFAULT_OUT` 默认路径
- `src/inference/models/policy_mevius2.onnx` — **新建**（从 mevius2 `policy.pt` 导出）
- `tools/start_robot.sh:146-147` — `jazzy` → `humble`
- `scripts/run_damiao_imu.sh:5,10,11` — 路径/distro/目录修正
- `scripts/damiao_imu_rviz.launch.py:5-6` — 注释路径修正
- `scripts/damiao_imu_node.py:10` — 注释路径修正
- `sim/launch/mujoco_sim.launch.py:7` — 注释 `jazzy` → `humble`
- `sim/mujoco_bridge.py:7` — 注释 `/home/esi/code/mevius2-master` → `/home/orange5plus/code/mevius2`
- 删除 `build_sp_b/c/d`、`install_sp_b/c/d`
- `sim/sim_inference_node.py` — **仅核对，预期零改动**（obs 复刻已与 mevius2 一致）
- `docs/superpowers/specs/2026-06-21-rk3588-humble-port-test-report.md` — **新建**测试报告

---

## Task 1: 环境引导 — apt 代理 + pip + ROS 依赖确认

**Files:**
- Create: `/etc/apt/apt.conf.d/95proxy`（系统文件，需 sudo）
- Verify: `/opt/ros/humble`, `/usr/bin/python3`

**Interfaces:**
- Produces: 可用的 `pip`、`apt` 代理、`source /opt/ros/humble/setup.bash` 后可 `import rclpy`

- [ ] **Step 1: 配置 apt 代理**

```bash
echo '1234' | sudo -S tee /etc/apt/apt.conf.d/95proxy > /dev/null <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7897";
Acquire::https::Proxy "http://127.0.0.1:7897";
EOF
```

- [ ] **Step 2: 安装 pip**

```bash
echo '1234' | sudo -S apt update
echo '1234' | sudo -S apt install -y python3-pip python3-colcon-common-extensions ros-humble-joy
```

- [ ] **Step 3: 验证 ROS + pip 可用**

Run:
```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy, sensor_msgs, std_msgs, geometry_msgs; print('rclpy OK')"
python3 -m pip --version
```
Expected: `rclpy OK`；pip 版本号输出。

- [ ] **Step 4: 验证 colcon/joy 存在**

Run: `command -v colcon && ros2 pkg list | grep -q joy && echo "colcon+joy OK"`
Expected: `colcon+joy OK`（若上面 apt 装成功则必过）。

- [ ] **Step 5: 提交**（此任务无仓库文件改动，跳过提交；若 Step 1-4 全过即完成）

---

## Task 2: pip 安装运行依赖（mujoco + onnxruntime）

**Files:**
- 无仓库文件；`--user` 安装到 `~/.local`

**Interfaces:**
- Produces: `python3 -c "import mujoco, onnxruntime, numpy"` 全部成功

- [ ] **Step 1: 配置 pip 代理并安装**

```bash
export http_proxy="http://127.0.0.1:7897"
export https_proxy="http://127.0.0.1:7897"
python3 -m pip install --user --upgrade pip
python3 -m pip install --user mujoco==3.9.0 onnxruntime==1.21.0
```

- [ ] **Step 2: 验证 import + 版本**

Run:
```bash
source /opt/ros/humble/setup.bash
python3 -c "import mujoco, onnxruntime as ort, numpy; print('mujoco', mujoco.__version__); print('ort', ort.__version__); print('numpy', numpy.__version__)"
```
Expected:
```
mujoco 3.9.0
ort 1.21.0
numpy 1.21.5
```
（numpy 版本可能更高，以 1.21.5+ 为准。）

- [ ] **Step 3: 提交**（无仓库改动，跳过）

---

## Task 3: 修正 export_policy_onnx.py 默认路径

**Files:**
- Modify: `tools/export_policy_onnx.py:35-36`

**Interfaces:**
- Produces: `DEFAULT_PT` 指向本机 mevius2 `policy.pt`，`DEFAULT_OUT` 指向仓库相对 `policy_mevius2.onnx`，无 `--pt/--out` 时即正确

- [ ] **Step 1: 修正默认路径**

Edit `tools/export_policy_onnx.py`，把：
```python
DEFAULT_PT = "/home/esi/code/mevius2-master/models/policy.pt"
DEFAULT_OUT = "/home/esi/code/roboparty_deploy/src/inference/models/policy_mevius2.onnx"
```
改为：
```python
DEFAULT_PT = "/home/orange5plus/code/mevius2/models/policy.pt"
DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    "src", "inference", "models", "policy_mevius2.onnx")
```

- [ ] **Step 2: 验证语法**

Run: `python3 -c "import ast; ast.parse(open('tools/export_policy_onnx.py').read()); print('syntax OK')"`
Expected: `syntax OK`

- [ ] **Step 3: 提交**

```bash
git add tools/export_policy_onnx.py
git commit -m "fix(sp): export_policy_onnx default paths -> local mevius2 + repo-relative

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Task 4: 导出 policy_mevius2.onnx（含数值对齐）

**Files:**
- Create: `src/inference/models/policy_mevius2.onnx`

**Interfaces:**
- Produces: `src/inference/models/policy_mevius2.onnx`，输入 `(1,34)` 输出 `(1,12)` float32，pt-vs-onnx `np.allclose` 通过

- [ ] **Step 1: 临时装 torch + onnx（仅导出用）**

```bash
export http_proxy="http://127.0.0.1:7897"
export https_proxy="http://127.0.0.1:7897"
python3 -m pip install --user torch onnx
```

- [ ] **Step 2: 确认源 .pt 存在**

Run: `ls -l /home/orange5plus/code/mevius2/models/policy.pt`
Expected: 文件存在，约 214664 字节。

- [ ] **Step 3: 运行导出**

Run:
```bash
python3 tools/export_policy_onnx.py
```
Expected: 脚本依次打印 load（probe 断言输出 `(1,12)`）、export、inspect（in `(1,34)`/out `(1,12)`）、verify（N 样本 max_abs/max_rel，最终 `ALLCLOSE PASS`），退出码 0。

- [ ] **Step 4: 验证产物**

Run:
```bash
ls -l src/inference/models/policy_mevius2.onnx
source /opt/ros/humble/setup.bash
python3 -c "import onnxruntime as ort; s=ort.InferenceSession('src/inference/models/policy_mevius2.onnx', providers=['CPUExecutionProvider']); print('in', s.get_inputs()[0].shape, 'out', s.get_outputs()[0].shape)"
```
Expected: onnx 文件存在（约 200KB）；`in [1, 34] out [1, 12]`。

- [ ] **Step 5: 提交**

```bash
git add src/inference/models/policy_mevius2.onnx
git commit -m "feat(sp): export policy_mevius2.onnx from mevius2 policy.pt (pt-vs-onnx aligned)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Task 5: 修正 scripts/ 与 sim/ 注释/路径残留（jazzy → humble, /home/esi, install_sp_d）

**Files:**
- Modify: `tools/start_robot.sh:146-147`
- Modify: `scripts/run_damiao_imu.sh:5,10,11`
- Modify: `scripts/damiao_imu_rviz.launch.py:5-6`
- Modify: `scripts/damiao_imu_node.py:10`
- Modify: `sim/launch/mujoco_sim.launch.py:7`
- Modify: `sim/mujoco_bridge.py:7`

**Interfaces:**
- Produces: 仓库内（排除 docs/.git/build*/install*）grep `jazzy`/`/home/esi`/`install_sp_d` 全干净

- [ ] **Step 1: start_robot.sh jazzy → humble**

Edit `tools/start_robot.sh:146-147`，把：
```bash
    source /opt/ros/jazzy/setup.bash || {
        print_error "无法source /opt/ros/jazzy/setup.bash，请检查路径是否正确"
```
改为：
```bash
    source /opt/ros/humble/setup.bash || {
        print_error "无法source /opt/ros/humble/setup.bash，请检查路径是否正确"
```

- [ ] **Step 2: run_damiao_imu.sh 路径/distro/目录**

Edit `scripts/run_damiao_imu.sh`，把：
```bash
cd /home/esi/code/roboparty_deploy
```
改为：
```bash
cd "$(dirname "$0")/.."
```
把：
```bash
source /opt/ros/jazzy/setup.bash
source install_sp_d/setup.bash   # provides imu_py
```
改为：
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash   # provides imu_py
```

- [ ] **Step 3: damiao_imu_rviz.launch.py 注释**

Edit `scripts/damiao_imu_rviz.launch.py:5-6`，把：
```python
  source /opt/ros/jazzy/setup.bash
  source install_sp_d/setup.bash   # provides imu_py
```
改为：
```python
  source /opt/ros/humble/setup.bash
  source install/setup.bash   # provides imu_py
```

- [ ] **Step 4: damiao_imu_node.py 注释**

Edit `scripts/damiao_imu_node.py:10`，把：
```python
  source install_sp_d/setup.bash   # provides imu_py
```
改为：
```python
  source install/setup.bash   # provides imu_py
```

- [ ] **Step 5: mujoco_sim.launch.py 注释**

Edit `sim/launch/mujoco_sim.launch.py:7`，把：
```python
#   export PATH=/usr/bin:$PATH; source /opt/ros/jazzy/setup.bash
```
改为：
```python
#   export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.bash
```

- [ ] **Step 6: mujoco_bridge.py 注释**

Edit `sim/mujoco_bridge.py:7`，把：
```python
# Mirrors /home/esi/code/mevius2-master/scripts/mevius2_main.py
```
改为：
```python
# Mirrors /home/orange5plus/code/mevius2/scripts/mevius2_main.py
```

- [ ] **Step 7: grep 验证干净**

Run:
```bash
grep -rn --exclude-dir=.git --exclude-dir='build*' --exclude-dir='install*' --exclude-dir=docs -e 'jazzy' -e '/home/esi' -e 'install_sp_d' . ; echo "exit=$?"
```
Expected: 无输出，`exit=1`（grep 无匹配返回 1）。**注意**：若 docs/ 下历史文档仍有这些字符串属正常（已排除 docs）。

- [ ] **Step 8: 提交**

```bash
git add tools/start_robot.sh scripts/run_damiao_imu.sh scripts/damiao_imu_rviz.launch.py scripts/damiao_imu_node.py sim/launch/mujoco_sim.launch.py sim/mujoco_bridge.py
git commit -m "fix(sp): replace jazzy//home/esi/install_sp_d remnants with humble/repo-relative

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Task 6: 清理 Jazzy/py3.12 构建产物

**Files:**
- Delete: `build_sp_b/`, `build_sp_c/`, `build_sp_d/`, `install_sp_b/`, `install_sp_c/`, `install_sp_d/`

**Interfaces:**
- Produces: 仓库根无 `build_sp_*`/`install_sp_*`

- [ ] **Step 1: 删除六个目录**

Run:
```bash
cd /home/orange5plus/code/robodog_jeston
rm -rf build_sp_b build_sp_c build_sp_d install_sp_b install_sp_c install_sp_d
```

- [ ] **Step 2: 验证不存在**

Run: `ls -d build_sp_* install_sp_* 2>/dev/null; echo "exit=$?"`
Expected: 无输出，`exit=2`（ls 无匹配）。

- [ ] **Step 3: 确认未误删 install/（应本来就不存在）**

Run: `ls -d install 2>/dev/null; echo "install-exists=$?"`
Expected: `install-exists=2`（不存在）。若存在则 STOP 报告——仿真主路径不创建 install/，需人工确认。

- [ ] **Step 4: 提交**（这些目录是否被 git 跟踪需确认）

Run: `git status --short | grep -E 'sp_[bcd]' || echo "not tracked"`
若 git 报告删除（tracked），则：
```bash
git add -A build_sp_b build_sp_c build_sp_d install_sp_b install_sp_c install_sp_d
git commit -m "chore(sp): remove Jazzy/py3.12 build/install residue

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```
若 `not tracked`（被 .gitignore 忽略），则无需提交，此任务完成。

---

## Task 7: 核对 sim_inference_node.py obs 复刻与 mevius2 一致（预期零改动）

**Files:**
- Verify: `sim/sim_inference_node.py` vs `/home/orange5plus/code/mevius2/scripts/mevius2_utils.py`, `parameters.py`

**Interfaces:**
- Produces: 确认 obs_scales / DOF_SYM（输入+输出）/ ACTION_SCALE / DEFAULT_ANGLE / joint order 全一致；不一致则修正并提交

- [ ] **Step 1: 逐项核对**

对照 `sim/sim_inference_node.py` 常量（`sim_inference_node.py:55-67`）与 mevius2：

| 项 | sim_inference_node.py | mevius2 来源 | 一致? |
|---|---|---|---|
| obs 维度 34 | `build_obs` 拼接 3+3+3+12+12+1=34 | `mevius2_utils.py:95-102` | 核对 |
| SCALE_ANG_VEL | 0.25 | `mevius2_utils.py:14` ang_vel=0.25 | 核对 |
| SCALE_LIN_VEL | 2.0 | `mevius2_utils.py:13` lin_vel=2.0 | 核对 |
| SCALE_DOF_POS | 1.0 | `mevius2_utils.py:15` dof_pos=1.0 | 核对 |
| SCALE_DOF_VEL | 0.05 | `mevius2_utils.py:16` dof_vel=0.05 | 核对 |
| DOF_SYM | `[1,1,1,-1,1,1,1,1,1,-1,1,1]` | `mevius2_utils.py:68-71` | 核对 |
| DOF_SYM 作用于输入 | `(dof_pos-default)*SCALE_DOF_POS*DOF_SYM` (`:168`) + `dof_vel*...*DOF_SYM` (`:169`) | `mevius2_utils.py:73-74` | 核对 |
| DOF_SYM 作用于输出 | `action * DOF_SYM * ACTION_SCALE` (`:185`) | `mevius2_utils.py:112-116` | 核对 |
| ACTION_SCALE | 0.2 | `parameters.py:81` | 核对 |
| DEFAULT_ANGLE | `[0,0.7,-1.2]*4` | `parameters.py:60-65` | 核对 |
| joint order | `[BL,BR,FL,FR]×(collar,hip,knee)` (`:46-51`) | `parameters.py:46-51` | 核对 |
| CLIP_OBS / CLIP_ACTION | 100 / 100 | `mevius2_utils.py:19-20` | 核对 |

**预期**：上轮代码审阅已确认全部一致，本步应零改动。

- [ ] **Step 2: 若全部一致——记录确认，无提交**

Run: `git status --short sim/sim_inference_node.py`
Expected: 无输出（未改动）。本任务完成。

- [ ] **Step 3: 若发现不一致——修正**

若 Step 1 发现某项不一致，Edit `sim/sim_inference_node.py` 改为与 mevius2 一致的值，然后：
```bash
git add sim/sim_inference_node.py
git commit -m "fix(sp): align sim_inference obs math with mevius2 (SCALE/SYM/DEFAULT)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Task 8: 仿真烟雾测试 — mujoco bridge 无头运行

**Files:**
- Verify: `sim/mujoco_bridge.py` 运行行为

**Interfaces:**
- Consumes: Task 2 (mujoco import), Task 5 (launch 注释修正)
- Produces: `/mujoco_sim_node` 上线，`/joint_states` ~200Hz 12 关节，`/imu` ~200Hz

- [ ] **Step 1: 启动 bridge（无头，后台）**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
python3 sim/mujoco_bridge.py --no-viewer > /tmp/bridge.log 2>&1 &
echo $! > /tmp/bridge.pid
sleep 4
```

- [ ] **Step 2: 验证节点上线 + joint_states**

Run:
```bash
ros2 node list | grep mujoco_sim_node
ros2 topic hz /joint_states --window 50 &
sleep 6; kill %1 2>/dev/null
```
Expected: `mujoco_sim_node` 出现；`/joint_states` ~200Hz。

- [ ] **Step 3: 验证关节名与数量**

Run: `timeout 3 ros2 topic echo /joint_states --once`
Expected: `name` 数组 12 项，为 `[BL,BR,FL,FR]×[collar,hip,knee]_joint`，`position` 接近 STANDBY。

- [ ] **Step 4: 验证 /imu**

Run: `ros2 topic hz /imu --window 50 & sleep 5; kill %1 2>/dev/null`
Expected: `/imu` ~200Hz。

- [ ] **Step 5: 停止 bridge**

Run: `kill $(cat /tmp/bridge.pid) 2>/dev/null; sleep 1`

- [ ] **Step 6: 提交**（无仓库改动，跳过；记录到测试报告 Task 11）

---

## Task 9: 仿真测试 — sim_inference_node + 接线

**Files:**
- Verify: `sim/sim_inference_node.py` 运行行为

**Interfaces:**
- Consumes: Task 4 (onnx), Task 8 (bridge)
- Produces: `/sim_inference_node` 上线，`/joint_targets` ~50Hz 12 维，三话题各 1 pub+1 sub

- [ ] **Step 1: 启动 bridge（后台）**

```bash
cd /home/orange5plus/code/robodog_jeston
source /opt/ros/humble/setup.bash
python3 sim/mujoco_bridge.py --no-viewer > /tmp/bridge.log 2>&1 &
echo $! > /tmp/bridge.pid
sleep 3
```

- [ ] **Step 2: 启动 inference 节点（后台）**

```bash
python3 sim/sim_inference_node.py > /tmp/inference.log 2>&1 &
echo $! > /tmp/inference.pid
sleep 4
grep -E 'loaded ONNX|sim_inference_node @' /tmp/inference.log
```
Expected: 日志含 `loaded ONNX ... in=[1, 34] out=[1, 12]` 与 `sim_inference_node @ 50Hz`。

- [ ] **Step 3: 验证 /joint_targets**

Run: `ros2 topic hz /joint_targets --window 50 & sleep 5; kill %1 2>/dev/null`
Expected: `/joint_targets` ~50Hz。

Run: `timeout 3 ros2 topic echo /joint_targets --once`
Expected: `data` 数组 12 项 float32。

- [ ] **Step 4: 验证接线（1 pub + 1 sub）**

Run:
```bash
for t in /joint_states /imu /joint_targets; do
  echo "== $t =="
  ros2 topic info -v $t | grep -A2 -E 'Publishers|Subscribers'
done
```
Expected: `/joint_states` 1 pub (mujoco_sim_node) + 1 sub (sim_inference_node)；`/imu` 1 pub + 1 sub；`/joint_targets` 1 pub (sim_inference) + 1 sub (mujoco_sim_node)；QoS 均 BEST_EFFORT/VOLATILE。

- [ ] **Step 5: 真步态闭环观察**

让两节点持续运行 8 秒，观察 inference 日志的 `tgt[0:3]` 与 bridge 是否保持站立：
```bash
sleep 8
grep 'tgt\[0:3\]' /tmp/inference.log | tail -3
ros2 topic echo /mujoco/base_pose --once 2>/dev/null | grep -E 'x:|y:|z:' | head -3
```
Expected: `tgt` 非全零（真模型产生动作）；base_pose 的 z 不显著塌陷（机器人保持站立/轻微步态）。若 base z 塌陷或 tgt 全零 → 回 Task 7 复查 obs 复刻。

- [ ] **Step 6: 停止两节点**

Run:
```bash
kill $(cat /tmp/inference.pid) $(cat /tmp/bridge.pid) 2>/dev/null
sleep 1
```

- [ ] **Step 7: 提交**（无仓库改动，跳过；记录到测试报告 Task 11）

---

## Task 10: 路径/清理最终验收 grep

**Files:**
- Verify: 全仓库

**Interfaces:**
- Consumes: Task 5, Task 6

- [ ] **Step 1: 路径残留 grep（排除 docs/.git/build*/install*）**

Run:
```bash
grep -rn --exclude-dir=.git --exclude-dir='build*' --exclude-dir='install*' --exclude-dir=docs -e 'jazzy' -e '/home/esi' -e 'install_sp_d' . ; echo "exit=$?"
```
Expected: 无输出，`exit=1`。

- [ ] **Step 2: 构建产物目录不存在**

Run: `ls -d build_sp_* install_sp_* 2>/dev/null; echo "exit=$?"`
Expected: 无输出，`exit=2`。

- [ ] **Step 3: import 冒烟**

Run:
```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy, sensor_msgs, std_msgs, geometry_msgs, mujoco, onnxruntime as ort, numpy; print('all import OK', mujoco.__version__, ort.__version__)"
```
Expected: `all import OK 3.9.0 1.21.0`

- [ ] **Step 4: 提交**（无仓库改动，跳过）

---

## Task 11: 写测试报告文档并提交

**Files:**
- Create: `docs/superpowers/specs/2026-06-21-rk3588-humble-port-test-report.md`

**Interfaces:**
- Consumes: Task 1–10 全部结果

- [ ] **Step 1: 写测试报告**

创建 `docs/superpowers/specs/2026-06-21-rk3588-humble-port-test-report.md`，内容包含：标题、日期、Phase 1 验收标准 AC-1..AC-9 逐条、每条对应 Task 步骤 + 实际命令输出摘录（从 `/tmp/*.log` 与上面各 Step 的输出粘贴）、T1..T12 测试表结果、结论（通过/部分通过 + 原因）、遗留项（Phase 2）。每条 AC 标记 ✅/❌。

- [ ] **Step 2: 提交**

```bash
git add docs/superpowers/specs/2026-06-21-rk3588-humble-port-test-report.md
git commit -m "docs(sp): RK3588/Humble sim port Phase 1 test report

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Self-Review

**1. Spec coverage**（对照 requirements FR / AC）：
- FR-1 环境 → Task 1, 2 ✅
- FR-2 ONNX 导出 → Task 3, 4 ✅
- FR-3 路径修正 → Task 5 ✅
- FR-4 清理 → Task 6 ✅
- FR-5 obs 复刻核对 → Task 7 ✅
- FR-6 仿真测试 → Task 8, 9 ✅
- FR-7 文档 → Task 11 ✅
- AC-1..AC-9 → Task 10（验收 grep/import）+ Task 8/9（仿真）+ Task 11（报告）全覆盖 ✅
- Phase 2 → 明确不在本计划，spec 第 7 节已记录 ✅

**2. Placeholder scan**：无 TBD/TODO；每步含具体命令或代码。Task 11 报告内容为"粘贴实际输出"——这是运行期产物，非占位符。✅

**3. Type consistency**：常量名跨任务一致（`DEFAULT_ANGLE`、`DOF_SYM`、`ACTION_SCALE`、`SCALE_*`）；文件路径一致。✅

无修复项。
