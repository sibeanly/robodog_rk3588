#!/bin/bash
# SPDX-License-Identifier: GPL-3.0
#
# run_sim_viewer.sh - 带 mujoco 可视化的独立启动脚本（手动验证用）
#
# 用法：
#   ./sim/run_sim_viewer.sh          # 启动（mujoco viewer 窗口 + inference）
#   ./sim/run_sim_viewer.sh stop     # 停止
#
# 与 run_sim.sh 的区别：
#   - bridge 前台运行，viewer 窗口关闭 = 仿真停止（直观）
#   - 启动前自动杀掉 damiao_imu_node 等幽灵 /imu publisher（否则会污染 obs 导致翻倒）
#   - inference 日志实时打印前 10 帧 + 每秒一行，便于观察
#
# 让机器人走动（另开一个终端）：
#   ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
#   ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'   # 停止
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# 必须 source ROS2 Humble
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  source /opt/ros/humble/setup.bash || {
    echo "ERROR: 无法 source /opt/ros/humble/setup.bash" >&2
    exit 1
  }
fi

INFER_PIDFILE=/tmp/sim_inference.pid
INFER_LOG=/tmp/sim_inference.log
BRIDGE_LOG=/tmp/sim_bridge.log

# --- stop 子命令 ---
if [ "${1:-}" = "stop" ]; then
  echo "停止仿真..."
  for f in "$INFER_PIDFILE"; do
    if [ -f "$f" ]; then
      pid=$(cat "$f" 2>/dev/null || true)
      if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
        echo "  已停止 inference PID $pid"
      fi
      rm -f "$f"
    fi
  done
  pkill -9 -f mujoco_bridge 2>/dev/null || true
  exit 0
fi

# --- 检查 DISPLAY ---
if [ -z "${DISPLAY:-}" ]; then
  echo "ERROR: DISPLAY 未设置，无法启动 viewer。请在桌面环境运行，或设置 DISPLAY。" >&2
  exit 1
fi
echo "DISPLAY=$DISPLAY"

# --- 关键：杀掉幽灵 /imu publisher（damiao_imu_node 等），否则会污染 obs 导致翻倒 ---
echo "[预检] 检查并清理 /imu 的幽灵 publisher..."
pkill -9 -f damiao_imu_node 2>/dev/null && echo "  已杀掉 damiao_imu_node" || echo "  无 damiao_imu_node（好）"
pkill -9 -f "static_transform_publisher.*imu_link" 2>/dev/null || true

# --- 清理残留的旧 sim 进程 ---
pkill -9 -f mujoco_bridge 2>/dev/null || true
if [ -f "$INFER_PIDFILE" ]; then
  oldpid=$(cat "$INFER_PIDFILE" 2>/dev/null || true)
  [ -n "${oldpid:-}" ] && kill -9 "$oldpid" 2>/dev/null || true
  rm -f "$INFER_PIDFILE"
fi

# --- 启动 inference（后台） ---
echo "[1/2] 启动 sim_inference_node（后台）..."
rm -f "$INFER_LOG"
python3 sim/sim_inference_node.py > "$INFER_LOG" 2>&1 &
echo $! > "$INFER_PIDFILE"
echo "  PID $(cat "$INFER_PIDFILE")  日志 $INFER_LOG"

# --- 启动 bridge（前台，带 viewer） ---
echo "[2/2] 启动 mujoco bridge（前台 + viewer）..."
echo "  mujoco viewer 窗口即将弹出。关闭窗口或 Ctrl+C 即可停止仿真。"
echo ""
echo "让机器人走动（另开终端）："
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'"
echo ""
echo "inference 日志（前 10 帧 + 每秒）：tail -f $INFER_LOG"
echo "================================================"
rm -f "$BRIDGE_LOG"
# bridge 前台运行；te tee 同时写日志。viewer 关闭后 bridge 退出，脚本结束。
python3 sim/mujoco_bridge.py 2>&1 | tee "$BRIDGE_LOG" &
BRIDGE_PID=$!

# bridge 退出时清理 inference
cleanup() {
  echo ""
  echo "[清理] bridge 已退出，停止 inference..."
  kill -9 "$BRIDGE_PID" 2>/dev/null || true
  if [ -f "$INFER_PIDFILE" ]; then
    kill -9 "$(cat "$INFER_PIDFILE")" 2>/dev/null || true
    rm -f "$INFER_PIDFILE"
  fi
  echo "已停止。bridge 日志: $BRIDGE_LOG  inference 日志: $INFER_LOG"
}
trap cleanup EXIT

# 等待 bridge 进程结束（viewer 关闭或崩溃）
wait "$BRIDGE_PID" 2>/dev/null || true
