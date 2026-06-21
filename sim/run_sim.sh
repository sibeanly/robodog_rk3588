#!/bin/bash
# SPDX-License-Identifier: GPL-3.0
#
# run_sim.sh - 一键启动 mevius2 mujoco 仿真（bridge + inference）
#
# 用法：
#   ./sim/run_sim.sh              # 无头模式（默认，RK3588 推荐）
#   ./sim/run_sim.sh --viewer     # 带 mujoco viewer（需 DISPLAY + EGL）
#   ./sim/run_sim.sh stop         # 停止仿真
#
# 启动后两个节点在后台运行，日志写到 /tmp/sim_bridge.log /tmp/sim_inference.log
# 让机器人走动：ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
# 停止：Ctrl+C 或 ./sim/run_sim.sh stop
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# 必须先 source ROS2 Humble
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
  source /opt/ros/humble/setup.bash || {
    echo "ERROR: 无法 source /opt/ros/humble/setup.bash" >&2
    exit 1
  }
fi

BRIDGE_PIDFILE=/tmp/sim_bridge.pid
INFER_PIDFILE=/tmp/sim_inference.pid
BRIDGE_LOG=/tmp/sim_bridge.log
INFER_LOG=/tmp/sim_inference.log

# --- stop 子命令 ---
if [ "${1:-}" = "stop" ]; then
  echo "停止仿真..."
  for f in "$INFER_PIDFILE" "$BRIDGE_PIDFILE"; do
    if [ -f "$f" ]; then
      pid=$(cat "$f" 2>/dev/null || true)
      if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
        echo "  已停止 PID $pid ($(basename "$f" .pid))"
      fi
      rm -f "$f"
    fi
  done
  exit 0
fi

# --- 解析 --viewer ---
VIEWER_ARG="--no-viewer"
if [ "${1:-}" = "--viewer" ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "WARN: --viewer 指定但 DISPLAY 未设置，回退无头模式" >&2
  else
    VIEWER_ARG=""
  fi
fi

# --- 清理可能残留的旧进程 ---
for f in "$BRIDGE_PIDFILE" "$INFER_PIDFILE"; do
  if [ -f "$f" ]; then
    oldpid=$(cat "$f" 2>/dev/null || true)
    if [ -n "${oldpid:-}" ] && kill -0 "$oldpid" 2>/dev/null; then
      kill "$oldpid" 2>/dev/null || true
    fi
    rm -f "$f"
  fi
done

# --- 启动 bridge（物理仿真）---
echo "[1/2] 启动 mujoco bridge..."
python3 sim/mujoco_bridge.py $VIEWER_ARG > "$BRIDGE_LOG" 2>&1 &
echo $! > "$BRIDGE_PIDFILE"
echo "  PID $(cat "$BRIDGE_PIDFILE")  日志 $BRIDGE_LOG"

# 等 bridge 上线（最多 8 秒）
for i in $(seq 1 16); do
  if grep -q "mujoco bridge @" "$BRIDGE_LOG" 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if ! grep -q "mujoco bridge @" "$BRIDGE_LOG" 2>/dev/null; then
  echo "ERROR: bridge 未在 8 秒内上线，日志：" >&2
  tail -20 "$BRIDGE_LOG" >&2
  exit 1
fi
echo "  bridge 已上线: $(grep 'mujoco bridge @' "$BRIDGE_LOG" | tail -1)"

# --- 启动 inference（策略推理）---
echo "[2/2] 启动 sim_inference_node..."
python3 sim/sim_inference_node.py > "$INFER_LOG" 2>&1 &
echo $! > "$INFER_PIDFILE"
echo "  PID $(cat "$INFER_PIDFILE")  日志 $INFER_LOG"

# 等 inference 加载 ONNX（最多 5 秒）
for i in $(seq 1 10); do
  if grep -q "loaded ONNX" "$INFER_LOG" 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if ! grep -q "loaded ONNX" "$INFER_LOG" 2>/dev/null; then
  echo "ERROR: inference 未在 5 秒内加载 ONNX，日志：" >&2
  tail -20 "$INFER_LOG" >&2
  exit 1
fi
echo "  inference 已上线: $(grep 'loaded ONNX' "$INFER_LOG" | tail -1)"

echo ""
echo "============================================"
echo "  仿真已启动（bridge + inference 后台运行）"
echo "============================================"
echo ""
echo "查看话题："
echo "  ros2 topic list"
echo "  ros2 topic echo /joint_states --qos-reliability best_effort --once"
echo ""
echo "让机器人走动："
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'   # 停止"
echo ""
echo "停止仿真："
echo "  ./sim/run_sim.sh stop   或   kill \$(cat $INFER_PIDFILE) \$(cat $BRIDGE_PIDFILE)"
echo ""
echo "实时日志：tail -f $BRIDGE_LOG  /  $INFER_LOG"
echo ""
echo "按 Ctrl+C 不会停止后台进程；如需停止请用上面的 stop 命令。"
