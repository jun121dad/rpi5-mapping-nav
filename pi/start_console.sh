#!/usr/bin/env bash
# 启动建图控制台（网页 + 进程管理）
# 重复执行安全：若端口被旧的控制台占用，会先清理再启动。
source /home/pi/ros_env.sh

PORT=8080

if ss -tln 2>/dev/null | grep -q ":$PORT "; then
  PID=$(ss -tlnp 2>/dev/null | grep ":$PORT " | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
  if [ -n "$PID" ] && ps -p "$PID" -o args= 2>/dev/null | grep -q map_console; then
    echo "[init] 端口 $PORT 被旧控制台占用 (pid $PID)，先停掉"
    kill -9 "$PID" 2>/dev/null
    sleep 2
  else
    echo "[错误] 端口 $PORT 被其它程序占用: ${PID:-unknown}"
    ss -tlnp 2>/dev/null | grep ":$PORT "
    echo "       请先处理占用，或修改 map_console.py 的 PORT"
    exit 1
  fi
fi

echo "[init] 启动控制台 → http://0.0.0.0:$PORT/"
exec python3 /home/pi/map_console.py
