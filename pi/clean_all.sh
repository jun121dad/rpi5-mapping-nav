#!/usr/bin/env bash
# 一键清理：停掉所有 ROS 相关进程，并校验干净
# 用法：bash /home/pi/clean_all.sh

echo "[1/4] 先发零速（如果桥接还在跑，让底盘停下）"
source /home/pi/ros_env.sh 2>/dev/null
timeout -k 1 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
sleep 1

echo "[2/4] 杀掉所有相关进程"
for p in calib_yaw map_console nav2_bringup controller_server planner_server bt_navigator \
         behavior_server velocity_smoother collision_monitor waypoint_follower smoother_server \
         route_server docking_server lifecycle_manager_navigation realsense2_camera \
         depthimage_to_laserscan depth_scan_fixed slam_toolbox wheeltec_bridge \
         static_transform_publisher robot_state_publisher rtsp_bridge v4l2_bridge \
         wander.py spin.py explorer detection_bridge semantic_map target_searcher; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 2

echo "[3/4] 停 ros2 daemon"
ros2 daemon stop >/dev/null 2>&1 || true
pkill -9 -f "/opt/ros/jazzy/bin/ros2" 2>/dev/null
sleep 2

echo "[4/4] 校验"
N=$(pgrep -f 'ros2|nav2|slam_toolbox|realsense|wheeltec_bridge|map_console|depth_scan|detection_bridge|semantic_map|target_searcher|explorer|detection_bridge|semantic_map|ros_v4l2_bridge|static_transform|calib_yaw' 2>/dev/null | wc -l | tr -d ' \n')
N=${N:-0}
echo "    ROS 相关进程: $N"
if [ "$N" -ne 0 ]; then
  echo "    仍有残留："
  pgrep -af 'ros2|nav2|slam_toolbox|realsense|wheeltec_bridge|map_console|depth_scan' | head -10
else
  echo "    ✓ 进程干净"
fi
ss -tln 2>/dev/null | grep -q ":8080" && echo "    ✗ 8080 仍被占用" || echo "    ✓ 8080 已释放"
# 检查串口占用（不需要 root：fuser 只读查询，失败时降级为跳过）
if fuser /dev/ttyACM0 >/dev/null 2>&1 || sudo -n fuser /dev/ttyACM0 >/dev/null 2>&1; then
  echo "    ✗ 串口仍被占用"
else
  echo "    ✓ 串口空闲"
fi
awk '{printf "    温度 %.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp
