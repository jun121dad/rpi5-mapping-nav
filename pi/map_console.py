#!/usr/bin/env python3
"""建图控制台：网页按钮控制起停/存图/遥控，自动探索可被手动接管。
   访问 http://<pi>:8080/   命令行兜底见 /home/pi/mapctl
"""
import json
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4')

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Float32, String
import tf2_ros

PORT = 8080
MAPS = '/home/pi/maps'
LOGDIR = '/home/pi/mapping_logs/console'
MAXV, MAXW = 0.20, 0.80
DETECT_ENABLED = True      # 三合一：检测 + 语义地图随栈一起起
NAV2_CMD = ['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=false',
            'params_file:=/home/pi/config/nav2_d455.yaml',
            'bond_timeout:=30.0',      # 心跳超时 4s→30s，避免 CPU 抖动直接判死
            'autostart:=True']
VIEW_M = 8.0            # 视野边长（米），车固定在中心
AUTOSAVE_SEC = 60.0
DEADMAN = 0.5

L = threading.Lock()
S = {
    'state': 'IDLE',        # IDLE / STARTING / MAPPING / ERROR
    'mode': 'manual',       # auto / manual（默认手动，按了“自动探索”才动）
    'err': '',
    'manual_v': 0.0, 'manual_w': 0.0, 'manual_ts': 0.0,
    'cmd_v': 0.0, 'cmd_w': 0.0,
    'map': None,            # (w,h,res,ox,oy,bytes)
    'battery': None, 'odom': (0.0, 0.0, 0.0), 'dist': 0.0,
    'fps': {'scan': 0.0, 'odom': 0.0, 'map': 0.0},
    'last_save': '', 'png': None, 'png_ts': 0.0,
    'traj': [],
    'nav_active': False, 'goal_xy': None, 'sem': [], 'det': {}, 'search': {}, 'explore': {}, 'view_m': 8.0,    # Nav2 是否在跑（跑起来后控制台不再写 /cmd_vel）
}


def clamp(v, lim):
    return max(-lim, min(lim, v))


def png_bytes(w, h, rgb):
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(tag, data):
        return (struct.pack('>I', len(data)) + tag + data +
                struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))

    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) +
            chunk(b'IDAT', zlib.compress(bytes(raw), 6)) + chunk(b'IEND', b''))


# ---------------------------------------------------------------- 进程管理
class Supervisor:
    def __init__(self):
        self.groups = []
        self.logdir = os.path.join(LOGDIR, time.strftime('%m%d_%H%M%S'))
        os.makedirs(self.logdir, exist_ok=True)

    def spawn(self, name, argv, env=None, cpu=None):
        if cpu:
            argv = ['taskset', '-c', cpu] + list(argv)
        f = open(os.path.join(self.logdir, name + '.log'), 'wb')
        p = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True, env=env)
        self.groups.append((name, os.getpgid(p.pid), p))
        return p

    STACK_PATTERNS = ('realsense2_camera', 'depthimage_to_laserscan', 'slam_toolbox',
                      'wheeltec_bridge', 'static_transform_publisher', 'nav2_bringup',
                      'controller_server', 'planner_server', 'bt_navigator',
                      'behavior_server', 'velocity_smoother', 'collision_monitor',
                      'waypoint_follower', 'smoother_server', 'route_server',
                      'docking_server', 'lifecycle_manager_navigation',
                      'detection_bridge', 'semantic_map', 'ros_v4l2_bridge', 'target_searcher')

    def cleanup_stack(self):
        """起栈前清掉所有残留，避免出现两套 slam/桥接同时发 TF"""
        for pat in self.STACK_PATTERNS:
            subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)
        time.sleep(2)

    def start(self):
        self.cleanup_stack()
        self.spawn('camera', [
            'ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'depth_module.depth_profile:=640x480x30',
            'depth_module.emitter_enabled:=1',
            'depth_module.visual_preset:=3',
            'depth_module.enable_auto_exposure:=true',
            'enable_color:=false',      # 检测已移除，彩色流无用（省 USB/CPU/电）
            'enable_depth:=true',
            'align_depth.enable:=false'], cpu='0,1')
        time.sleep(9)
        # 自研深度→激光：正确补偿相机俯仰，只保留离地 0.1-0.8 m 的点
        self.spawn('scan', ['python3', '/home/pi/depth_scan_fixed.py'], cpu='3')
        if os.path.exists('/dev/ttyACM0'):
            self.spawn('bridge', [
                'ros2', 'run', 'wheeltec_bridge', 'bridge', '--ros-args',
                '-p', 'port:=/dev/ttyACM0', '-p', 'publish_odom:=true'], cpu='2')
        else:
            self.spawn('odom_fake', [
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0', '--y', '0', '--z', '0',
                '--frame-id', 'odom', '--child-frame-id', 'base_link'])
        self.spawn('cam_tf', [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0.08', '--y', '0', '--z', '0.49',
            '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])
        # Nav2 部分节点默认用 base_footprint，缺了会让 collision_monitor 吞掉所有指令
        self.spawn('base_footprint', [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', '0', '--y', '0', '--z', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'])
        time.sleep(4)
        self.spawn('slam', [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            'use_sim_time:=false', 'slam_params_file:=/home/pi/config/slam_d455.yaml'], cpu='2')
        # 前沿探索节点（只做建图+导航，检测链已移除）
        self.spawn('explorer', ['python3', '/home/pi/explorer.py'], cpu='2')
        time.sleep(1)
        self.spawn('nav2', NAV2_CMD, cpu='0,1')

    def alive(self):
        return [n for n, _, p in self.groups if p.poll() is None]

    def stop(self):
        for _, pgid, _ in self.groups:
            try:
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(3)
        for _, pgid, _ in self.groups:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
        self.groups = []

    NAV_PATTERNS = ('nav2_bringup', 'controller_server', 'planner_server', 'bt_navigator',
                    'behavior_server', 'velocity_smoother', 'collision_monitor',
                    'waypoint_follower', 'smoother_server', 'route_server',
                    'docking_server', 'lifecycle_manager_navigation')

    def start_nav(self):
        self.stop_nav()                      # 先确保没有残留，避免两个 Nav2 打架
        time.sleep(1)
        self.spawn('nav2', NAV2_CMD, cpu='0,1')
        return True

    def stop_nav(self):
        """停掉 Nav2 全部节点（含脱离进程组的子进程），保留建图"""
        hit = False
        for n, pgid, _ in self.groups:
            if n == 'nav2':
                hit = True
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except Exception:
                    pass
        self.groups = [(n, g, p) for (n, g, p) in self.groups if n != 'nav2']
        time.sleep(1.0)
        for pat in self.NAV_PATTERNS:
            subprocess.run(['pkill', '-9', '-f', pat], capture_output=True)
        time.sleep(0.5)
        return hit


# ---------------------------------------------------------------- ROS 节点
class ConsoleNode(Node):
    def __init__(self):
        super().__init__('map_console')
        self.map = None
        self.scan = None
        self.n = {'scan': 0, 'odom': 0, 'map': 0}
        self.t = {'scan': 0.0, 'odom': 0.0, 'map': 0.0}
        self.rate = {'scan': 0, 'odom': 0, 'map': 0}
        self._last_rate = time.time()

        q_vol = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
        q_lat = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q_vol)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q_lat)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float32, '/wheeltec/voltage', self.on_volt, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_echo, 10)
        self.create_subscription(String, '/semantic/status', self.on_sem, 10)
        self.create_subscription(String, '/detections/status', self.on_det, 10)
        self.create_subscription(String, '/explore/status', self.on_explore, 10)
        self.pub_sem_cmd = self.create_publisher(String, '/semantic/command', 10)
        self.pub_explore = self.create_publisher(Bool, '/explore/enable', 10)
        self.pub_duration = self.create_publisher(Float32, '/explore/duration', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_future = None
        self._nav_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.wander_state = 'FWD'
        self.turn_dir = 1.0
        self.create_timer(0.1, self.control_tick)
        self.create_timer(1.0, self.slow_tick)

    # --- 导航目标
    def send_nav_goal(self, gx, gy, yaw=None):
        now_t = time.time()
        if now_t - getattr(self, '_last_goal_ts', 0.0) < 0.8:
            return 'TOO_FAST'          # 去抖：短时间内重复点击忽略
        self._last_goal_ts = now_t
        if yaw is None:
            with L:
                rx, ry, ryaw = S['odom']
            yaw = math.atan2(gy - ry, gx - rx) if (abs(gx - rx) + abs(gy - ry)) > 0.05 else ryaw
        # 不用 server_is_ready() 预判：多线程执行器下它会误报 False。
        # 直接发送，拒绝由 _on_goal_response 回调处理。
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        with L:
            S['goal_xy'] = (gx, gy)
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)
        self._nav_future = fut
        return 'ok'

    def _on_goal_response(self, fut):
        try:
            self._nav_handle = fut.result()
            ok = bool(self._nav_handle and self._nav_handle.accepted)
            self.get_logger().info('导航目标%s' % ('已接受' if ok else '被拒绝'))
        except Exception as exc:
            self.get_logger().warn('目标响应异常: %s' % exc)

    def cancel_nav(self):
        if self._nav_handle is None:
            return 'no goal'
        try:
            self._nav_handle.cancel_goal_async()
            with L:
                S['goal_xy'] = None
            self._nav_handle = None
            return 'cancelling'
        except Exception as exc:
            return 'cancel failed: %s' % exc

    def send_nav_goal_px(self, px, py, w=None, h=None):
        with L:
            rx, ry, ryaw = S['odom']
            view = float(S.get('view_m', VIEW_M))
        m = self.map
        res = m.info.resolution if m is not None else 0.05
        # 与 render() 完全相同的取整规则，保证点击位置与画面像素严格对应
        hw = max(6, int(round(view / 2.0 / res)))
        cover = hw * 2 * res              # 画面实际覆盖的米数
        W = float(w) if (w and w > 0) else 640.0
        H = float(h) if (h and h > 0) else 640.0
        gx = rx + (px / W - 0.5) * cover
        gy = ry - (py / H - 0.5) * cover
        # 目标点必须落在已知空闲区域，否则规划必然失败（会触发 Spin 恢复，看着像原地转圈）
        if m is not None:
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            ci = int((gx - ox) / res)
            cj = int((gy - oy) / res)
            if 0 <= ci < m.info.width and 0 <= cj < m.info.height:
                g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
                c = int(g[cj, ci])
                if c < 0:
                    return 'UNKNOWN_AREA'
                if c >= 50:
                    return 'BLOCKED_AREA'
            else:
                return 'OUT_OF_MAP'
        return self.send_nav_goal(gx, gy)

    def on_explore(self, m):
        try:
            with L:
                S['explore'] = json.loads(m.data)
        except Exception:
            pass
    def on_search(self, m):
        try:
            with L:
                S['search'] = json.loads(m.data)
        except Exception:
            pass

    def on_det(self, m):
        try:
            d = json.loads(m.data)
            with L:
                S['det'] = d
        except Exception:
            pass

    def on_sem(self, m):
        try:
            d = json.loads(m.data)
            with L:
                S['sem'] = d.get('objects', [])
        except Exception:
            pass

    # --- 回调
    def on_cmd_echo(self, m):
        """显示用：把 /cmd_vel 上的实际值（可能来自 Nav2）同步到状态栏"""
        with L:
            S['cmd_v'] = float(m.linear.x)
            S['cmd_w'] = float(m.angular.z)

    def on_map(self, m):
        self.map = m
        self.n['map'] += 1
        self.t['map'] = time.time()

    def on_scan(self, m):
        self.scan = m
        self.n['scan'] += 1
        self.t['scan'] = time.time()

    def on_odom(self, m):
        self.n['odom'] += 1
        self.t['odom'] = time.time()

    def on_volt(self, m):
        with L:
            S['battery'] = float(m.data)

    # --- 位姿
    def pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    # --- 10 Hz 控制
    def control_tick(self):
        now = time.time()
        with L:
            mode = S['mode']
            state = S['state']
            nav = S['nav_active']
            mv, mw, mts = S['manual_v'], S['manual_w'], S['manual_ts']
            manual_fresh = (now - mts) < DEADMAN and (abs(mv) > 1e-3 or abs(mw) > 1e-3)

        # Nav2 接管时：没手动输入就不写；一旦有手动输入，停掉 Nav2 交还控制权
        if nav:
            if manual_fresh:
                # 立刻交还控制权，停 Nav2 放后台做（避免阻塞 2.5 秒导致按键失灵）
                with L:
                    S['nav_active'] = False
                threading.Thread(target=SUP.stop_nav, daemon=True).start()
            else:
                return

        # 手动永远优先，且不依赖建图状态（IDLE 也能开车）
        v = w = 0.0
        if manual_fresh:
            v, w = mv, mw
            with L:
                S['mode'] = 'manual'
        elif mode == 'auto' and state == 'MAPPING':
            v, w = self.auto_step()

        with L:
            S['ticks'] = S.get('ticks', 0) + 1
        v, w = clamp(v, MAXV), clamp(w, MAXW)
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)
        with L:
            S['cmd_v'], S['cmd_w'] = v, w

    # --- 自动探索（保守）
    def auto_step(self):
        sc = self.scan
        if sc is None or time.time() - self.t['scan'] > 1.0:
            return 0.0, 0.0

        def s_min(lo, hi):
            best, n = 99.0, 0
            for i, r in enumerate(sc.ranges):
                a = sc.angle_min + i * sc.angle_increment
                if lo <= a <= hi and math.isfinite(r) and r > 0.05:
                    best = min(best, r)
                    n += 1
            return best if n else 99.0

        front = s_min(-0.35, 0.35)
        if self.wander_state == 'FWD':
            if front < 0.55:
                left, right = s_min(-1.3, -0.4), s_min(0.4, 1.3)
                self.turn_dir = 1.0 if left > right else -1.0
                self.wander_state = 'TURN'
                return 0.0, 0.0
            return 0.12, 0.0
        else:
            if front > 0.95:
                self.wander_state = 'FWD'
                return 0.12, 0.0
            return 0.0, 0.35 * self.turn_dir

    # --- 1 Hz 慢循环
    def slow_tick(self):
        now = time.time()
        dt = now - self._last_rate
        if dt > 0.5:
            with L:
                for k in self.n:
                    rate = (self.n[k] - self.rate[k]) / dt
                    S['fps'][k] = round(rate, 1)
                    self.rate[k] = self.n[k]
            self._last_rate = now

        p = self.pose()
        if p:
            with L:
                S['odom'] = p
                tr = S['traj']
                if not tr or math.hypot(p[0] - tr[-1][0], p[1] - tr[-1][1]) > 0.05:
                    tr.append((p[0], p[1]))
                    if len(tr) > 3000:
                        del tr[:500]
                S['dist'] = sum(math.hypot(tr[i + 1][0] - tr[i][0], tr[i + 1][1] - tr[i][1])
                                for i in range(len(tr) - 1)) if len(tr) > 1 else 0.0

        m = self.map
        if m is not None:
            with L:
                S['map'] = (m.info.width, m.info.height, m.info.resolution,
                            m.info.origin.position.x, m.info.origin.position.y,
                            bytes(np.array(m.data, dtype=np.int16).tobytes()), m)

    # --- 渲染
    def render(self):
        m = self.map
        if m is None:
            return None
        with L:
            traj = list(S['traj'])
            odom = S['odom']
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        g = np.array(m.data, dtype=np.int16).reshape(h, w)

        with L:
            view_m = float(S.get('view_m', VIEW_M))
        hw = max(6, int(round(view_m / 2.0 / res)))
        # 缩放：保持输出约 640px，每像素对应的米数随视野变化
        sc = max(1, int(round(640.0 / (hw * 2.0))))
        k = sc / 4.0      # 相对基准(4)的缩放系数，用于标记尺寸
        W = H = hw * 2 * sc
        img = np.full((H, W, 3), 100, dtype=np.uint8)

        # 车始终在画布中心
        if odom:
            rx, ry, ryaw = odom
        else:
            rx, ry, ryaw = ox + w * res / 2.0, oy + h * res / 2.0, 0.0

        cx = int(round((rx - ox) / res))
        cy = int(round((ry - oy) / res))
        x0, y0 = cx - hw, cy - hw
        sx0, sy0 = max(0, x0), max(0, y0)
        sx1, sy1 = min(w, x0 + 2 * hw), min(h, y0 + 2 * hw)
        if sx1 > sx0 and sy1 > sy0:
            sub = g[sy0:sy1, sx0:sx1]
            cell = np.full(sub.shape + (3,), 100, dtype=np.uint8)
            cell[(sub >= 0) & (sub < 50)] = 245
            cell[sub >= 50] = 15
            cell = np.repeat(np.repeat(cell, sc, axis=0), sc, axis=1)
            cell = np.flipud(cell)
            dx = (sx0 - x0) * sc
            dy = (y0 + 2 * hw - sy1) * sc
            img[dy:dy + cell.shape[0], dx:dx + cell.shape[1]] = cell

        def px(x, y):
            return (int((x - rx) / res * sc + W / 2.0),
                    int(H / 2.0 - (y - ry) / res * sc))

        def put(p, color, r=1):
            xa, xb = max(p[0] - r, 0), min(p[0] + r + 1, W)
            ya, yb = max(p[1] - r, 0), min(p[1] + r + 1, H)
            if xa < xb and ya < yb:
                img[ya:yb, xa:xb] = color

        # 边框 + 左下角 1 m 比例尺
        img[0:2, :] = 60
        img[-2:, :] = 60
        img[:, 0:2] = 60
        img[:, -2:] = 60
        bar = int(round(1.0 / res)) * sc
        img[H - 24:H - 16, 12:12 + bar] = 235

        # 轨迹（黄）
        for x, y in traj[-3000:]:
            put(px(x, y), np.array([230, 200, 0], dtype=np.uint8), max(1, int(round(k))))

        # 激光点（红）
        if self.scan is not None:
            try:
                tr = self.tf_buffer.lookup_transform('map', self.scan.header.frame_id, rclpy.time.Time())
                t, q = tr.transform.translation, tr.transform.rotation
                qx, qy, qz, qw = q.x, q.y, q.z, q.w
                for i, r in enumerate(self.scan.ranges):
                    if not math.isfinite(r) or r < self.scan.range_min or r > self.scan.range_max:
                        continue
                    a = self.scan.angle_min + i * self.scan.angle_increment
                    ax_, ay_, az_ = r * math.cos(a), 0.0, r * math.sin(a)
                    ix = qw * ax_ + qy * az_ - qz * ay_
                    iy = qw * ay_ + qz * ax_ - qx * az_
                    iz = qw * az_ + qx * ay_ - qy * ax_
                    iw = -qx * ax_ - qy * ay_ - qz * az_
                    rx2 = ix * qw + iw * -qx + iy * -qz - iz * -qy
                    ry2 = iy * qw + iw * -qy + iz * -qx - ix * -qz
                    put(px(t.x + rx2, t.y + ry2), np.array([255, 60, 60], dtype=np.uint8), max(1, int(round(k))))
            except Exception:
                pass

        # 语义物体（彩色方块）
        SEM_COLORS = {'person': (255, 90, 90), 'cup': (80, 230, 255), 'bottle': (100, 150, 255),
                      'chair': (255, 200, 80), 'dining table': (200, 150, 80), 'couch': (150, 100, 255),
                      'tv': (230, 230, 230), 'laptop': (120, 255, 120), 'book': (255, 150, 220),
                      'cell phone': (180, 180, 255), 'keyboard': (230, 230, 150), 'mouse': (150, 230, 230)}
        with L:
            sem = list(S.get('sem', []))
        for o in sem:
            try:
                sp = px(float(o['x']), float(o['y']))
            except Exception:
                continue
            col = np.array(SEM_COLORS.get(o.get('cls', ''), (255, 220, 120)), dtype=np.uint8)
            put((sp[0], sp[1]), col, max(2, int(round(3 * k))))

        # 目标点（青色圈 + 十字）
        with L:
            gx_gy = S.get('goal_xy')
        if gx_gy:
            gp = px(gx_gy[0], gx_gy[1])
            G = np.array([80, 255, 190], dtype=np.uint8)
            gr = max(4, int(round(6 * k)))
            gc = max(6, int(round(9 * k)))
            for i in range(-gr, gr + 1):
                put((gp[0] + i, gp[1]), G, 0)
                put((gp[0], gp[1] + i), G, 0)
            for ang in range(0, 360, 12):
                a_ = math.radians(ang)
                put((int(gp[0] + gc * math.cos(a_)), int(gp[1] + gc * math.sin(a_))), G, 0)

        # 车（中心点 + 朝向箭头，全部走 px() 保证与地图同向）
        BLUE = np.array([40, 150, 255], dtype=np.uint8)
        put((W // 2, H // 2), BLUE, max(3, int(round(4 * k))))
        tip = px(rx + 0.75 * math.cos(ryaw), ry + 0.75 * math.sin(ryaw))
        steps = max(abs(tip[0] - W // 2), abs(tip[1] - H // 2), 1)
        for i in range(steps + 1):
            put((int(W // 2 + (tip[0] - W // 2) * i / float(steps)),
                 int(H // 2 + (tip[1] - H // 2) * i / float(steps))), BLUE, 1)
        for s in (+1, -1):
            wing = ryaw + s * 2.45
            wpt = px(rx + 0.75 * math.cos(ryaw) + 0.28 * math.cos(wing),
                     ry + 0.75 * math.sin(ryaw) + 0.28 * math.sin(wing))
            st2 = max(abs(wpt[0] - tip[0]), abs(wpt[1] - tip[1]), 1)
            for i in range(st2 + 1):
                put((int(tip[0] + (wpt[0] - tip[0]) * i / float(st2)),
                     int(tip[1] + (wpt[1] - tip[1]) * i / float(st2))), BLUE, 1)

        return png_bytes(W, H, img.tobytes())


NODE = None
SUP = Supervisor()


def save_map(name=None):
    m = NODE.map if NODE else None
    if m is None:
        return None, 'no map yet'
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    g = np.array(m.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 205, dtype=np.uint8)
    known = g >= 0
    img[known] = (255 - np.clip(g[known], 0, 100) * 255 // 100).astype(np.uint8)
    img = img[::-1, :]
    if not name:
        name = 'map_' + time.strftime('%m%d_%H%M%S')
    if not name.startswith('/'):
        name = os.path.join(MAPS, name)
    os.makedirs(os.path.dirname(name), exist_ok=True)
    with open(name + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    with open(name + '.yaml', 'w') as f:
        f.write('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                % (os.path.basename(name), res, ox, oy))
    occ = int((g >= 50).sum())
    with L:
        S['last_save'] = os.path.basename(name)
    return name, 'occupied=%d free=%d unknown=%d %dx%d' % (
        occ, int(((g >= 0) & (g < 50)).sum()), int((g < 0).sum()), w, h)


def start_stack():
    with L:
        if S['state'] in ('MAPPING', 'STARTING'):
            return 'already ' + S['state']
        S['state'] = 'STARTING'
        S['err'] = ''
    def run():
        try:
            SUP.start()
            time.sleep(6)
            with L:
                S['state'] = 'MAPPING'
                S['nav_active'] = True
        except Exception as exc:
            with L:
                S['state'] = 'ERROR'
                S['err'] = str(exc)
    threading.Thread(target=run, daemon=True).start()
    return 'starting'


def stop_stack():
    def run():
        try:
            save_map('autostop_' + time.strftime('%H%M%S'))
        except Exception:
            pass
        SUP.stop()
        with L:
            S['state'] = 'IDLE'
            S['nav_active'] = False
            S['cmd_v'] = S['cmd_w'] = 0.0
    threading.Thread(target=run, daemon=True).start()
    return 'stopping'


def estop():
    # 急停必须同时停掉 Nav2，否则它会继续发速度覆盖零速
    stopped_nav = SUP.stop_nav()
    with L:
        S['manual_v'] = S['manual_w'] = 0.0
        S['manual_ts'] = 0.0
        S['cmd_v'] = S['cmd_w'] = 0.0
        S['nav_active'] = False
    if NODE:
        for _ in range(10):
            NODE.pub_cmd.publish(Twist())
            time.sleep(0.05)
    return 'estopped (nav2 stopped)' if stopped_nav else 'estopped'


PAGE = r"""
<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DGN · RPI5 MAPPING CONSOLE</title>
<style>
:root{
  --bg:#06090c; --panel:#0a0f14; --line:#17242f; --line2:#22323f;
  --cyan:#22d3ee; --cyan-soft:rgba(34,211,238,.10); --cyan-line:rgba(34,211,238,.35);
  --amber:#f5a524; --red:#ef4444; --green:#34d399;
  --txt:#c3d2df; --dim:#5a6d7f;
  --mono:"JetBrains Mono","Cascadia Mono","Consolas","SFMono-Regular",monospace;
}
*{box-sizing:border-box}
html,body{height:100%;margin:0}
body{
  background:
    radial-gradient(1100px 520px at 50% -8%, #0c1822 0%, transparent 62%),
    repeating-linear-gradient(0deg, rgba(34,211,238,.030) 0 1px, transparent 1px 34px),
    repeating-linear-gradient(90deg, rgba(34,211,238,.030) 0 1px, transparent 1px 34px),
    var(--bg);
  color:var(--txt); font:13px/1.5 var(--mono); overflow:hidden;
}
/* ---------- 顶栏 ---------- */
header{
  display:flex; align-items:center; gap:18px; height:46px; padding:0 16px;
  border-bottom:1px solid var(--line);
  background:linear-gradient(180deg, rgba(34,211,238,.05), transparent);
}
.brand{display:flex; align-items:center; gap:8px; font-weight:600; letter-spacing:.12em; color:#e6f4ff; white-space:nowrap}
.brand .dot{width:7px;height:7px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:pulse 1.8s infinite}
.brand .sep{color:var(--cyan-line);font-weight:400}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.25}}
.tele{display:flex; gap:0; margin-left:auto; overflow:hidden}
.tel{display:flex; flex-direction:column; justify-content:center; padding:0 12px; border-left:1px solid var(--line); min-width:74px}
.tel i{font-style:normal; font-size:9px; letter-spacing:.16em; color:var(--dim)}
.tel b{font-size:13px; font-weight:600; color:#e8f6ff; white-space:nowrap}
.tel.clock b{color:var(--cyan)}
/* ---------- 主体 ---------- */
main{display:flex; height:calc(100% - 46px); padding:12px; gap:12px}
.viewport{flex:1; min-width:0; display:flex; flex-direction:column; border:1px solid var(--line); background:var(--panel); position:relative}
.vp-head,.vp-foot{display:flex; justify-content:space-between; align-items:center; padding:6px 10px; font-size:10px; letter-spacing:.14em; color:var(--dim)}
.vp-head{border-bottom:1px solid var(--line)}
.vp-head #vphint{color:var(--green)}
.vp-foot{border-top:1px solid var(--line)}
.vp-body{flex:1; position:relative; display:flex; align-items:center; justify-content:center; overflow:hidden; background:#05080a}
.vp-body img{max-width:100%; max-height:100%; image-rendering:pixelated; display:block; cursor:crosshair}
#nosig{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:10px;width:70%;height:60%;border:1px dashed var(--line2);color:var(--dim);letter-spacing:.3em;font-size:16px}
#nosig span{font-size:11px;letter-spacing:.1em;color:#3f4f5d}
.cross{position:absolute; left:50%; top:50%; width:34px; height:34px; transform:translate(-50%,-50%); pointer-events:none; opacity:.55}
.cross:before,.cross:after{content:""; position:absolute; background:var(--cyan)}
.cross:before{left:50%; top:0; width:1px; height:100%; transform:translateX(-.5px)}
.cross:after{top:50%; left:0; height:1px; width:100%; transform:translateY(-.5px)}
.corner{position:absolute; width:16px; height:16px; border:1px solid var(--cyan-line)}
.corner.tl{left:6px; top:6px; border-right:0; border-bottom:0}
.corner.tr{right:6px; top:6px; border-left:0; border-bottom:0}
.corner.bl{left:6px; bottom:6px; border-right:0; border-top:0}
.corner.br{right:6px; bottom:6px; border-left:0; border-top:0}
/* ---------- 控制面板 ---------- */
.panel{width:268px; flex:none; display:flex; flex-direction:column; gap:8px; padding:12px; border:1px solid var(--line); background:var(--panel)}
.group{display:flex; flex-direction:column; gap:6px}
.group.row{flex-direction:row}
.group.row .btn{flex:1}
.btn{
  position:relative; padding:9px 10px; border:1px solid var(--line2); background:#0d141a;
  color:var(--txt); font:12px/1.2 var(--mono); letter-spacing:.06em; cursor:pointer;
  clip-path:polygon(8px 0,100% 0,100% calc(100% - 8px),calc(100% - 8px) 100%,0 100%,0 8px);
  transition:background .12s,border-color .12s,color .12s,box-shadow .12s;
}
.btn:hover{background:var(--cyan-soft); border-color:var(--cyan-line); color:#e9fbff; box-shadow:0 0 0 1px rgba(34,211,238,.12) inset}
.btn:active{transform:translateY(1px)}
.btn.primary{border-color:var(--cyan); color:var(--cyan); background:rgba(34,211,238,.07)}
.btn.primary:hover{background:rgba(34,211,238,.16)}
.btn.danger{border-color:rgba(239,68,68,.55); color:#ffb4b0}
.btn.danger:hover{background:rgba(239,68,68,.12)}
.estop{
  margin-top:2px; padding:13px; font-size:15px; font-weight:700; letter-spacing:.28em;
  border-color:var(--red); color:#fff; background:linear-gradient(180deg,#7f1d1d,#5b1414);
}
.estop:hover{background:linear-gradient(180deg,#991f1f,#6d1717); box-shadow:0 0 14px rgba(239,68,68,.35)}
.slider{padding:8px 10px; border:1px solid var(--line); background:#080d12}
.slider-head{display:flex; align-items:baseline; gap:6px; font-size:10px; letter-spacing:.12em; color:var(--dim)}
.slider-head b{color:var(--amber); font-size:14px}
input[type=range]{width:100%; margin-top:6px; -webkit-appearance:none; height:3px; background:linear-gradient(90deg,var(--cyan) 0%,var(--cyan) 45%,#1b2733 45%); outline:0}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:12px;height:12px;background:var(--cyan);border-radius:0;box-shadow:0 0 8px rgba(34,211,238,.7)}
#pad{display:grid; grid-template-columns:repeat(3,1fr); gap:6px; margin-top:2px}
#pad .btn{padding:12px 0; font-size:14px; text-align:center; user-select:none; touch-action:none}
.logwrap{flex:1; min-height:70px; display:flex; flex-direction:column; border:1px solid var(--line); background:#05080a}
.loghead{padding:4px 8px; font-size:9px; letter-spacing:.18em; color:var(--dim); border-bottom:1px solid var(--line)}
#log{flex:1; overflow:auto; padding:6px 8px; font-size:11px; color:#8fa6b8; white-space:pre-wrap}
#log .ln{display:block}
#log .ln:before{content:"» "; color:var(--cyan)}
/* ---------- 扫描线 ---------- */
.scanlines{position:fixed; inset:0; pointer-events:none; z-index:9;
  background:repeating-linear-gradient(0deg, rgba(255,255,255,.025) 0 1px, transparent 1px 3px); opacity:.5}
/* ---------- 状态色 ---------- */
.ok{color:var(--green)!important}
.warn{color:var(--amber)!important}
.bad{color:var(--red)!important}
.search{display:flex; gap:6px}
.search input{flex:1; min-width:0; padding:8px 10px; border:1px solid var(--line2); background:#080d12; color:var(--txt); font:12px var(--mono); outline:0}
.search input:focus{border-color:var(--cyan-line)}
.search .btn{padding:8px 10px; flex:none}
.sres{padding:6px 8px; border:1px solid var(--line); background:#05080a; font-size:11px; color:var(--dim); white-space:nowrap; overflow:hidden; text-overflow:ellipsis}
.sres.ok{color:var(--green); border-color:rgba(52,211,153,.35)}
.sres.warn{color:var(--amber); border-color:rgba(245,165,36,.35)}
.sres.bad{color:var(--red); border-color:rgba(239,68,68,.35)}
.zoombar{position:absolute; right:10px; top:10px; display:flex; align-items:center; gap:5px; padding:3px 8px; background:rgba(6,10,14,.8); border:1px solid var(--line); font-size:11px; z-index:5}
.zoombar button{width:22px; height:22px; padding:0; font-size:13px; line-height:1; border:1px solid var(--line2); background:#0b1218; color:var(--txt); cursor:pointer}
.zoombar button:hover{border-color:var(--cyan-line); color:var(--cyan)}
.zoombar b{min-width:40px; text-align:center; color:var(--cyan); font-weight:600}</style>
</head>
<body>
<header>
  <div class="brand"><span class="dot"></span>DGN<span class="sep">/</span>RPI5<span class="sep">/</span>MAPPING CONSOLE</div>
  <div class="tele">
    <div class="tel"><i>STATE</i><b id="st">--</b></div>
    <div class="tel"><i>MODE</i><b id="mode">--</b></div>
    <div class="tel"><i>MAP</i><b id="map">--</b></div>
    <div class="tel" style="display:none"><i>DETECT</i><b id="det">--</b></div>
    <div class="tel"><i>BATT</i><b id="bat">--</b></div>
    <div class="tel"><i>TEMP</i><b id="tmp">--</b></div>
    <div class="tel"><i>VEL</i><b id="vel">--</b></div>
    <div class="tel"><i>ODO</i><b id="dist">--</b></div>
    <div class="tel clock"><i>CLOCK</i><b id="clk">--:--:--</b></div>
  </div>
</header>

<main>
  <section class="viewport">
    <div class="vp-head"><span>D455F · /SCAN · SLAM_TOOLBOX · NAV2</span><span id="vphint">LIVE</span></div>
    <div class="vp-body">
      <img id="img" alt="">
      <div class="zoombar">
        <button id="zin" title="放大">＋</button>
        <b id="zval">8 m</b>
        <button id="zout" title="缩小">－</button>
      </div>
      <div id="nosig">NO SIGNAL<span>等待「开始建图」</span></div>
      <div class="cross" id="cross"></div>
      <div class="corner tl"></div><div class="corner tr"></div>
      <div class="corner bl"></div><div class="corner br"></div>
    </div>
    <div class="vp-foot"><span>RES 0.05 M/CELL</span><span>FOV 78.8°</span><span style="color:var(--cyan)">◈ 点击地图发送导航目标</span><span>SCALE 1 M</span></div>
  </section>

  <aside class="panel">
    <div class="group">
      <button id="start" class="btn primary">▶ 开始建图</button>
      <button id="stopb" class="btn danger">■ 停止并保存</button>
      <button id="save" class="btn">💾 立即存图</button>
    </div>
    <div class="group row">
      <button id="auto" class="btn">🤖 自动探索</button>
      <button id="man" class="btn">✋ 手动接管</button>
    </div>
    <div class="search">
      <input id="em" type="number" value="10" min="1" max="120" step="1" title="探索时长（分钟）">
      <span style="font-size:11px;align-self:center;color:#7a8a99;padding:0 6px">分钟</span>
    </div>
    <div class="group row">
      <button id="navresume" class="btn">♻ 恢复导航</button>
      <button id="navcancel" class="btn">✖ 取消目标</button>
    </div>
    <div class="search" style="display:none">
      <input id="q" type="text" placeholder="找什么？">
      <button id="go" class="btn primary">🔍</button>
    </div>
    <div id="sres" class="sres">待命</div>
    <div class="slider">
      <div class="slider-head"><span>速度上限</span><b id="vmax">0.12</b><span>m/s</span></div>
      <input type="range" id="spd" min="4" max="20" value="12">
    </div>
    <div id="pad">
      <div></div><button class="btn" data-k="w">↑</button><div></div>
      <button class="btn" data-k="a">←</button><button class="btn" data-k="s">↓</button><button class="btn" data-k="d">→</button>
    </div>
    <button id="estop" class="btn estop">STOP</button>
    <div class="logwrap"><div class="loghead">EVENT LOG</div><div id="log"></div></div>
  </aside>
</main>
<div class="scanlines"></div>

<script>
let st={}, keys=new Set(), vmax=0.12;
const $=id=>document.getElementById(id);
const log=m=>{const d=$('log');const s=document.createElement('span');s.className='ln';s.textContent=new Date().toLocaleTimeString()+' '+m;d.prepend(s);while(d.childElementCount>60)d.lastChild.remove();};
async function post(u){try{const r=await fetch(u,{method:'POST'});log(await r.text());}catch(e){log('ERR '+e)}}
$('start').onclick=()=>post('/start');
$('stopb').onclick=()=>post('/stop');
$('save').onclick=()=>post('/save');
$('auto').onclick=()=>{const e=$('em');const m=(e&&e.value!=='')?e.value:10;post('/explore/start?min='+encodeURIComponent(m));};
const ZOOMS=[2,3,4,6,8,12,16,24];
let zi=ZOOMS.indexOf(8); if(zi<0) zi=4;
let zoomInit=false;
function applyZoom(send){ $('zval').textContent=ZOOMS[zi]+' m'; if(send) post('/zoom?m='+ZOOMS[zi]); }
$('zin').onclick=()=>{ if(zi>0){zi--;} applyZoom(true); };
$('zout').onclick=()=>{ if(zi<ZOOMS.length-1){zi++;} applyZoom(true); };
(function(){ const vb=document.querySelector('.vp-body'); if(vb) vb.addEventListener('wheel',e=>{e.preventDefault(); if(e.deltaY<0){if(zi>0)zi--;}else{if(zi<ZOOMS.length-1)zi++;} applyZoom(true);},{passive:false}); })();
$('man').onclick=()=>post('/mode?m=manual');
$('navresume').onclick=()=>post('/nav/start');
$('navcancel').onclick=()=>post('/goal/cancel');
$('go').onclick=()=>{const q=$('q').value.trim(); if(!q) return; fetch('/search?q='+encodeURIComponent(q),{method:'POST'}).then(r=>r.text()).then(t=>log(t));};
$('q').addEventListener('keydown',e=>{if(e.key==='Enter') $('go').click();});
$('estop').onclick=()=>post('/estop');
$('spd').oninput=e=>{vmax=e.target.value/100;$('vmax').textContent=vmax.toFixed(2);};
addEventListener('keydown',e=>{const k=e.key.toLowerCase();if('wasd'.includes(k)){keys.add(k);e.preventDefault();}});
addEventListener('keyup',e=>{const k=e.key.toLowerCase();if('wasd'.includes(k)){keys.delete(k);send(0,0);}});
document.querySelectorAll('#pad button').forEach(b=>{
  const k=b.dataset.k;
  const down=e=>{e.preventDefault();keys.add(k);};
  const up=e=>{keys.delete(k);send(0,0);};
  b.addEventListener('pointerdown', down);
  b.addEventListener('pointerup', up);
  b.addEventListener('pointercancel', up);
  b.addEventListener('pointerleave', up);
});
function send(v,w){fetch('/drive?v='+v+'&w='+w,{method:'POST'}).catch(()=>{});}
// 安全：窗口失焦或切到后台，立刻清空按键并停车
function stopAll(){ keys.clear(); send(0,0); }
addEventListener('blur', stopAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
addEventListener('pagehide', stopAll);
setInterval(()=>{
  if(keys.size===0) return;
  let v=0,w=0;
  if(keys.has('w'))v=vmax; if(keys.has('s'))v=-vmax*0.6;
  if(keys.has('a'))w=0.6; if(keys.has('d'))w=-0.6;
  send(v.toFixed(3),w.toFixed(3));
},100);
setInterval(()=>{$('clk').textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});},1000);
$('img').addEventListener('click', ev => {
  const im = $('img');
  if (!im.naturalWidth) return;
  const r = im.getBoundingClientRect();
  const px = (ev.clientX - r.left) / r.width * im.naturalWidth;
  const py = (ev.clientY - r.top) / r.height * im.naturalHeight;
  // 把渲染图的原始尺寸一起传给服务端，保证任意缩放下点击点=导航点
  fetch('/goal?px=' + px.toFixed(1) + '&py=' + py.toFixed(1)
        + '&w=' + im.naturalWidth + '&h=' + im.naturalHeight, {method:'POST'})
    .then(res => res.text())
    .then(txt => log(txt === 'ok' ? ('目标已发送 (' + px.toFixed(0) + ',' + py.toFixed(0) + ')') : ('目标失败: ' + txt)))
    .catch(e => log('目标请求失败 ' + e));
});
async function tick(){
  try{
    st=await (await fetch('/state')).json();
    $('st').textContent=st.state;
    $('st').className=(st.state==='MAPPING'?'ok':(st.state==='ERROR'?'bad':'warn'));
    $('mode').textContent=(st.mode==='manual'?'MANUAL':'AUTO')+(st.nav_active?' · NAV2':'');
    $('mode').className=(st.nav_active?'ok':'');
    $('map').textContent=st.map_w?st.map_w+'×'+st.map_h:'--';
    const d=st.det||{};
    const nobj=(st.sem||[]).length;
    $('det').textContent = d.running ? ((d.fps||0).toFixed(0)+'fps·'+nobj) : 'OFF';
    $('det').className = d.running ? 'ok' : 'warn';
    const ex=st.explore||{};
    if(ex.msg!==undefined){
      const on=!!ex.enabled;
      $('sres').textContent = (on?'[EXPLORING]':'[IDLE]')+'  已访:'+(ex.visited||0)+'  黑:'+(ex.black||0)+'  发送:'+(ex.sent||0)+'  失败:'+(ex.fail||0)+'  '+(ex.msg||'');
      $('sres').className = 'sres ' + (on?'ok':'');
    }
    $('bat').textContent=st.battery?st.battery.toFixed(2)+'V':'--';
    $('bat').className=(st.battery&&st.battery<11.5?'bad':'');
    $('tmp').textContent=st.temp?st.temp.toFixed(0)+'°C':'--';
    $('tmp').className=(st.temp&&st.temp>78?'bad':(st.temp&&st.temp>72?'warn':''));
    $('vel').textContent=st.cmd_v.toFixed(2)+'/'+st.cmd_w.toFixed(2);
    $('dist').textContent=st.dist.toFixed(2)+'m';
    if(st.view_m && !zoomInit){ zoomInit=true; const k=ZOOMS.indexOf(Math.round(st.view_m)); if(k>=0){ zi=k; } applyZoom(false); }
    if(st.map_w){ $('nosig').style.display='none'; $('cross').style.display='block'; $('img').style.display='block'; $('img').src='/map.png?t='+Date.now(); }
    else { $('nosig').style.display='flex'; $('cross').style.display='none'; $('img').style.display='none'; }
    $('vphint').textContent=st.fps&&st.fps.scan>0?('LIVE '+st.fps.scan.toFixed(0)+'HZ'):'NO DATA';
    $('vphint').className=(st.fps&&st.fps.scan>0?'ok':'bad');
  }catch(e){ $('st').textContent='OFFLINE'; $('st').className='bad'; }
}
log('console ready');
tick(); setInterval(tick,1000);
</script>
</body>
</html>
"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype='text/plain; charset=utf-8'):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == '/':
            return self._send(200, PAGE, 'text/html; charset=utf-8')
        if u.path == '/state':
            with L:
                d = {
                    'state': S['state'], 'mode': S['mode'], 'err': S['err'],
                    'battery': S['battery'], 'dist': S['dist'],
                    'cmd_v': S['cmd_v'], 'cmd_w': S['cmd_w'],
                    'fps': S['fps'], 'last_save': S['last_save'],
                    'traj': len(S['traj']), 'nav_active': S['nav_active'],
                    'manual_age': round(time.time() - S['manual_ts'], 3), 'sem': S.get('sem', []), 'det': S.get('det', {}), 'search': S.get('search', {}), 'explore': S.get('explore', {}), 'view_m': S.get('view_m', VIEW_M), 'pose': S.get('odom'), 'goal_xy': S.get('goal_xy'),
                    'ticks': S.get('ticks', 0),
                }
                mp = S['map']
                if mp:
                    d['map_w'], d['map_h'] = mp[0], mp[1]
            try:
                with open('/sys/class/thermal/thermal_zone0/temp') as f:
                    d['temp'] = int(f.read().strip()) / 1000.0
            except Exception:
                d['temp'] = None
            return self._send(200, json.dumps(d), 'application/json')
        if u.path == '/map.png':
            now = time.time()
            with L:
                cached, ts = S['png'], S['png_ts']
            if NODE and (cached is None or now - ts > 0.4):
                png = NODE.render()
                if png:
                    with L:
                        S['png'], S['png_ts'] = png, now
                    cached = png
            if cached is None:
                return self._send(503, 'no map yet')
            return self._send(200, cached, 'image/png')
        if u.path == '/files':
            try:
                fs = sorted(os.listdir(MAPS))
            except Exception:
                fs = []
            return self._send(200, json.dumps(fs), 'application/json')
        if u.path == '/download':
            f = q.get('f', [''])[0]
            p = os.path.join(MAPS, os.path.basename(f))
            if os.path.isfile(p):
                with open(p, 'rb') as fh:
                    return self._send(200, fh.read(), 'application/octet-stream')
            return self._send(404, 'not found')
        return self._send(404, 'not found')

    def do_POST(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == '/start':
            return self._send(200, start_stack())
        if u.path == '/stop':
            return self._send(200, stop_stack())
        if u.path == '/save':
            name = q.get('name', [None])[0]
            try:
                p, info = save_map(name)
                return self._send(200, 'saved %s %s' % (p, info) if p else 'FAIL ' + str(info))
            except Exception as exc:
                return self._send(500, 'FAIL %s' % exc)
        if u.path == '/nav/start':
            try:
                with L:
                    S['manual_v'] = S['manual_w'] = 0.0
                    S['manual_ts'] = 0.0
                SUP.start_nav()
                with L:
                    S['nav_active'] = True
                return self._send(200, 'nav started')
            except Exception as exc:
                return self._send(500, 'FAIL %s' % exc)
        if u.path == '/explore/start':
            try:
                mins = float(q.get('min', ['10'])[0])
            except Exception:
                mins = 10.0
            if NODE is None:
                return self._send(500, 'node not ready')
            f = Float32()
            f.data = float(mins)
            NODE.pub_duration.publish(f)
            b = Bool()
            b.data = True
            NODE.pub_explore.publish(b)
            with L:
                S['mode'] = 'auto'
                S['nav_active'] = True
            return self._send(200, 'explore %.0f min' % mins)
        if u.path == '/zoom':
            try:
                vm = float(q.get('m', ['8'])[0])
            except Exception:
                vm = 8.0
            vm = max(2.0, min(24.0, vm))
            with L:
                S['view_m'] = vm
            return self._send(200, 'zoom %.1f m' % vm)
        if u.path == '/goal':
            if NODE is None:
                return self._send(500, 'node not ready')
            try:
                px_ = float(q.get('px', ['0'])[0])
                py_ = float(q.get('py', ['0'])[0])
                w_ = float(q.get('w', ['0'])[0])
                h_ = float(q.get('h', ['0'])[0])
            except Exception:
                return self._send(400, 'bad px/py')
            r = NODE.send_nav_goal_px(px_, py_, w_, h_)
            if r == 'NAV2_NOT_READY':
                return self._send(409, 'Nav2 未就绪：先点「恢复导航」')
            if r == 'TOO_FAST':
                return self._send(429, '点击太快，已忽略（0.8 秒内只认第一次）')
            if r == 'UNKNOWN_AREA':
                return self._send(409, '该点还未探索（未知区域），请点已建图的白色区域')
            if r == 'BLOCKED_AREA':
                return self._send(409, '该点是障碍物，请点白色空闲区域')
            if r == 'OUT_OF_MAP':
                return self._send(409, '该点超出地图范围')
            if r == 'ok':
                # 点击即导航：自动进入导航模式，控制台停止发零速，交给 Nav2 驱动
                with L:
                    S['nav_active'] = True
                    S['mode'] = 'auto'
                    S['manual_v'] = S['manual_w'] = 0.0
                    S['manual_ts'] = 0.0
            return self._send(200, r)
        if u.path == '/goal/cancel':
            if NODE is None:
                return self._send(500, 'node not ready')
            return self._send(200, NODE.cancel_nav())
        if u.path == '/search':
            qs = q.get('q', [''])[0]
            if NODE is None:
                return self._send(500, 'node not ready')
            msg = String()
            msg.data = qs
            NODE.pub_sem_cmd.publish(msg)
            return self._send(200, 'search: %s' % qs)
        if u.path == '/estop':
            return self._send(200, estop())
        if u.path == '/drive':
            try:
                v = clamp(float(q.get('v', ['0'])[0]), MAXV)
                w = clamp(float(q.get('w', ['0'])[0]), MAXW)
            except Exception:
                v = w = 0.0
            with L:
                S['manual_v'], S['manual_w'] = v, w
                S['manual_ts'] = time.time()
                S['mode'] = 'manual'
            return self._send(200, 'ok %.3f %.3f ts=%.3f' % (v, w, S['manual_ts']))
        if u.path == '/mode':
            m = q.get('m', ['auto'])[0]
            if m in ('auto', 'manual'):
                with L:
                    S['mode'] = m
                    if m == 'manual':
                        S['manual_v'] = S['manual_w'] = 0.0
                        S['manual_ts'] = 0.0
                # 自动探索交给 explorer + Nav2；手动则收回控制权
                if NODE is not None:
                    b = Bool()
                    b.data = (m == 'auto')
                    NODE.pub_explore.publish(b)
                with L:
                    S['nav_active'] = (m == 'auto')
                return self._send(200, 'mode ' + m)
            return self._send(400, 'bad mode')
        return self._send(404, 'not found')


def autosave_loop():
    while True:
        time.sleep(AUTOSAVE_SEC)
        try:
            if NODE and NODE.map is not None:
                save_map('autosave')
        except Exception:
            pass


def main():
    global NODE
    os.makedirs(MAPS, exist_ok=True)
    os.makedirs(LOGDIR, exist_ok=True)
    rclpy.init()
    NODE = ConsoleNode()
    ex = MultiThreadedExecutor()
    ex.add_node(NODE)
    threading.Thread(target=ex.spin, daemon=True).start()
    threading.Thread(target=autosave_loop, daemon=True).start()
    try:
        srv = ThreadingHTTPServer(('0.0.0.0', PORT), H)
    except OSError as exc:
        print('[错误] 无法监听端口 %d: %s' % (PORT, exc))
        print('       常见原因：已有一个 map_console 在跑。')
        print('       处理：pkill -f map_console  然后重新执行 start_console.sh')
        return
    print('console on http://0.0.0.0:%d/' % PORT, flush=True)
    srv.serve_forever()


main()
