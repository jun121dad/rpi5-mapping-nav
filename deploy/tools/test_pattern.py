#!/usr/bin/env python3
"""漂移诊断用标准路线（开环定时，保证可重复）

  ① 前进 2.0 m @ 0.10 m/s  (20 s)
  ② 停 2 s
  ③ 原地转 180° @ 0.30 rad/s (10.5 s)
  ④ 停 2 s
  ⑤ 前进 2.0 m @ 0.10 m/s  (20 s)
  ⑥ 停

安全：/scan 正前方 ±25° 内 < 0.45 m 立即停车退出。
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

V_FWD = 0.10
W_TURN = 0.30
DIST = 2.0
T_FWD = DIST / V_FWD          # 20 s
T_TURN = math.pi / W_TURN     # 10.47 s
STOP_T = 2.0
SAFE_DIST = 0.45


class Driver(Node):
    def __init__(self):
        super().__init__('test_pattern')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        self.scan = m

    def front_min(self):
        sc = self.scan
        if sc is None:
            return 99.0
        best = 99.0
        for i, r in enumerate(sc.ranges):
            a = sc.angle_min + i * sc.angle_increment
            if -0.44 < a < 0.44 and math.isfinite(r) and r > 0.05:
                best = min(best, r)
        return best

    def send(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)

    def hold(self, v, w, dur, label, safety=False):
        print('  %-14s v=%.2f w=%.2f  %.1fs' % (label, v, w, dur), flush=True)
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)
            if safety:
                fm = self.front_min()
                if fm < SAFE_DIST:
                    print('  ⚠ 前方 %.2f m 有障碍，紧急停车！' % fm, flush=True)
                    self.send(0.0, 0.0)
                    return False
            self.send(v, w)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        return True


def main():
    rclpy.init()
    n = Driver()
    print('=== 10 秒后开始，请在车头位置做标记 ===', flush=True)
    n.hold(0.0, 0.0, 10.0, '预热/标记')
    print('=== 开始 ===', flush=True)
    ok = n.hold(V_FWD, 0.0, T_FWD, '①前进 2m', safety=True)
    if not ok:
        rclpy.shutdown(); return
    n.hold(0.0, 0.0, 8.0, '②停 — 可以测量直行距离')
    n.hold(0.0, W_TURN, T_TURN, '③转 180°')
    n.hold(0.0, 0.0, 8.0, '④停 — 可以测量转角')
    n.hold(V_FWD, 0.0, T_FWD, '⑤返回 2m', safety=True)
    n.hold(0.0, 0.0, 2.0, '⑥停')
    print('=== 完成 ===', flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
