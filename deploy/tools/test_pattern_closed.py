#!/usr/bin/env python3
"""漂移诊断用标准路线（闭环版）

与开环版区别：用 /odom 反馈控制距离和转角，消除"电机死区导致少转 15°"的问题。

  ① 前进到 odom 位移 2.00 m
  ② 停 6 s（可测量）
  ③ 原地转到 odom 航向变化 180°
  ④ 停 6 s（可测量）
  ⑤ 前进到 odom 位移 2.00 m（回到起点）
  ⑥ 停

安全：/scan 正前方 ±25° 内 < 0.45 m 立即停车。
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

V_FWD = 0.10
W_TURN = 0.15   # 慢转，减少滑移
DIST = 2.0
ANGLE = math.pi
SAFE_DIST = 0.45


class Driver(Node):
    def __init__(self):
        super().__init__('test_pattern')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.odom = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

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

    def spin(self, sec=0.02):
        rclpy.spin_once(self, timeout_sec=sec)

    # 前进 dist 米（odom 闭环）
    def forward(self, dist, label):
        if self.odom is None:
            print('  无 odom，退出'); return False
        x0, y0, _ = self.odom
        print('  %-18s 目标 %.2f m' % (label, dist), flush=True)
        t0 = time.time()
        while time.time() - t0 < 60:
            self.spin()
            if self.odom is None:
                continue
            d = math.hypot(self.odom[0] - x0, self.odom[1] - y0)
            if d >= dist:
                break
            fm = self.front_min()
            if fm < SAFE_DIST:
                print('  ⚠ 前方 %.2f m 有障碍，停车' % fm, flush=True)
                self.send(0.0, 0.0); return False
            self.send(V_FWD, 0.0)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        print('    完成: 位移 %.3f m, 用时 %.1f s' % (d, time.time() - t0), flush=True)
        return True

    # 左转 angle 弧度（odom 闭环）
    def turn(self, angle, label):
        if self.odom is None:
            return False
        y0 = self.odom[2]
        acc = 0.0
        prev = y0
        print('  %-18s 目标 %.1f°' % (label, math.degrees(angle)), flush=True)
        t0 = time.time()
        while time.time() - t0 < 40:
            self.spin()
            y = self.odom[2]
            d = y - prev
            while d > math.pi: d -= 2 * math.pi
            while d < -math.pi: d += 2 * math.pi
            acc += d
            prev = y
            if abs(acc) >= angle - math.radians(0.5):
                break
            self.send(0.0, W_TURN if acc >= 0 else -W_TURN)
            time.sleep(0.05)
        self.send(0.0, 0.0)
        print('    完成: 转了 %.1f°, 用时 %.1f s' % (math.degrees(acc), time.time() - t0), flush=True)
        return True

    def pause(self, sec, label):
        print('  %-18s %.0f 秒（可测量）' % (label, sec), flush=True)
        t0 = time.time()
        while time.time() - t0 < sec:
            self.spin()
            self.send(0.0, 0.0)


def main():
    rclpy.init()
    n = Driver()
    print('=== 等待 odom 数据 ===', flush=True)
    t0 = time.time()
    while n.odom is None and time.time() - t0 < 10:
        n.spin(0.1)
    if n.odom is None:
        print('✗ 没有 odom，退出'); return
    print('=== 10 秒后开始，请在车头做标记 ===', flush=True)
    n.pause(10, '预热')
    print('=== 开始 ===', flush=True)
    if not n.forward(DIST, '①前进 2m'):
        rclpy.shutdown(); return
    n.pause(6, '②停')
    n.turn(ANGLE, '③转 180°')
    n.pause(6, '④停')
    if not n.forward(DIST, '⑤返回 2m'):
        rclpy.shutdown(); return
    n.pause(2, '⑥停')
    print('=== 完成 ===', flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
