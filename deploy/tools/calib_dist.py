#!/usr/bin/env python3
"""用墙当尺子，标定轮式平移比例。

原理：车朝墙前进，扫描中心波束的距离减少量 = 实际位移。
      和里程计报告的位移相比，就得到比例误差。

前提：车头正前方 ≥1.5 m 有一面平整的墙或柜子，且地面干净。
用法：python3 /home/pi/calib_dist.py
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

TARGET = 0.5      # 目标前进距离（m）
SPEED = 0.10      # 速度（m/s）


class Calib(Node):
    def __init__(self):
        super().__init__('calib_dist')
        self.scan = None
        self.ox = self.oy = None
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_scan(self, m):
        self.scan = m

    def on_odom(self, m):
        self.ox = m.pose.pose.position.x
        self.oy = m.pose.pose.position.y

    def front_range(self):
        """中心 ±8 束的中位数，返回 None 表示无效"""
        if self.scan is None:
            return None
        r = np.array(self.scan.ranges, dtype=float)
        n = len(r)
        c = n // 2
        seg = r[max(0, c - 8):c + 9]
        seg = seg[np.isfinite(seg)]
        seg = seg[(seg >= self.scan.range_min) & (seg <= self.scan.range_max)]
        return float(np.median(seg)) if len(seg) else None


def main():
    rclpy.init()
    n = Calib()
    print('等待扫描...')
    t0 = time.time()
    while (n.scan is None or n.ox is None) and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.scan is None or n.ox is None:
        print('✗ 没有 /scan 或 /odom（栈没起？）')
        return 1

    r0 = n.front_range()
    if r0 is None or r0 < 1.2:
        print('✗ 前方只有 %.2f m，需要 ≥1.5 m 空地' % (r0 or -1))
        return 2
    print('前方墙距离: %.2f m' % r0)

    x0, y0 = n.ox, n.oy
    print('前进 %.2f m（%.2f m/s）...' % (TARGET, SPEED))
    tw = Twist()
    tw.linear.x = SPEED
    t_start = time.time()
    while time.time() - t_start < 15:
        n.pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.05)
        d = math.hypot(n.ox - x0, n.oy - y0)
        if d >= TARGET:
            break
        if n.front_range() is not None and n.front_range() < 0.5:
            print('  前方太近了，提前停')
            break
    tw.linear.x = 0.0
    for _ in range(10):
        n.pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.03)
    time.sleep(0.5)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)

    r1 = n.front_range()
    odom = math.hypot(n.ox - x0, n.oy - y0)
    if r1 is None:
        print('✗ 结束时扫描无效')
        return 3
    actual = r0 - r1
    print('')
    print('=== 结果 ===')
    print('墙距离: %.3f m → %.3f m' % (r0, r1))
    print('实际位移: %.3f m' % actual)
    print('里程计:   %.3f m' % odom)
    if odom < 0.05:
        print('✗ 车没动，检查底盘')
        return 4
    factor = actual / odom
    print('比例: 实际/里程计 = %.4f' % factor)
    if abs(factor - 1.0) < 0.03:
        print('✅ 平移比例正常（±3%），偏差不是这里造成的')
    else:
        print('⚠ 里程计平移偏差 %.1f%%，需要在桥接里把 vx 乘 %.4f' % ((factor - 1) * 100, factor))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
