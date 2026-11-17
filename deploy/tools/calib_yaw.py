#!/usr/bin/env python3
"""360° 手推标定：推车转一整圈，测 IMU 积分角，给出比例修正。

用法：
  1. 确保桥接在跑（ros2 run wheeltec_bridge bridge）
  2. python3 /home/pi/calib_yaw.py
  3. 按提示把车**缓慢推一整圈**（约 360°），然后停住
  4. 看输出：会给出「实测角度」和「建议 gyro_scale」
"""
import math
import select
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

import subprocess

DEFAULT_SCALE = 0.0002663   # ±500 dps


def read_bridge_scale():
    # 自动读取桥接实际使用的 gyro_scale，避免脚本与桥接不一致
    try:
        out = subprocess.run(['ros2', 'param', 'get', '/wheeltec_bridge', 'gyro_scale'],
                             capture_output=True, text=True, timeout=6).stdout
        for line in out.splitlines():
            if 'value is' in line:
                return float(line.split(':')[-1])
    except Exception:
        pass
    return DEFAULT_SCALE


SCALE = DEFAULT_SCALE
TARGET_DEG = 360.0        # 目标：手推一整圈


class Calib(Node):
    def __init__(self):
        super().__init__('calib_yaw')
        self.bias = 0.0
        self.prev_t = None
        self.ang = 0.0
        self.moving = False
        self.last_move = 0.0
        self.samples = 0
        self.yaw0 = None
        self.yaw1 = None
        self.create_subscription(Int16MultiArray, '/wheeltec/imu_raw', self.on_imu, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        y = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        if self.yaw0 is None:
            self.yaw0 = y
        self.yaw1 = y

    def on_imu(self, m):
        t = time.time()
        g = float(m.data[5])
        self.samples += 1
        if self.prev_t is not None:
            dt = t - self.prev_t
            if 0.0 < dt < 0.5:
                w = (g - self.bias) * SCALE
                if abs(w) > 0.04:
                    self.ang += w * dt
                    self.moving = True
                    self.last_move = t
        self.prev_t = t


def main():
    global SCALE
    SCALE = read_bridge_scale()
    print('使用 gyro_scale = %.7f' % SCALE)
    has_tty = bool(sys.stdin and sys.stdin.isatty())
    rclpy.init()
    n = Calib()
    print('等待 IMU 数据...')
    t0 = time.time()
    while n.samples < 20 and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.samples < 20:
        print('✗ 没有 /wheeltec/imu_raw（桥接没起？）')
        return 1

    print('1) 标定零偏（请保持车静止 2 秒）...')
    t0 = time.time()
    vals = []
    while time.time() - t0 < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        vals.append(0)
    n.bias = 0.0
    # 重新采 1.5 秒静止数据求均值
    import collections
    raw = collections.deque(maxlen=40)
    orig = n.on_imu

    def capture(m):
        raw.append(float(m.data[5]))
        orig(m)
    n.create_subscription(Int16MultiArray, '/wheeltec/imu_raw', capture, 50)
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(n, timeout_sec=0.05)
    if raw:
        n.bias = float(np.mean(raw))
    sd = float(np.std(raw)) if raw else 999.0
    print('   零偏 = %.1f counts（波动 %.1f）' % (n.bias, sd))
    if abs(n.bias) > 100 or sd > 40:
        print('')
        print('✗ 标定时车在动（零偏 %.0f，波动 %.0f）' % (n.bias, sd))
        print('  请把手完全离开车，静止 3 秒后重跑')
        n.destroy_node()
        rclpy.shutdown()
        return 3

    print('')
    print('2) 现在请把车【推一整圈】（360°），回到起点标记处停下')
    print('   · 脚本会自己数圈，看到「1.00 圈」就停')
    print('   · 中途停顿不会结束')
    if has_tty:
        print('   · 想提前结束，直接按【回车】')
    print('')
    t_start = time.time()
    last_print = 0.0
    over_warned = False
    while time.time() - t_start < 240:
        rclpy.spin_once(n, timeout_sec=0.05)
        now = time.time()
        deg_now = math.degrees(n.ang)
        turns = abs(deg_now) / 360.0
        if now - last_print > 0.3:
            print('   %+7.1f°  =  %.2f 圈' % (deg_now, turns), end='\r')
            last_print = now
        # 回车可提前结束
        if has_tty and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            print('')
            print('[收到回车，提前结束]')
            break
        # 只有"约一整圈"且停稳才算完成
        if n.moving and now - n.last_move > 2.0 and 340 <= abs(deg_now) <= 380:
            print('')
            print('[一圈完成，已停稳]')
            break
        # 超过一圈就提醒
        if abs(deg_now) > 400 and not over_warned:
            print('')
            print('  ⚠ 已经超过一圈（%.2f 圈），请在起点标记处停下，或按回车用当前角度' % turns)
            over_warned = True
            n.last_move = now
        # 停了但不够一圈
        if n.moving and now - n.last_move > 6.0 and abs(deg_now) < 340:
            print('')
            print('  ⚠ 已静止，但只有 %.2f 圈；继续推满一圈' % turns)
            n.last_move = now
    print('')
    deg = math.degrees(n.ang)
    turns = abs(deg) / 360.0
    print('=== 结果 ===')
    print('IMU 实测转角: %+.1f°  =  %.3f 圈' % (deg, turns))
    if n.yaw0 is not None and n.yaw1 is not None:
        d = math.degrees(n.yaw1 - n.yaw0)
        d = (d + 180) % 360 - 180
        print('里程计航向:   %+.1f°（仅供参考，手推时轮子会拖）' % d)
    if abs(deg) < 300:
        print('')
        print('✗ 不足一圈，这次不能用来定标；请重跑并推满一圈')
        return 2
    # 按圈数归一：如果推了 N 圈，每圈对应角度 = |deg| / N
    per_turn = abs(deg) / round(turns) if round(turns) >= 1 else abs(deg)
    factor = TARGET_DEG / per_turn
    new_scale = SCALE * factor
    print('')
    print('按 %d 圈计算，每圈 %.1f°' % (round(turns), per_turn))
    print('修正系数: %.4f' % factor)
    print('建议新 gyro_scale = %.6f' % new_scale)
    print('建议新 gyro_scale = %.6f' % new_scale)
    print('')
    print('如果实测接近 360°（±10°内），说明比例已经对了，不用改。')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
