#!/usr/bin/env python3
"""判断相机视线与车头方向是否一致。

用法：
  1) 在车头正前方 0.5 m 处放一个纸箱/挡板（车头 = 给 +x 速度时车前进的方向）
  2) python3 /home/pi/check_align.py
  3) 看输出里的"最近障碍方位角"：
       0°   → 相机正对车头        ✅ 装对了
     +90°  → 相机朝向车的左侧    ❌ 差 90°
     -90°  → 相机朝向车的右侧    ❌ 差 90°
     180°  → 相机装反了          ❌ 差 180°
"""
import math
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def rot(q, v):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    ix = w * vx + y * vz - z * vy
    iy = w * vy + z * vx - x * vz
    iz = w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz
    return (ix * w + iw * -x + iy * -z - iz * -y,
            iy * w + iw * -y + iz * -x - ix * -z,
            iz * w + iw * -z + ix * -y - iy * -x)


class Checker(Node):
    def __init__(self):
        super().__init__('check_align')
        self.buf = tf2_ros.Buffer()
        self.tf = tf2_ros.TransformListener(self.buf, self)
        self.scan = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

    def on_scan(self, m):
        self.scan = m


def main():
    rclpy.init()
    n = Checker()
    print('等待 /scan ...')
    t0 = time.time()
    while n.scan is None and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.scan is None:
        print('✗ 没有 /scan（栈没起？）')
        return 1

    m = n.scan
    tr = None
    t1 = time.time()
    while tr is None and time.time() - t1 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tr = n.buf.lookup_transform('base_link', m.header.frame_id, rclpy.time.Time())
        except Exception:
            pass
    if tr is None:
        print('✗ TF base_link <- %s 不可用（等了 10 秒）' % m.header.frame_id)
        return 2

    t, q = tr.transform.translation, tr.transform.rotation
    best = None
    nvalid = 0
    for i, r in enumerate(m.ranges):
        if not math.isfinite(r) or r < m.range_min or r > m.range_max:
            continue
        nvalid += 1
        a = m.angle_min + i * m.angle_increment
        p = rot(q, (r * math.cos(a), r * math.sin(a), 0.0))
        bx, by, bz = t.x + p[0], t.y + p[1], t.z + p[2]
        d = math.hypot(bx, by)
        if best is None or d < best[0]:
            best = (d, math.degrees(math.atan2(by, bx)), bz)

    if best is None:
        print('✗ 没有任何有效回波')
        return 3

    d, ang, z = best
    print('\n有效回波: %d / %d 束' % (nvalid, len(m.ranges)))
    print('最近障碍: 距离 %.2f m, 方位 %+.1f°, 高度 %+.2f m' % (d, ang, z))
    if abs(d - 0.5) > 0.35:
        print('⚠️ 最近障碍不在 0.5 m 处，请把纸箱放到车头正前方 0.5 m 再测')
    if abs(ang) < 25:
        print('判定: ✅ 相机与车头方向一致')
    elif 60 < abs(ang) < 120:
        print('判定: ❌ 相机与车头差 90°（%s）' % ('朝左' if ang > 0 else '朝右'))
    elif abs(ang) > 150:
        print('判定: ❌ 相机装反了（差 180°）')
    else:
        print('判定: ⚠️ 偏差 %.0f°，需要校正安装角' % ang)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
