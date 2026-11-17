#!/usr/bin/env python3
"""标定相机俯仰角：用地板当参照

原理：相机离地高度已知（默认 0.49 m）。对候选俯仰角 θ，把深度点转到 base_link：
      得分 = 落在 z≈0（地板）附近的点数
      得分最高的 θ 就是真实俯仰角。

用法：
  python3 /home/pi/calib_pitch.py            # 默认高度 0.49 m
  python3 /home/pi/calib_pitch.py 0.52       # 指定高度
前提：车放在平整地面，相机视野下半部分能看到地板。
"""
import math
import sys

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

HEIGHT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.49
X_OFF = 0.08


class Calib(Node):
    def __init__(self):
        super().__init__('calib_pitch')
        self.depth = None
        self.info = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info',
                                 self.on_info, qos_profile_sensor_data)

    def on_depth(self, m):
        if self.depth is None:
            try:
                self.depth = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
                self.h, self.w = m.height, m.width
                self.frame = m.header.frame_id
            except Exception:                              # noqa: BLE001
                self.depth = None

    def on_info(self, m):
        self.info = m


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    n = Calib()
    print('等待深度图...')
    import time
    t0 = time.time()
    while (n.depth is None or n.info is None) and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.depth is None or n.info is None:
        print('✗ 没有深度图或 camera_info')
        return 1

    # 取 base_link ← 光学系 的旋转（当前 TF 无俯仰）
    tr = None
    t0 = time.time()
    while tr is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tr = n.tf.lookup_transform('base_link', n.frame, rclpy.time.Time())
        except Exception:
            pass
    if tr is None:
        print('✗ TF 不可用')
        return 2
    R_opt = quat_to_R(tr.transform.rotation)

    K = n.info.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    H, W = n.h, n.w
    # 取画面下半部分、每 6 像素一个采样点
    vs, us = np.mgrid[int(H * 0.45):H:6, 0:W:6]
    z = n.depth[vs, us].astype(np.float32) / 1000.0
    ok = (z > 0.15) & (z < 5.0)
    us, vs, z = us[ok], vs[ok], z[ok]
    if z.size < 500:
        print('✗ 地板可见点太少（%d），请把车放平、镜头朝下一点' % z.size)
        return 3
    Xc = (us - cx) * z / fx
    Yc = (vs - cy) * z / fy
    P = np.stack([Xc, Yc, z], axis=1) @ R_opt.T      # → camera_link 系

    print('采样点 %d 个，开始搜索俯仰角...' % z.size)
    best = None
    for deg in np.arange(-40.0, 20.01, 0.5):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        Q = P @ Ry.T
        Q[:, 2] += HEIGHT
        Q[:, 0] += X_OFF
        score = int(np.sum(np.abs(Q[:, 2]) < 0.02))
        if best is None or score > best[1]:
            best = (deg, score)
    deg, score = best
    print('')
    print('=== 结果 ===')
    print('最佳俯仰角: %+.1f°（向下为正）' % deg)
    print('地板点命中: %d / %d = %.1f%%' % (score, z.size, 100.0 * score / z.size))
    q = (math.sin(math.radians(deg) / 2.0), math.cos(math.radians(deg) / 2.0))
    print('')
    print('对应静态 TF 参数：')
    print('  --x %.3f --y 0.000 --z %.3f --qx 0 --qy %.6f --qz 0 --qw %.6f' % (
        X_OFF, HEIGHT, q[0], q[1]))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
