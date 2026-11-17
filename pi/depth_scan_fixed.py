#!/usr/bin/env python3
"""深度图 → 2D 激光（正确考虑相机俯仰角）

比 depthimage_to_laserscan 好在：
  1. 用完整 TF 把点转到 base_link，相机俯仰/横滚都被正确补偿
  2. 只保留离地 [min_h, max_h] 的点 → 地板、天花板被滤掉
  3. 按角度分箱取最小值，输出稳定激光

参数：
  min_height  0.20   保留点的最低高度（m，相对 base_link）
  max_height  1.20   保留点的最高高度
  beams       640    输出光束数
  angle_min  -0.68   弧度
  angle_max   0.698
  range_min   0.45
  range_max   6.00
  row_step    2      深度图行采样步长
  col_step    2      列采样步长
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class DepthScan(Node):
    def __init__(self):
        super().__init__('depth_scan_fixed')
        self.declare_parameter('min_height', 0.20)
        self.declare_parameter('max_height', 1.20)
        self.declare_parameter('beams', 640)
        self.declare_parameter('angle_min', -0.68)
        self.declare_parameter('angle_max', 0.698)
        self.declare_parameter('range_min', 0.45)
        self.declare_parameter('range_max', 6.00)
        self.declare_parameter('ground_clear', False)
        self.declare_parameter('process_every', 2)      # 每 N 帧处理一次（降 CPU）
        self.declare_parameter('row_step', 2)
        self.declare_parameter('col_step', 2)
        self.declare_parameter('output_frame', 'base_link')

        self.min_h = self.get_parameter('min_height').value
        self.max_h = self.get_parameter('max_height').value
        self.nbeams = self.get_parameter('beams').value
        self.amin = self.get_parameter('angle_min').value
        self.amax = self.get_parameter('angle_max').value
        self.rmin = self.get_parameter('range_min').value
        self.rmax = self.get_parameter('range_max').value
        self.ground_clear = bool(self.get_parameter('ground_clear').value)
        self.process_every = max(1, int(self.get_parameter('process_every').value))
        self._frame_count = 0
        self.rstep = self.get_parameter('row_step').value
        self.cstep = self.get_parameter('col_step').value
        self.out_frame = self.get_parameter('output_frame').value

        self.info = None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info',
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw',
                                 self.on_img, qos_profile_sensor_data)
        self.get_logger().info('depth_scan_fixed 启动: 高度窗 [%.2f, %.2f] m, %d 束' % (
            self.min_h, self.max_h, self.nbeams))

    def on_info(self, m):
        self.info = m

    def on_img(self, m):
        self._frame_count += 1
        if self._frame_count % self.process_every != 0:
            return
        if self.info is None:
            return
        try:
            tr = self.tf.lookup_transform(self.out_frame, m.header.frame_id,
                                          rclpy.time.Time())
        except Exception:
            return
        t = tr.transform.translation
        q = tr.transform.rotation

        # 四元数 → 旋转矩阵
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

        depth = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]

        rows = np.arange(0, m.height, self.rstep)
        cols = np.arange(0, m.width, self.cstep)
        d = depth[np.ix_(rows, cols)].astype(np.float32) / 1000.0
        u, v = np.meshgrid(cols, rows)
        zz = d
        xx = (u - cx) * zz / fx
        yy = (v - cy) * zz / fy

        ok = (zz > 0.1) & (zz < self.rmax + 0.5) & np.isfinite(zz)
        if not ok.any():
            return
        px = xx[ok].ravel()
        py = yy[ok].ravel()
        pz = zz[ok].ravel()

        bx = R[0, 0] * px + R[0, 1] * py + R[0, 2] * pz + t.x
        by = R[1, 0] * px + R[1, 1] * py + R[1, 2] * pz + t.y
        bz = R[2, 0] * px + R[2, 1] * py + R[2, 2] * pz + t.z

        # 障碍点：离地 0.1~0.8 m（墙、家具）
        keep = (bz >= self.min_h) & (bz <= self.max_h)
        # 地面点：贴近地面（用于把空地标成空闲，否则房间中间永远是未知）
        gkeep = (bz >= -0.04) & (bz < self.min_h)

        out = np.full(self.nbeams, np.inf, dtype=np.float32)

        def project(mask):
            if not mask.any():
                return None, None
            px_, py_ = bx[mask], by[mask]
            r_ = np.hypot(px_, py_)
            a_ = np.arctan2(py_, px_)
            g = (r_ >= self.rmin) & (r_ <= self.rmax) & (a_ >= self.amin) & (a_ <= self.amax)
            if not g.any():
                return None, None
            return r_[g], a_[g]

        ro, ao = project(keep)
        if ro is not None:
            idx = ((ao - self.amin) / (self.amax - self.amin) * (self.nbeams - 1)).astype(np.int32)
            np.minimum.at(out, idx, ro)

        # 地面点仅用于"清除空闲"，绝不能当成测距值发布：
        # 任何有限的 range 都会被 slam 当作障碍端点（会把地面画成墙）。
        # 因此默认不发布地面波束（留 inf），空闲区由真实回波沿射线清除。
        if self.ground_clear:
            rg, ag = project(gkeep)
            if rg is not None:
                idx2 = ((ag - self.amin) / (self.amax - self.amin) * (self.nbeams - 1)).astype(np.int32)
                ground = np.full(self.nbeams, np.inf, dtype=np.float32)
                np.minimum.at(ground, idx2, rg)
                use = np.isfinite(ground) & ~np.isfinite(out)
                out[use] = ground[use]

        if not np.isfinite(out).any():
            return

        scan = LaserScan()
        # 用处理时刻的系统时间：点是按最新 TF 转换的，相机时间戳是设备时钟（与系统差几十年）
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.out_frame
        scan.angle_min = float(self.amin)
        scan.angle_max = float(self.amax)
        scan.angle_increment = float((self.amax - self.amin) / (self.nbeams - 1))
        scan.time_increment = 0.0
        scan.scan_time = 0.033
        scan.range_min = float(self.rmin)
        scan.range_max = float(self.rmax)
        scan.ranges = [float(v) if np.isfinite(v) else float('inf') for v in out]
        self.pub.publish(scan)


def main():
    rclpy.init()
    n = DepthScan()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
