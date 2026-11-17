#!/usr/bin/env python3
"""记录 map->odom 修正量 + 里程计位姿，用于诊断回头漂移"""
import csv, math, time, rclpy, tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry

OUT = '/home/pi/drift_log.csv'

class D(Node):
    def __init__(self):
        super().__init__('drift_logger')
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.f = open(OUT, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'ox', 'oy', 'oyaw_deg', 'corr_x', 'corr_y', 'corr_yaw_deg', 'total_yaw_deg'])
        self.t0 = time.time()
        self.create_timer(0.2, self.tick)   # 5 Hz
        self.get_logger().info('记录到 %s，5 Hz' % OUT)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z), 1-2*q.z*q.z))
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def tick(self):
        if self.odom is None:
            return
        try:
            tr = self.tfb.lookup_transform('map', 'odom', rclpy.time.Time())
        except Exception:
            return
        q = tr.transform.rotation
        cyaw = math.degrees(math.atan2(2*(q.w*q.z), 1-2*q.z*q.z))
        ox, oy, oyaw = self.odom
        self.w.writerow(['%.2f' % (time.time()-self.t0), '%.3f' % ox, '%.3f' % oy,
                         '%.2f' % oyaw, '%.3f' % tr.transform.translation.x,
                         '%.3f' % tr.transform.translation.y, '%.2f' % cyaw,
                         '%.2f' % (oyaw + cyaw)])
        self.f.flush()


def main():
    rclpy.init()
    n = D()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


if __name__ == '__main__':
    main()