#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WHEELTEC Mini 底盘 <-> ROS 2 桥接节点。

上行（车 -> Pi）：24 字节状态帧，20 Hz
    [0]0x7B [1]Flag_Stop [2:4]Vx [4:6]Vy [6:8]Vz
    [8:10]Ax [10:12]Ay [12:14]Az [14:16]Gx [16:18]Gy [18:20]Gz
    [20:22]电压mV [22]XOR(0..21) [23]0x7D
    速度单位 mm/s，角速度单位 0.001 rad/s，加速度/角速度是 MPU6050 原始值

下行（Pi -> 车）：11 字节控制帧
    [0]0x7B [1:3]保留 [3:5]Vx [5:7]Vy [7:9]Vz [9]XOR(0..8) [10]0x7D
    速度单位 mm/s，角速度单位 0.001 rad/s
"""

import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16MultiArray

try:
    import serial
except ImportError:      # 允许无串口环境下 import 成功，运行时再报错
    serial = None

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
CMD_LEN = 11
FB_LEN = 24

INT16_MAX = 32767
INT16_MIN = -32768


def xor_checksum(buf, n):
    c = 0
    for i in range(n):
        c ^= buf[i]
    return c


def clamp_i16(v):
    if v > INT16_MAX:
        return INT16_MAX
    if v < INT16_MIN:
        return INT16_MIN
    return int(v)


class WheeltecBridge(Node):
    def __init__(self):
        super().__init__('wheeltec_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_linear', 0.6)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('imu_weight', 1.0)
        self.declare_parameter('gyro_scale', 0.0002663)
        self.declare_parameter('wheel_scale', 1.003)
        self.declare_parameter('imu_sign', 1.0)
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.imu_w = float(self.get_parameter('imu_weight').value)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)
        self.wheel_scale = float(self.get_parameter('wheel_scale').value)
        self.imu_sign = float(self.get_parameter('imu_sign').value)
        self.gyro_bias = 0.0
        self.bias_sum = 0.0
        self.bias_n = 0
        self.bias_ready = False
        self.gyro_z = 0.0
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_linear_y', False)
        self.declare_parameter('invert_angular_z', False)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_v = float(self.get_parameter('max_linear').value)
        self.max_w = float(self.get_parameter('max_angular').value)
        self.inv_x = bool(self.get_parameter('invert_linear_x').value)
        self.inv_y = bool(self.get_parameter('invert_linear_y').value)
        self.inv_z = bool(self.get_parameter('invert_angular_z').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.ser = None
        self.rx_buf = bytearray()
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_stamp = 0.0
        self.last_open_try = 0.0
        self.last_fb_time = 0.0
        self.frames = 0

        # 里程计积分状态
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.last_odom_time = self.get_clock().now()

        self.pub_vel = self.create_publisher(Twist, 'wheeltec/velocity', 10)
        self.pub_volt = self.create_publisher(Float32, 'wheeltec/voltage', 10)
        self.pub_enabled = self.create_publisher(Bool, 'wheeltec/enabled', 10)
        self.pub_imu_raw = self.create_publisher(Int16MultiArray, 'wheeltec/imu_raw', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = None
        if self.publish_odom:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        self._open_serial()
        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            'wheeltec_bridge 启动: %s @ %d, %.1f Hz, 限幅 %.2f m/s / %.2f rad/s'
            % (self.port, self.baud, self.rate, self.max_v, self.max_w))

    # ------------------------------------------------------------------ 串口
    def _open_serial(self):
        if serial is None:
            self.get_logger().error('未安装 pyserial，请执行: sudo apt install python3-serial')
            return
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baud, bytesize=8,
                parity='N', stopbits=1, timeout=0, write_timeout=0.1)
            self.get_logger().info('串口已打开: %s' % self.port)
        except Exception as exc:                       # noqa: BLE001
            self.ser = None
            self.get_logger().error('串口打开失败 (%s): %s' % (self.port, exc))

    def _ensure_serial(self, now):
        if self.ser is not None:
            return
        if now - self.last_open_try < 1.0:
            return
        self.last_open_try = now
        self._open_serial()

    # ------------------------------------------------------------------ 回调
    def cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x * (-1.0 if self.inv_x else 1.0)
        vy = msg.linear.y * (-1.0 if self.inv_y else 1.0)
        wz = msg.angular.z * (-1.0 if self.inv_z else 1.0)

        if not math.isfinite(vx) or not math.isfinite(vy) or not math.isfinite(wz):
            return

        self.cmd = (
            max(-self.max_v, min(self.max_v, vx)),
            max(-self.max_v, min(self.max_v, vy)),
            max(-self.max_w, min(self.max_w, wz)),
        )
        self.cmd_stamp = self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ 主循环
    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self._ensure_serial(now)
        if self.ser is None:
            return

        self._read_serial()
        self._send_cmd(now)

    def _read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf.extend(self.ser.read(n))
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warning('串口读失败: %s' % exc)
            self.ser = None
            return

        while len(self.rx_buf) >= FB_LEN:
            if self.rx_buf[0] != FRAME_HEADER:
                idx = self.rx_buf.find(FRAME_HEADER)
                if idx < 0:
                    self.rx_buf.clear()
                    return
                del self.rx_buf[:idx]
                if len(self.rx_buf) < FB_LEN:
                    return

            frame = self.rx_buf[:FB_LEN]
            if frame[FB_LEN - 1] != FRAME_TAIL or xor_checksum(frame, FB_LEN - 2) != frame[FB_LEN - 2]:
                del self.rx_buf[0]
                continue

            del self.rx_buf[:FB_LEN]
            self._publish_feedback(frame)

    def _publish_feedback(self, f):
        stopped = bool(f[1])
        vx, vy, wz = struct.unpack_from('>3h', f, 2)
        ax, ay, az, gx, gy, gz = struct.unpack_from('>6h', f, 8)
        volt_mv = struct.unpack_from('>h', f, 20)[0]

        vx_ms = vx / 1000.0 * self.wheel_scale
        vy_ms = vy / 1000.0 * self.wheel_scale
        wz_rad = wz / 1000.0

        gz_raw = float(gz)
        if not self.bias_ready:
            if abs(wz_rad) < 0.05:
                self.bias_sum += gz_raw
                self.bias_n += 1
                if self.bias_n >= 60:
                    self.gyro_bias = self.bias_sum / self.bias_n
                    self.bias_ready = True
                    self.get_logger().info('陀螺零偏标定: %.1f counts' % self.gyro_bias)
        elif abs(wz_rad) < 0.02:
            self.gyro_bias = 0.995 * self.gyro_bias + 0.005 * gz_raw
        gz_cal = (gz_raw - self.gyro_bias) * self.gyro_scale * self.imu_sign
        # 钳制：超过 3 rad/s 视为噪声（滑移转向正常不会这么快）
        self.gyro_z = max(-3.0, min(3.0, gz_cal))

        tw = Twist()
        tw.linear.x = vx_ms
        tw.linear.y = vy_ms
        tw.angular.z = wz_rad
        self.pub_vel.publish(tw)

        self.pub_volt.publish(Float32(data=volt_mv / 1000.0))
        self.pub_enabled.publish(Bool(data=not stopped))
        self.pub_imu_raw.publish(Int16MultiArray(data=[ax, ay, az, gx, gy, gz]))

        self.frames += 1
        now = self.get_clock().now()
        self.last_fb_time = now.nanoseconds / 1e9

        if self.publish_odom:
            self._integrate_odom(now, vx_ms, vy_ms, wz_rad)

    def _integrate_odom(self, now, vx, vy, wz):
        dt_raw = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt_raw <= 0.0:
            return
        # 标称周期，抖动时钉回标称值；丢帧时用实际间隔补偿（不再丢弃这段位移）
        nominal = 1.0 / float(self.rate_hz) if hasattr(self, 'rate_hz') else 0.05
        if 0.6 * nominal <= dt_raw <= 1.4 * nominal:
            dt = nominal
        elif dt_raw < 0.6 * nominal:
            dt = 0.6 * nominal
        else:
            dt = min(dt_raw, 0.5)
            self.dropped_frames = getattr(self, 'dropped_frames', 0) + 1
            if self.dropped_frames % 50 == 1:
                self.get_logger().warn('里程计丢帧 %d 次，本次间隔 %.3fs 已用速度补偿' % (self.dropped_frames, dt_raw))

        wz_use = wz
        if self.use_imu and self.bias_ready:
            wz_use = self.imu_w * self.gyro_z + (1.0 - self.imu_w) * wz
        mid_th = self.odom_th + wz_use * dt * 0.5
        self.odom_x += (vx * math.cos(mid_th) - vy * math.sin(mid_th)) * dt
        self.odom_y += (vx * math.sin(mid_th) + vy * math.cos(mid_th)) * dt
        self.odom_th += wz_use * dt
        self.odom_th = math.atan2(math.sin(self.odom_th), math.cos(self.odom_th))

        qz = math.sin(self.odom_th * 0.5)
        qw = math.cos(self.odom_th * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz_use
        # 协方差：平移不确定性大（滑移/比例误差），旋转小（陀螺 ±0.2°）
        # 让 slam 的扫描匹配敢于纠正位置，同时信任航向
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.005     # x, y  σ≈4.5 cm（实测误差 5.8%）
        cov[14] = 0.005             # z
        cov[21] = cov[28] = 0.01    # roll, pitch
        cov[35] = 0.0002            # yaw   σ≈0.8°（陀螺实测 0.075°/min，可信）
        odom.pose.covariance = cov
        tw_cov = [0.0] * 36
        tw_cov[0] = tw_cov[7] = 0.01
        tw_cov[35] = 0.001
        odom.twist.covariance = tw_cov
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.odom_x
            tf.transform.translation.y = self.odom_y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)

    def _send_cmd(self, now):
        age = now - self.cmd_stamp
        if age > self.cmd_timeout:
            vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = self.cmd

        buf = bytearray(CMD_LEN)
        buf[0] = FRAME_HEADER
        buf[1] = 0x00
        buf[2] = 0x00
        struct.pack_into('>h', buf, 3, clamp_i16(round(vx * 1000.0)))
        struct.pack_into('>h', buf, 5, clamp_i16(round(vy * 1000.0)))
        struct.pack_into('>h', buf, 7, clamp_i16(round(wz * 1000.0)))
        buf[9] = xor_checksum(buf, 9)
        buf[10] = FRAME_TAIL

        try:
            self.ser.write(buf)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warning('串口写失败: %s' % exc)
            self.ser = None

    # ------------------------------------------------------------------ 退出
    def stop_car(self):
        if self.ser is None:
            return
        buf = bytearray(CMD_LEN)
        buf[0] = FRAME_HEADER
        buf[9] = xor_checksum(buf, 9)
        buf[10] = FRAME_TAIL
        try:
            for _ in range(3):
                self.ser.write(buf)
        except Exception:                              # noqa: BLE001
            pass

    def destroy_node(self):
        self.stop_car()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:                          # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheeltecBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
