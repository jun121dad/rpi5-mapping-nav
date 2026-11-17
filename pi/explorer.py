#!/usr/bin/env python3
"""前沿探索 v2：信息增益 + 可达性 + 黑名单

相比 v1（只按最近距离挑）的改进：
  1. 信息增益：优先选"周围未知区域大"的前沿，而不是只挑最近的
  2. 可达性：用连通域判断前沿是否与车位在同一片自由空间，排除墙后的前沿
  3. 前沿聚类：把相邻的前沿格聚成一簇，取质心，避免选到噪声点
  4. 黑名单：导航失败的点以更大半径排除，避免反复尝试同一处
  5. 朝向：目标朝向未知区域一侧，让窄视野相机看着新区域

开关：/explore/enable (std_msgs/Bool)
状态：/explore/status (String JSON)
"""
import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import Bool, Float32, String

FREE_MAX = 50


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.declare_parameter('enable_on_start', False)
        self.declare_parameter('min_goal_dist', 0.8)
        self.declare_parameter('max_goal_dist', 3.5)
        self.declare_parameter('clearance_min', 0.35)
        self.declare_parameter('visited_radius', 1.0)
        self.declare_parameter('blacklist_radius', 1.6)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('min_cluster_cells', 3)
        self.declare_parameter('gain_radius_m', 1.0)
        self.declare_parameter('max_blacklist', 60)
        self.declare_parameter('duration_min', 10.0)     # 探索时长（分钟，0=无限）

        p = self.get_parameter
        self.enabled = bool(p('enable_on_start').value)
        self.dmin = float(p('min_goal_dist').value)
        self.dmax = float(p('max_goal_dist').value)
        self.clear_min = float(p('clearance_min').value)
        self.visited_r = float(p('visited_radius').value)
        self.black_r = float(p('blacklist_radius').value)
        self.goal_timeout = float(p('goal_timeout').value)
        self.min_cells = int(p('min_cluster_cells').value)
        self.gain_r = float(p('gain_radius_m').value)
        self.max_black = int(p('max_blacklist').value)
        self.duration_min = float(p('duration_min').value)
        self.t_enable = 0.0

        self.map = None
        self.visited = []      # 成功到达的点
        self.blacklist = []    # 失败的点
        self.goal_xy = None
        self.goal_active = False
        self.t_goal = 0.0
        self.goal_handle = None
        self.msg = '待命'
        self.n_sent = 0
        self.n_fail = 0
        self.last_score = 0.0

        q_lat = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q_lat)
        self.create_subscription(Bool, '/explore/enable', self.on_enable, 10)
        self.create_subscription(Float32, '/explore/duration', self.on_duration, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub = self.create_publisher(String, '/explore/status', 10)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_timer(1.0, self.tick)
        self.get_logger().info('探索 v2 启动（信息增益 + 可达性）（%s）'
                               % ('启用' if self.enabled else '待命'))

    # ---------------------------------------------------------- 回调
    def on_map(self, m):
        self.map = m

    def on_enable(self, m):
        want = bool(m.data)
        if want == self.enabled:
            return
        self.enabled = want
        if not want:
            self._cancel()
            self.msg = '已停止'
        else:
            self.t_enable = time.time()
            self.msg = '开始探索（%.0f 分钟）' % self.duration_min if self.duration_min > 0 else '开始探索（不限时）'
        self.get_logger().info('探索 %s' % ('启用' if want else '停止'))

    def on_duration(self, m):
        try:
            self.duration_min = float(m.data)
            if self.enabled:
                self.t_enable = time.time()      # 运行中改时长，从现在重新计时
            self.get_logger().info('探索时长设为 %.1f 分钟（0=无限）' % self.duration_min)
        except Exception:
            pass
    def _cancel(self):
        try:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        except Exception:                                  # noqa: BLE001
            pass
        self.goal_handle = None
        self.goal_active = False
        self.goal_xy = None

    def _pose(self):
        try:
            tr = self.tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
            return tr.transform.translation.x, tr.transform.translation.y
        except Exception:                                  # noqa: BLE001
            return None

    # ---------------------------------------------------------- 核心：选前沿
    def _pick_frontier(self):
        m = self.map
        if m is None or m.info.width < 10:
            return None, '地图还没建起来'
        w, h, res = m.info.width, m.info.height, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)

        free = (grid >= 0) & (grid < FREE_MAX)
        unk = grid < 0
        if not free.any():
            return None, '还没有空闲区域'

        pose = self._pose()
        if pose is None:
            return None, '没有位姿'
        rx, ry = pose

        # ① 可达性：自由空间的连通域，取车所在那一片
        lbl, n = ndimage.label(free, structure=np.ones((3, 3), dtype=int))
        if n == 0:
            return None, '自由空间不连通'
        ci = int((rx - ox) / res)
        cj = int((ry - oy) / res)
        ci = max(0, min(w - 1, ci))
        cj = max(0, min(h - 1, cj))
        lab = lbl[cj, ci]
        if lab == 0:                     # 车不在自由格上（定位抖动），取最近自由格
            ys, xs = np.nonzero(free)
            if len(xs) == 0:
                return None, '车不在自由空间'
            k = int(np.argmin((xs - ci) ** 2 + (ys - cj) ** 2))
            lab = lbl[ys[k], xs[k]]
        reachable = (lbl == lab)

        # ② 前沿 = 自由且 8 邻域有未知
        unk_nb = np.zeros_like(unk)
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                unk_nb |= np.roll(np.roll(unk, dj, axis=0), di, axis=1)
        frontier = reachable & free & unk_nb
        if not frontier.any():
            return None, '没有前沿（可能已探索完）'

        # ③ 聚类，取每个簇的质心与大小
        flbl, fn = ndimage.label(frontier, structure=np.ones((3, 3), dtype=int))
        if fn == 0:
            return None, '没有前沿簇'
        idx = np.arange(1, fn + 1)
        sizes = ndimage.sum(frontier, flbl, idx)
        cy = ndimage.center_of_mass(frontier, flbl, idx)
        keep = sizes >= self.min_cells
        if not keep.any():
            return None, '前沿簇都太小'

        # ④ 信息增益：每个前沿周围未知格数
        r_cells = max(2, int(self.gain_r / res))
        unk_f = unk.astype(np.float32)
        # 用积分图快速求窗口和
        integ = np.cumsum(np.cumsum(unk_f, axis=0), axis=1)
        integ = np.pad(integ, ((1, 0), (1, 0)))

        def win_sum(cj_, ci_, r):
            y0 = max(0, cj_ - r); y1 = min(h, cj_ + r + 1)
            x0 = max(0, ci_ - r); x1 = min(w, ci_ + r + 1)
            return float(integ[y1, x1] - integ[y0, x1] - integ[y1, x0] + integ[y0, x0])

        # ④b 净空距离场：每个自由格到最近障碍的距离（米）
        occ = (grid >= FREE_MAX) | (grid < 0)
        dist_field = ndimage.distance_transform_edt(~occ) * res

        # ④b 净空距离场：每个自由格到最近障碍的距离（米）
        occ = (grid >= FREE_MAX) | (grid < 0)
        dist_field = ndimage.distance_transform_edt(~occ) * res

        cands = []
        for k in range(fn):
            if not keep[k]:
                continue
            ys_k, xs_k = np.nonzero(flbl == (k + 1))
            if len(xs_k) == 0:
                continue
            clr = dist_field[ys_k, xs_k]
            j = int(np.argmax(clr))
            fj, fi = int(ys_k[j]), int(xs_k[j])
            clearance = float(clr[j])
            fx = ox + (fi + 0.5) * res
            fy = oy + (fj + 0.5) * res
            d = math.hypot(fx - rx, fy - ry)
            if d < self.dmin or d > self.dmax:
                continue
            if any(math.hypot(fx - vx, fy - vy) < self.visited_r for vx, vy in self.visited):
                continue
            if any(math.hypot(fx - bx, fy - by) < self.black_r for bx, by in self.blacklist):
                continue
            gain = win_sum(fj, fi, r_cells)
            score = gain / (d + 1.0) * min(1.0, clearance / 0.6)
            cands.append((score, fx, fy, d, gain, clearance, int(sizes[k])))

        if not cands:
            return None, '没有候选前沿（距离/黑名单过滤后为空）'

        # 净空要求逐级放宽，避免地图小时死锁
        for thr in (self.clear_min, 0.25, 0.15, 0.0):
            sel = [c for c in cands if c[5] >= thr]
            if sel:
                best = max(sel, key=lambda c: c[0])
                self.last_score = best[0]
                return best, ''
        return None, '候选前沿都被过滤掉了'
    # ---------------------------------------------------------- 发目标
    def _goto(self, x, y):
        if not self.nav.server_is_ready():
            self.msg = 'Nav2 未就绪'
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        pose = self._pose()
        yaw = math.atan2(y - pose[1], x - pose[0]) if pose else math.atan2(y, x)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.goal_xy = (x, y)
        self.goal_active = True
        self.t_goal = time.time()
        self.n_sent += 1
        self.nav.send_goal_async(goal).add_done_callback(self._on_resp)
        return True

    def _on_resp(self, fut):
        try:
            gh = fut.result()
            if gh and gh.accepted:
                self.goal_handle = gh
                gh.get_result_async().add_done_callback(self._on_result)
            else:
                self.msg = '目标被拒绝'
                self._mark_fail()
        except Exception:                                  # noqa: BLE001
            self._mark_fail()

    def _on_result(self, fut):
        try:
            st = fut.result().status
        except Exception:                                  # noqa: BLE001
            st = 0
        # 探索已停止：忽略被取消目标的回调（status=5/CANCELED 不是真失败）
        if not self.enabled:
            self.goal_xy = None
            self.goal_active = False
            self.goal_handle = None
            return
        if st == 4:
            self.msg = '到达前沿'
            if self.goal_xy:
                self.visited.append(self.goal_xy)
        else:
            self.n_fail += 1
            self.msg = '导航失败(status=%s)' % st
            self._mark_fail()
            return
        self.goal_xy = None
        self.goal_active = False
        self.goal_handle = None

    def _mark_fail(self):
        if self.goal_xy is not None:
            self.blacklist.append(self.goal_xy)
            if len(self.blacklist) > self.max_black:
                self.blacklist.pop(0)
        self.goal_xy = None
        self.goal_active = False
        self.goal_handle = None

    # ---------------------------------------------------------- 主循环
    def tick(self):
        if not self.enabled:
            self._publish()
            return
        now = time.time()
        # 自定义时长到点自动结束
        if self.duration_min > 0 and (now - self.t_enable) >= self.duration_min * 60.0:
            self._cancel()
            self.enabled = False
            self.msg = '探索结束（已达到 %.0f 分钟）' % self.duration_min
            self.get_logger().info(self.msg)
            self._publish()
            return
        if self.goal_active:
            if now - self.t_goal > self.goal_timeout:
                self.msg = '目标超时，换一个'
                self._cancel()
                self.n_fail += 1
                self._mark_fail()
            self._publish()
            return
        best, reason = self._pick_frontier()
        if best is None:
            self.msg = reason
        else:
            score, fx, fy, d, gain, clearance, size = best
            self._goto(fx, fy)
            self.msg = '前往 (%.2f, %.2f) 距离%.1fm 增益%d格 得分%.1f' % (
                fx, fy, d, gain, score)
        self._publish()

    def _publish(self):
        s = String()
        s.data = json.dumps({
            'enabled': self.enabled,
            'goal': self.goal_xy,
            'visited': len(self.visited),
            'black': len(self.blacklist),
            'sent': self.n_sent,
            'fail': self.n_fail,
            'msg': self.msg,
            'duration_min': self.duration_min,
            'elapsed_min': round((time.time() - self.t_enable) / 60.0, 2) if self.enabled else 0.0,
        }, ensure_ascii=False)
        self.pub.publish(s)


def main():
    rclpy.init()
    n = Explorer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n._cancel()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
