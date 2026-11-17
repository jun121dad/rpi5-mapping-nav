#!/usr/bin/env python3
"""诊断幽灵墙：统计近场深度点的高度/方位分布"""
import numpy as np, rclpy, tf2_ros, time, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

class D(Node):
    def __init__(self):
        super().__init__('ghost_diag')
        self.d = None; self.info = None; self.frame = None
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.on_d, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.on_i, qos_profile_sensor_data)
    def on_d(self, m):
        if self.d is None:
            self.d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)
            self.frame = m.header.frame_id; self.h, self.w = m.height, m.width
    def on_i(self, m): self.info = m

def q2R(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

rclpy.init(); n=D()
t0=time.time()
while (n.d is None or n.info is None) and time.time()-t0<15: rclpy.spin_once(n,timeout_sec=0.2)
tr=None; t0=time.time()
while tr is None and time.time()-t0<10:
    rclpy.spin_once(n,timeout_sec=0.2)
    try: tr=n.tfb.lookup_transform('base_link', n.frame, rclpy.time.Time())
    except Exception: pass
if n.d is None or tr is None:
    print("数据不全"); raise SystemExit

R=q2R(tr.transform.rotation); T=tr.transform.translation
K=n.info.k; fx,fy,cx,cy=K[0],K[4],K[2],K[5]
H,W=n.h,n.w
vs,us=np.mgrid[0:H:3,0:W:3]
z=n.d[vs,us].astype(np.float32)/1000.0
ok=(z>0.10)&(z<6.0)
us,vs,z=us[ok],vs[ok],z[ok]
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P=np.stack([X,Y,z],1)@R.T + np.array([T.x,T.y,T.z])
d=np.hypot(P[:,0],P[:,1]); hgt=P[:,2]
brg=np.degrees(np.arctan2(P[:,1],P[:,0]))
print("总点数 %d" % len(P))
print("\n=== 按距离分段：点数 / 高度中位 / 高度5-95分位 ===")
for lo,hi in [(0.2,0.4),(0.4,0.6),(0.6,0.8),(0.8,1.0),(1.0,1.5),(1.5,2.5),(2.5,6.0)]:
    m=(d>=lo)&(d<hi)
    if m.sum()<10: continue
    hh=hgt[m]
    print("  %.1f-%.1f m: %5d 点, 高度中位 %+.3f, 5%% %+.3f, 95%% %+.3f" % (
        lo,hi,m.sum(),np.median(hh),np.percentile(hh,5),np.percentile(hh,95)))
print("\n=== 障碍带 0.10~0.80 m 内的点 ===")
ob=(hgt>=0.10)&(hgt<=0.80)&(d<6.0)
print("  共 %d 点" % ob.sum())
if ob.sum():
    dd=d[ob]
    for lo,hi in [(0.2,0.5),(0.5,0.8),(0.8,1.2),(1.2,2.0),(2.0,6.0)]:
        m=(dd>=lo)&(dd<hi)
        if m.sum()<5: continue
        bb=brg[ob][m]
        print("    距离 %.1f-%.1f: %4d 点, 方位 %+.0f°~%+.0f° (中位 %+.0f°), 高度中位 %+.2f" % (
            lo,hi,m.sum(),bb.min(),bb.max(),np.median(bb),np.median(hgt[ob][m])))
print("\n=== 地面带 -0.04~0.10 m ===")
gr=(hgt>=-0.04)&(hgt<0.10)
print("  共 %d 点, 距离 0.2-2m 内 %d 点" % (gr.sum(), (gr&(d<2.0)).sum()))
n.destroy_node(); rclpy.shutdown()