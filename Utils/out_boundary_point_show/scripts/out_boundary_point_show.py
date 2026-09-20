#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 订阅 /cloud_registered，按探索长方体边界框着色后转发：
#   - 框外的点 -> 红色
#   - 框内的点 -> viridis 色族，按 y 坐标（在 box y 范围内归一化）变化
# 输出话题默认 /cloud_registered_colored，RViz 里 PointCloud2 的
# Color Transformer 选 RGB8 即可看到颜色。
#
# 参数（均有默认值，默认框 = tmp_at_feiji_rigo.yaml 的 box_0）：
#   ~cloud_topic  输入点云话题        (默认 /cloud_registered)
#   ~out_topic    输出点云话题        (默认 /cloud_registered_colored)
#   ~box_min      [x,y,z] 框下界      (默认 [-1.0, -10.0, -0.2])
#   ~box_max      [x,y,z] 框上界      (默认 [52.0, 75.0, 2.0])
#   ~draw_box     是否发布框线 Marker (默认 true)

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker

# matplotlib viridis 的控制点采样（避免依赖 matplotlib），线性插值
_VIRIDIS = np.array([
    [0.267004, 0.004874, 0.329415],
    [0.282623, 0.140926, 0.457517],
    [0.253935, 0.265254, 0.529983],
    [0.206756, 0.371758, 0.553117],
    [0.163625, 0.471133, 0.558148],
    [0.127568, 0.566949, 0.550556],
    [0.134692, 0.658636, 0.517649],
    [0.266941, 0.748751, 0.440573],
    [0.477504, 0.821444, 0.318195],
    [0.741388, 0.873449, 0.149561],
    [0.993248, 0.906157, 0.143936],
])

_FIELDS = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('rgb', 12, PointField.FLOAT32, 1),
]


def viridis(t):
    """t: 0~1 数组 -> (N,3) float32 RGB(0~255)"""
    t = np.clip(t, 0.0, 1.0) * (len(_VIRIDIS) - 1)
    i = np.minimum(t.astype(np.int32), len(_VIRIDIS) - 2)
    f = (t - i)[:, None]
    rgb = _VIRIDIS[i] * (1.0 - f) + _VIRIDIS[i + 1] * f
    return (rgb * 255.0).round().astype(np.uint32)


class BoundaryPointShow(object):
    def __init__(self):
        self.box_min = np.array(
            rospy.get_param('~box_min', [-1.0, -10.0, -0.2]), dtype=np.float64)
        self.box_max = np.array(
            rospy.get_param('~box_max', [52.0, 75.0, 2.0]), dtype=np.float64)
        cloud_topic = rospy.get_param('~cloud_topic', '/cloud_registered')
        out_topic = rospy.get_param('~out_topic', '/cloud_registered_colored')
        self.draw_box = rospy.get_param('~draw_box', True)

        self.pub = rospy.Publisher(out_topic, PointCloud2, queue_size=2)
        if self.draw_box:
            self.box_pub = rospy.Publisher('~box_marker', Marker, queue_size=1,
                                           latch=True)
        self.sub = rospy.Subscriber(cloud_topic, PointCloud2, self.cb,
                                    queue_size=2)
        rospy.loginfo('out_boundary_point_show: %s -> %s, box %s ~ %s',
                      cloud_topic, out_topic, self.box_min.tolist(),
                      self.box_max.tolist())

    def publish_box(self, header):
        m = Marker()
        m.header = header
        m.ns = 'exploration_box'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.9
        lo, hi = self.box_min, self.box_max
        corners = np.array([[lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
                            [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
                            [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
                            [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]]])
        edges = [(0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]
        for a, b in edges:
            for c in (a, b):
                from geometry_msgs.msg import Point
                p = Point()
                p.x, p.y, p.z = corners[c]
                m.points.append(p)
        self.box_pub.publish(m)

    def cb(self, msg):
        pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)],
            dtype=np.float32)
        if pts.shape[0] == 0:
            return

        inside = np.all((pts >= self.box_min) & (pts <= self.box_max), axis=1)

        # 框内：y 在 box y 范围内归一化 -> viridis；框外：红色
        rgb_u32 = np.empty(pts.shape[0], dtype=np.uint32)
        rgb_u32[~inside] = (255 << 16)  # 红
        if inside.any():
            y_span = max(self.box_max[1] - self.box_min[1], 1e-6)
            t = (pts[inside, 1] - self.box_min[1]) / y_span
            c = viridis(t)
            rgb_u32[inside] = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]

        out = np.column_stack([pts, rgb_u32.view(np.float32)])
        header = msg.header  # 保留 frame_id / 时间戳，rviz 直接可对齐 tf
        self.pub.publish(point_cloud2.create_cloud(header, _FIELDS, out))
        if self.draw_box:
            self.publish_box(header)


if __name__ == '__main__':
    rospy.init_node('out_boundary_point_show')
    BoundaryPointShow()
    rospy.spin()
