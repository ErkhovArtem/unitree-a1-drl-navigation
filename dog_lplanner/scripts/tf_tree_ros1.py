#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Минимальный TF-lookup для ROS1 без tf2_ros/tf2_py.

Причина: в ROS Melodic tf2_py собран под Python2, а инференс-скрипт у нас на Python3
(из-за onnxruntime). Поэтому используем только сообщения /tf и чистый python.

Поддерживает:
- Подписку на /tf и /tf_static (если есть)
- Поиск преобразования между фреймами (BFS по графу)
- Преобразование PointStamped (geometry_msgs/PointStamped) между фреймами

Внимание: временные метки игнорируются (берём последние известные трансформы).
"""

from collections import defaultdict, deque
from typing import Dict, Optional, Set, Tuple

import rospy
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage


class _Transform(object):
    """Transform A->B: p_B = R(q)*p_A + t. Compatible with Python 3.6."""

    __slots__ = ("t", "q")

    def __init__(self, t, q):
        self.t = t  # (x,y,z)
        self.q = q  # (x,y,z,w)


def _quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_inv(q):
    x, y, z, w = q
    n2 = x * x + y * y + z * z + w * w
    if n2 <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / n2, -y / n2, -z / n2, w / n2)


def _quat_rotate(q, v):
    """Rotate vector v by quaternion q (x,y,z,w)."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    # t = 2 * cross(q_vec, v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    # v' = v + qw * t + cross(q_vec, t)
    vpx = vx + qw * tx + (qy * tz - qz * ty)
    vpy = vy + qw * ty + (qz * tx - qx * tz)
    vpz = vz + qw * tz + (qx * ty - qy * tx)
    return (vpx, vpy, vpz)


def _compose(a_to_b: _Transform, b_to_c: _Transform) -> _Transform:
    """Compose A->B then B->C = A->C."""
    t_ab, q_ab = a_to_b.t, a_to_b.q
    t_bc, q_bc = b_to_c.t, b_to_c.q
    q_ac = _quat_mul(q_bc, q_ab)
    t_ab_rot = _quat_rotate(q_bc, t_ab)
    t_ac = (t_ab_rot[0] + t_bc[0], t_ab_rot[1] + t_bc[1], t_ab_rot[2] + t_bc[2])
    return _Transform(t=t_ac, q=q_ac)


def _invert(a_to_b: _Transform) -> _Transform:
    q_inv = _quat_inv(a_to_b.q)
    t = a_to_b.t
    t_inv = _quat_rotate(q_inv, (-t[0], -t[1], -t[2]))
    return _Transform(t=t_inv, q=q_inv)


class TFGraph:
    def __init__(self):
        self._tf: Dict[Tuple[str, str], _Transform] = {}
        self._adj: Dict[str, Set[str]] = defaultdict(set)

        self._sub_tf = rospy.Subscriber("/tf", TFMessage, self._tf_cb, queue_size=20)
        # /tf_static может отсутствовать — подписка не мешает
        self._sub_tf_static = rospy.Subscriber("/tf_static", TFMessage, self._tf_cb, queue_size=5)

    def _tf_cb(self, msg: TFMessage):
        for ts in msg.transforms:
            parent = ts.header.frame_id.strip("/")
            child = ts.child_frame_id.strip("/")
            if not parent or not child:
                continue
            t = (float(ts.transform.translation.x), float(ts.transform.translation.y), float(ts.transform.translation.z))
            q = (
                float(ts.transform.rotation.x),
                float(ts.transform.rotation.y),
                float(ts.transform.rotation.z),
                float(ts.transform.rotation.w),
            )
            self._tf[(parent, child)] = _Transform(t=t, q=q)
            self._adj[parent].add(child)
            self._adj[child].add(parent)

    def _edge_transform(self, a: str, b: str) -> Optional[_Transform]:
        """Return transform A->B if known, using inversion if only B->A is stored."""
        key = (a, b)
        if key in self._tf:
            return self._tf[key]
        rev = (b, a)
        if rev in self._tf:
            return _invert(self._tf[rev])
        return None

    def lookup(self, target_frame: str, source_frame: str) -> Optional[_Transform]:
        """
        Return transform Source->Target (p_target = R*p_source + t).
        """
        src = source_frame.strip("/")
        tgt = target_frame.strip("/")
        if src == tgt:
            return _Transform(t=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))

        # BFS in undirected graph, then compose transforms along the path
        q = deque([src])
        came_from: Dict[str, Optional[str]] = {src: None}
        while q:
            cur = q.popleft()
            if cur == tgt:
                break
            for nb in self._adj.get(cur, set()):
                if nb in came_from:
                    continue
                if self._edge_transform(cur, nb) is None:
                    continue
                came_from[nb] = cur
                q.append(nb)

        if tgt not in came_from:
            return None

        # reconstruct path src -> ... -> tgt
        path = []
        cur = tgt
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()

        # compose transforms along path
        total = _Transform(t=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))  # src->src
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            a_to_b = self._edge_transform(a, b)
            if a_to_b is None:
                return None
            total = _compose(total, a_to_b)  # src->a then a->b => src->b

        # total currently is src->tgt (as desired)
        return total

    def transform_point(self, target_frame: str, point: PointStamped) -> Optional[PointStamped]:
        src = point.header.frame_id.strip("/")
        tgt = target_frame.strip("/")
        tfm = self.lookup(tgt, src)
        if tfm is None:
            return None

        p = (float(point.point.x), float(point.point.y), float(point.point.z))
        p_rot = _quat_rotate(tfm.q, p)
        p_out = (p_rot[0] + tfm.t[0], p_rot[1] + tfm.t[1], p_rot[2] + tfm.t[2])

        out = PointStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = tgt
        out.point.x = p_out[0]
        out.point.y = p_out[1]
        out.point.z = p_out[2]
        return out

