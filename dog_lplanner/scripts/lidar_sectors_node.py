#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 node: LaserScan → sector distances (policy-compatible layout).

Subscribes: ~scan_topic (LaserScan).
Publishes: ~sectors_topic (Float32MultiArray, meters); optional MarkerArray on ~lidar_viz_topic.
"""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import rospy
import yaml
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker, MarkerArray

from lidar_processor_ros1 import LidarProcessorROS1


def _load_yaml(path: str) -> Dict[str, Any]:
    if not path:
        return {}
    p = Path(path)
    if not p.exists():
        rospy.logwarn("Config file not found: %s", str(p))
        return {}
    try:
        with p.open("r") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            rospy.logwarn("Config is not a dict: %s", str(p))
            return {}
        return data
    except Exception as e:
        rospy.logwarn("Failed to read config %s: %s", str(p), str(e))
        return {}


class LidarSectorsNode:
    def __init__(self) -> None:
        rospy.init_node("lidar_sectors", anonymous=True)

        self.scan_topic: str = rospy.get_param("~scan_topic", "/scan")
        self.sectors_topic: str = rospy.get_param("~sectors_topic", "/lidar_sectors")

        self.publish_viz: bool = bool(rospy.get_param("~publish_viz", True))
        self.lidar_viz_topic: str = rospy.get_param("~lidar_viz_topic", "/lidar_sectors/viz")

        config_path: str = rospy.get_param("~config_path", "")
        cfg = _load_yaml(config_path)

        min_range = float(rospy.get_param("~min_range", cfg.get("min_lidar_range", 0.25)))
        max_range = float(rospy.get_param("~max_range", cfg.get("max_lidar_range", 3.0)))
        self.num_sectors: int = int(rospy.get_param("~num_sectors", cfg.get("num_sectors", 40)))

        self.lidar_processor = LidarProcessorROS1(
            min_range=min_range,
            max_range=max_range,
            num_sectors=self.num_sectors,
        )

        self.sectors_pub = rospy.Publisher(self.sectors_topic, Float32MultiArray, queue_size=1)
        self.viz_pub = (
            rospy.Publisher(self.lidar_viz_topic, MarkerArray, queue_size=1)
            if self.publish_viz
            else None
        )

        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo("LidarSectorsNode initialized")
        rospy.loginfo("  Scan topic: %s", self.scan_topic)
        rospy.loginfo("  Sectors topic: %s (%d sectors)", self.sectors_topic, self.num_sectors)
        rospy.loginfo("  publish_viz: %s", str(self.publish_viz))
        if self.publish_viz:
            rospy.loginfo("  Viz topic: %s", self.lidar_viz_topic)

    def scan_callback(self, msg: LaserScan) -> None:
        sectors = self.lidar_processor.process_laser_scan_to_sectors(msg)

        out = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = "sectors"
        dim.size = int(self.num_sectors)
        dim.stride = int(self.num_sectors)
        out.layout.dim = [dim]
        out.layout.data_offset = 0
        out.data = [float(x) for x in sectors]
        self.sectors_pub.publish(out)

        if self.viz_pub is not None:
            self.viz_pub.publish(self._build_markers(sectors, msg))

    def _build_markers(self, sectors: np.ndarray, scan: LaserScan) -> MarkerArray:
        marker_array = MarkerArray()
        n = int(len(sectors))
        if n <= 0:
            return marker_array

        sector_angle = 2.0 * np.pi / float(n)
        frame_id = scan.header.frame_id or "base_link"

        from geometry_msgs.msg import Point  # local import

        for sector_idx in range(n):
            angle_center = -float(sector_idx) * float(sector_angle)
            dist = float(sectors[sector_idx])

            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = scan.header.stamp
            m.ns = "lidar_sectors"
            m.id = sector_idx
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime = rospy.Duration(0.2)

            m.scale.x = 0.02
            m.scale.y = 0.05
            m.scale.z = 0.05

            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 0.9

            p0 = Point()
            p0.x = 0.0
            p0.y = 0.0
            p0.z = 0.0

            p1 = Point()
            p1.x = dist * float(np.cos(angle_center))
            p1.y = dist * float(np.sin(angle_center))
            p1.z = 0.0

            m.points = [p0, p1]
            marker_array.markers.append(m)

        return marker_array


def main() -> None:
    _ = LidarSectorsNode()
    rospy.spin()


if __name__ == "__main__":
    main()

