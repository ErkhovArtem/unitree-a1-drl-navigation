#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LaserScan → 40 sector ranges (meters) for the policy.

Matches training ``process_lidar_to_sectors`` (see POLICY_SPEC_FOR_AGENTS.md).
"""

import numpy as np
from sensor_msgs.msg import LaserScan


class LidarProcessorROS1:
    """Bin LaserScan into ``num_sectors`` buckets over [-π, π]."""

    def __init__(self, min_range=0.25, max_range=3.0, num_sectors=40):
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.num_sectors = int(num_sectors)

    def process_laser_scan_to_sectors(self, laser_scan_msg):
        """Return shape (num_sectors,) float32, minimum range per sector in meters."""
        if laser_scan_msg is None:
            return np.full(self.num_sectors, self.max_range, dtype=np.float32)

        ranges = np.array(laser_scan_msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), self.max_range, ranges)
        ranges = np.clip(ranges, 0, self.max_range)

        valid_mask = ranges >= self.min_range
        valid_ranges = ranges[valid_mask]

        angle_min = laser_scan_msg.angle_min
        angle_increment = laser_scan_msg.angle_increment

        num_ranges = len(ranges)
        angles_all = np.array([angle_min + i * angle_increment for i in range(num_ranges)])
        valid_angles = angles_all[valid_mask]

        sector_min_distances = np.full(self.num_sectors, self.max_range, dtype=np.float32)
        if len(valid_ranges) == 0:
            return sector_min_distances

        valid_angles = np.arctan2(np.sin(valid_angles), np.cos(valid_angles))

        sector_angle = 2 * np.pi / self.num_sectors

        for sector_idx in range(self.num_sectors):
            angle_min_sector = -np.pi + sector_idx * sector_angle
            angle_max_sector = angle_min_sector + sector_angle

            if sector_idx == self.num_sectors - 1:
                sector_mask = (valid_angles >= angle_min_sector) & (valid_angles <= angle_max_sector)
            else:
                sector_mask = (valid_angles >= angle_min_sector) & (valid_angles < angle_max_sector)

            sector_distances = valid_ranges[sector_mask]
            if len(sector_distances) > 0:
                sector_min_distances[sector_idx] = float(sector_distances.min())

        # Reindex: sector 0 = forward (0 rad); indices increase clockwise (see spec).
        n = int(self.num_sectors)
        if n > 0:
            sector_min_distances = np.roll(sector_min_distances, -n // 2)
            if n > 1:
                sector_min_distances = np.concatenate(
                    [sector_min_distances[:1], sector_min_distances[:0:-1]]
                )

        return sector_min_distances
