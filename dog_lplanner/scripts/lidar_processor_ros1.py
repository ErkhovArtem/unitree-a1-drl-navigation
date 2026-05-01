#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 обработчик LaserScan для конвертации в 40 секторов для политики.

Логика совместима с process_lidar_to_sectors из пайплайна обучения (см. POLICY_SPEC_FOR_AGENTS.txt).
"""

import numpy as np
from sensor_msgs.msg import LaserScan


class LidarProcessorROS1:
    """
    Обработчик LaserScan для конвертации в 40 секторов.
    
    Секторизация как в обучении:
    - 40 секторов [-π, π]
    - Фильтрация по min_range/max_range
    - Минимум по сектору
    - Обработка inf/nan значений
    """
    
    def __init__(self, min_range=0.25, max_range=3.0, num_sectors=40):
        """
        Инициализация обработчика.
        
        Args:
            min_range: Минимальная дальность (м), исключаем робота
            max_range: Максимальная дальность (м)
            num_sectors: Количество секторов (40)
        """
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.num_sectors = int(num_sectors)
        
    def process_laser_scan_to_sectors(self, laser_scan_msg):
        """
        Конвертирует LaserScan в 40 секторов (метры).
        
        Args:
            laser_scan_msg: sensor_msgs.msg.LaserScan
            
        Returns:
            numpy array [40] с минимальными дистанциями по секторам (метры)
        """
        if laser_scan_msg is None:
            return np.full(self.num_sectors, self.max_range, dtype=np.float32)
        
        # Получаем ranges из LaserScan
        ranges = np.array(laser_scan_msg.ranges, dtype=np.float32)
        
        # Обрабатываем inf/nan значения (заменяем на max_range)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), self.max_range, ranges)
        
        # Обрезаем до [0, max_range]
        ranges = np.clip(ranges, 0, self.max_range)
        
        # Фильтруем точки слишком близко (min_range)
        valid_mask = ranges >= self.min_range
        valid_ranges = ranges[valid_mask]
        
        # Вычисляем углы для каждого луча
        # LaserScan: angle_min, angle_max, angle_increment
        angle_min = laser_scan_msg.angle_min
        angle_increment = laser_scan_msg.angle_increment
        
        # Создаем массив углов для валидных лучей
        num_ranges = len(ranges)
        angles_all = np.array([angle_min + i * angle_increment for i in range(num_ranges)])
        valid_angles = angles_all[valid_mask]
        
        # Инициализируем массив секторов максимальной дальностью
        sector_min_distances = np.full(self.num_sectors, self.max_range, dtype=np.float32)
        
        # Если нет валидных данных, возвращаем max_range для всех секторов
        if len(valid_ranges) == 0:
            return sector_min_distances
        
        # Нормализуем углы в диапазон [-π, π] (на случай если LaserScan использует другой диапазон)
        valid_angles = np.arctan2(np.sin(valid_angles), np.cos(valid_angles))
        
        # Создаем секторы
        sector_angle = 2 * np.pi / self.num_sectors
        
        for sector_idx in range(self.num_sectors):
            # Определяем границы сектора (точно как в process_lidar_to_sectors)
            angle_min_sector = -np.pi + sector_idx * sector_angle
            angle_max_sector = angle_min_sector + sector_angle
            
            # Находим лучи в этом секторе
            # Для последнего сектора (39) включаем правую границу π
            if sector_idx == self.num_sectors - 1:
                # Последний сектор: [π-Δ, π] (включает π)
                sector_mask = (valid_angles >= angle_min_sector) & (valid_angles <= angle_max_sector)
            else:
                # Обычные секторы: [angle_min, angle_max)
                sector_mask = (valid_angles >= angle_min_sector) & (valid_angles < angle_max_sector)
            
            sector_distances = valid_ranges[sector_mask]
            
            if len(sector_distances) > 0:
                # Находим ближайшее препятствие в секторе
                min_dist = float(sector_distances.min())
                sector_min_distances[sector_idx] = min_dist
            # Если нет лучей в секторе, остается max_range

        # --- Преобразования индексации секторов ---
        # Требуемая конвенция:
        # 1) Индекс 0 смотрит вперёд (0 рад). Для 40 секторов это соответствует
        #    исходному сектору 20 (сдвиг на 180°).
        # 2) Индексы возрастают по часовой стрелке (clockwise).
        #
        # Исходная конвенция выше: индексы возрастают против часовой стрелки,
        # диапазон [-π, π), где "вперёд" (0 рад) находится в середине массива.
        n = int(self.num_sectors)
        if n > 0:
            # Сдвиг на 180°: old[n/2] -> new[0]
            sector_min_distances = np.roll(sector_min_distances, -n // 2)

            # Инверсия направления (по часовой), сохраняя сектор 0 на месте:
            # new[0] = old[0], new[1] = old[-1], new[2] = old[-2], ...
            if n > 1:
                sector_min_distances = np.concatenate(
                    [sector_min_distances[:1], sector_min_distances[:0:-1]]
                )

        return sector_min_distances
