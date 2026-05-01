#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 нода для инференса политики SAC Actor.

Подписывается на:
- /scan (sensor_msgs/LaserScan) - данные лидара
- /high_state (unitree_legged_msgs/HighState) - угловая скорость rotateSpeed
- /clicked_point (geometry_msgs/PointStamped) - цель из RViz; нода переводит точку в map_frame (~map)

Публикует:
- /high_cmd (unitree_legged_msgs/HighCmd) - команды управления

Управление состоянием через ROS параметр /policy_running (bool).
Безопасный режим через /policy_safe_mode (bool): команды на робота НЕ отправляются,
но визуализация скорости в RViz публикуется.
"""

import rospy
import numpy as np
import yaml
from pathlib import Path
from collections import deque
from typing import Optional

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from unitree_legged_msgs.msg import HighCmd, HighState, LED

# Импортируем классы для инференса
import sys
sys.path.insert(0, str(Path(__file__).parent))
from inference_onnx import SACInference
from lidar_processor_ros1 import LidarProcessorROS1
from tf_tree_ros1 import TFGraph

# Для ROS пакета используем rospkg для поиска файлов
try:
    import rospkg
    rospack = rospkg.RosPack()
    ROSPKG_AVAILABLE = True
except:
    ROSPKG_AVAILABLE = False


class PolicyInferenceNode:
    """ROS1 нода для инференса политики."""
    
    def __init__(self):
        """Инициализация ноды."""
        rospy.init_node('policy_inference', anonymous=True)
        
        # Параметры из ROS параметров или конфига
        # Пытаемся найти пакет через rospkg
        if ROSPKG_AVAILABLE:
            try:
                pkg_path = rospack.get_path('dog_lplanner')
                default_config = str(Path(pkg_path) / 'config' / 'a1_ros1.yaml')
                default_model = str(Path(pkg_path) / 'sac_actor.onnx')
            except:
                # Fallback на относительные пути
                default_config = str(Path(__file__).parent.parent / 'config' / 'a1_ros1.yaml')
                default_model = str(Path(__file__).parent.parent / 'sac_actor.onnx')
        else:
            # Fallback на относительные пути
            default_config = str(Path(__file__).parent.parent / 'config' / 'a1_ros1.yaml')
            default_model = str(Path(__file__).parent.parent / 'sac_actor.onnx')
        
        config_path = rospy.get_param('~config_path', default_config)
        model_path = rospy.get_param('~model_path', default_model)
        
        # Если пути относительные, делаем их абсолютными относительно директории скрипта
        if not Path(config_path).is_absolute():
            config_path = str(Path(__file__).parent / config_path)
        if not Path(model_path).is_absolute():
            model_path = str(Path(__file__).parent / model_path)
        
        # Топики
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.high_state_topic = rospy.get_param('~high_state_topic', '/high_state')
        self.clicked_point_topic = rospy.get_param('~clicked_point_topic', '/clicked_point')
        # Публикуем напрямую в /high_cmd (безопасность через /policy_safe_mode)
        self.high_cmd_topic = rospy.get_param('~high_cmd_topic', '/high_cmd')
        self.lidar_viz_topic = rospy.get_param('~lidar_viz_topic', '/policy_inference/lidar_viz')
        self.cmd_viz_topic = rospy.get_param('~cmd_viz_topic', '/policy_inference/cmd_viz')
        self.cmd_viz_frame = rospy.get_param('~cmd_viz_frame', 'base_link')
        self.cmd_viz_scale = float(rospy.get_param('~cmd_viz_scale', 1.0))  # meters per (m/s)
        
        # Фреймы для tf (из параметров или конфига)
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.map_frame = rospy.get_param('~map_frame', config.get('map_frame', 'map'))
        self.base_frame = rospy.get_param('~base_frame', config.get('base_frame', 'base_link'))
        
        # TF graph (python-only) for Python3 compatibility on ROS Melodic
        self.tf_graph = TFGraph()
        
        # Параметры цели (fallback, если нет clicked_point)
        self.target_x = rospy.get_param('~target_x', 5.0)
        self.target_y = rospy.get_param('~target_y', 0.0)
        
        # Частота инференса
        self.inference_rate = rospy.get_param('~inference_rate', 10.0)  # 10 Hz
        
        # Параметры лидара из конфига (config уже загружен выше)
        min_range = config.get('min_lidar_range', 0.25)
        max_range = config.get('max_lidar_range', 3.0)
        num_sectors = config.get('num_sectors', 40)
        
        self.lidar_processor = LidarProcessorROS1(
            min_range=min_range,
            max_range=max_range,
            num_sectors=num_sectors
        )
        
        # Инициализируем инференс
        if not Path(model_path).exists():
            rospy.logerr(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        self.inference = SACInference(model_path, config_path)
        
        # Кэш данных
        self.last_lidar_scan = None
        self.last_lidar_sectors = None
        self.last_high_state = None
        self.current_target_x = self.target_x
        self.current_target_y = self.target_y
        self.target_received = False  # Флаг получения цели из clicked_point
        # Цель из RViz: после клика храним в map_frame (~map), в base пересчитываем каждый тик
        self.goal_source_frame = self.map_frame.strip("/")
        self.goal_point_x = 0.0
        self.goal_point_y = 0.0
        self.goal_point_z = 0.0
        
        # Подписки
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_callback,
            queue_size=1
        )
        
        self.high_state_sub = rospy.Subscriber(
            self.high_state_topic,
            HighState,
            self.high_state_callback,
            queue_size=1
        )
        
        # Подписка на clicked_point из RViz
        self.clicked_point_sub = rospy.Subscriber(
            self.clicked_point_topic,
            PointStamped,
            self.clicked_point_callback,
            queue_size=1
        )
        
        # Публикатор команд
        self.cmd_pub = rospy.Publisher(
            self.high_cmd_topic,
            HighCmd,
            queue_size=1
        )

        # Публикатор визуализации "команды скорости" (вектор политики)
        self.cmd_viz_pub = rospy.Publisher(
            self.cmd_viz_topic,
            MarkerArray,
            queue_size=1
        )
        
        # Публикатор визуализации лидара
        self.lidar_viz_pub = rospy.Publisher(
            self.lidar_viz_topic,
            MarkerArray,
            queue_size=1
        )
        
        # Инициализируем ROS параметр для состояния
        if not rospy.has_param('/policy_running'):
            rospy.set_param('/policy_running', False)
        if not rospy.has_param('/policy_safe_mode'):
            # Safe by default
            rospy.set_param('/policy_safe_mode', True)
        
        rospy.loginfo("Policy Inference Node initialized")
        rospy.loginfo(f"  Model: {model_path}")
        rospy.loginfo(f"  Config: {config_path}")
        rospy.loginfo(f"  Scan topic: {self.scan_topic}")
        rospy.loginfo(f"  High state topic: {self.high_state_topic}")
        rospy.loginfo(f"  Clicked point topic: {self.clicked_point_topic}")
        rospy.loginfo(f"  HighCmd topic: {self.high_cmd_topic}")
        rospy.loginfo(f"  Cmd viz topic: {self.cmd_viz_topic} (frame: {self.cmd_viz_frame})")
        rospy.loginfo(f"  Map frame: {self.map_frame}, Base frame: {self.base_frame}")
        rospy.loginfo(f"  Inference rate: {self.inference_rate} Hz")

    def publish_cmd_visualization(self, vx: float, vy: float, w: float, safe_mode: bool):
        """Публикует в RViz вектор скорости (vx, vy) + подпись."""
        ma = MarkerArray()

        # Arrow (скорость в плоскости)
        m = Marker()
        m.header.frame_id = self.cmd_viz_frame
        m.header.stamp = rospy.Time.now()
        m.ns = "policy_cmd"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.lifetime = rospy.Duration(0.25)

        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0

        end = Point()
        end.x = float(vx * self.cmd_viz_scale)
        end.y = float(vy * self.cmd_viz_scale)
        end.z = 0.0

        m.points = [start, end]
        m.scale.x = 0.04  # shaft diameter
        m.scale.y = 0.10  # head diameter
        m.scale.z = 0.10  # head length

        if safe_mode:
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.2
        else:
            m.color.r = 0.9
            m.color.g = 0.2
            m.color.b = 0.2
        m.color.a = 0.9
        ma.markers.append(m)

        # Text
        t = Marker()
        t.header.frame_id = self.cmd_viz_frame
        t.header.stamp = m.header.stamp
        t.ns = "policy_cmd"
        t.id = 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.lifetime = rospy.Duration(0.25)
        t.pose.position.x = 0.0
        t.pose.position.y = 0.0
        t.pose.position.z = 0.35
        t.scale.z = 0.18
        t.color.r = 1.0
        t.color.g = 1.0
        t.color.b = 1.0
        t.color.a = 1.0
        running = bool(rospy.get_param('/policy_running', False))
        t.text = (
            f"vx={vx:.2f} vy={vy:.2f} w={w:.2f}\\n"
            f"RUN={'ON' if running else 'OFF'}  SAFE={'ON' if safe_mode else 'OFF'}"
        )
        ma.markers.append(t)

        self.cmd_viz_pub.publish(ma)
        
    def scan_callback(self, msg):
        """Callback для LaserScan."""
        self.last_lidar_scan = msg
        # Всегда обрабатываем и публикуем визуализацию по факту прихода лидара,
        # независимо от /policy_running (чтобы стрелки были всегда).
        try:
            sectors = self.lidar_processor.process_laser_scan_to_sectors(msg)
            self.last_lidar_sectors = sectors
            self.publish_lidar_visualization(sectors)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Lidar processing/viz failed: {e}")
    
    def high_state_callback(self, msg):
        """Callback для HighState: сохраняем состояние, angular_vel берём из rotateSpeed."""
        self.last_high_state = msg
    
    def clicked_point_callback(self, msg):
        """Callback для clicked_point из RViz.

        RViz кладёт в header.frame_id текущий Fixed Frame; чтобы цель не ломалась при его смене,
        переводим точку в map_frame и сохраняем только координаты в карте.
        """
        src = msg.header.frame_id.strip("/")
        map_id = self.map_frame.strip("/")
        point_in_map = self.tf_graph.transform_point(map_id, msg)

        if point_in_map is not None:
            self.goal_source_frame = map_id
            self.goal_point_x = float(point_in_map.point.x)
            self.goal_point_y = float(point_in_map.point.y)
            self.goal_point_z = float(point_in_map.point.z)
        else:
            rospy.logwarn_throttle(
                1.0,
                f"Cannot transform clicked point {src} -> {map_id}; "
                f"storing raw in '{src}' (fix TF or set RViz Fixed Frame to {map_id}).",
            )
            self.goal_source_frame = src
            self.goal_point_x = float(msg.point.x)
            self.goal_point_y = float(msg.point.y)
            self.goal_point_z = float(msg.point.z)

        self.target_received = True

        stamped_goal = PointStamped()
        stamped_goal.header.frame_id = self.goal_source_frame
        stamped_goal.header.stamp = rospy.Time(0)
        stamped_goal.point.x = self.goal_point_x
        stamped_goal.point.y = self.goal_point_y
        stamped_goal.point.z = self.goal_point_z

        point_in_base = self.tf_graph.transform_point(self.base_frame, stamped_goal)
        if point_in_base is not None:
            self.current_target_x = point_in_base.point.x
            self.current_target_y = point_in_base.point.y
            if point_in_map is not None:
                rospy.loginfo(
                    f"Target in {map_id}: ({self.goal_point_x:.2f}, {self.goal_point_y:.2f}) "
                    f"(click was in '{src}') -> ({self.current_target_x:.2f}, {self.current_target_y:.2f}) "
                    f"in {self.base_frame}"
                )
            else:
                rospy.loginfo(
                    f"Target in '{self.goal_source_frame}' (no TF to {map_id}; click was in '{src}'): "
                    f"({self.goal_point_x:.2f}, {self.goal_point_y:.2f}) -> "
                    f"({self.current_target_x:.2f}, {self.current_target_y:.2f}) in {self.base_frame}"
                )
        else:
            rospy.logwarn_throttle(
                1.0,
                f"Goal in '{self.goal_source_frame}' ({self.goal_point_x:.2f}, {self.goal_point_y:.2f}); "
                f"waiting for TF {self.goal_source_frame} -> {self.base_frame}",
            )
    
    def publish_lidar_visualization(self, lidar_sectors):
        """
        Публикует MarkerArray с визуализацией секторов лидара.
        Каждый луч имеет стрелку и лейбл с номером и углом.
        
        Args:
            lidar_sectors: numpy array [40] с дистанциями по секторам (метры)
        """
        if self.last_lidar_scan is None:
            return
        
        marker_array = MarkerArray()
        num_sectors = len(lidar_sectors)
        sector_angle = 2 * np.pi / num_sectors
        
        for sector_idx in range(num_sectors):
            # Конвенция секторов (см. lidar_processor_ros1.py):
            # - sector 0 смотрит вперёд (0 рад)
            # - индексы возрастают по часовой стрелке
            angle_center = -float(sector_idx) * float(sector_angle)
            
            distance = float(lidar_sectors[sector_idx])
            
            # Создаем маркер-стрелку
            marker = Marker()
            marker.header.frame_id = self.last_lidar_scan.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "lidar_sectors"
            marker.id = sector_idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Начальная точка (центр)
            start_point = Point()
            start_point.x = 0.0
            start_point.y = 0.0
            start_point.z = 0.0
            
            # Конечная точка (на расстоянии distance)
            end_point = Point()
            end_point.x = float(distance * np.cos(angle_center))
            end_point.y = float(distance * np.sin(angle_center))
            end_point.z = 0.0

            # if sector_idx>=20 and sector_idx<=22:
                # print(f"Id: {sector_idx}, x: {end_point.x}, y: {end_point.y}")
            
            marker.points = [start_point, end_point]
            
            # Размеры стрелки
            marker.scale.x = 0.05  # толщина
            marker.scale.y = 0.1   # ширина головки
            marker.scale.z = 0.1  # высота головки
            
            # Цвет (от красного близко к зеленому далеко)
            ratio = distance / self.lidar_processor.max_range
            marker.color.r = float(1.0 - ratio)
            marker.color.g = float(ratio)
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(marker)
            
            # Создаем лейбл с номером и углом
            label_marker = Marker()
            label_marker.header.frame_id = self.last_lidar_scan.header.frame_id
            label_marker.header.stamp = rospy.Time.now()
            label_marker.ns = "lidar_labels"
            label_marker.id = sector_idx
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            
            # Позиция лейбла (немного дальше конца стрелки)
            label_offset = 0.2
            label_marker.pose.position.x = float((distance + label_offset) * np.cos(angle_center))
            label_marker.pose.position.y = float((distance + label_offset) * np.sin(angle_center))
            label_marker.pose.position.z = 0.1
            
            # Текст лейбла: номер и угол центра сектора (рад)
            label_angle = angle_center
            label_marker.text = f"{sector_idx}\n{label_angle:.3f}"
            
            # Размер текста
            label_marker.scale.z = 0.15
            
            # Цвет текста
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            
            label_marker.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(label_marker)
        
        self.lidar_viz_pub.publish(marker_array)
    
    def get_target_info(self):
        """
        Вычисляет информацию о цели в локальной системе робота (base_link).

        Если цель задана через clicked_point, она хранится в map_frame (см. clicked_point_callback);
        на каждом вызове — пересчёт в base_frame по TF, пока робот движется.

        Returns:
            (distance, sin_angle, cos_angle)
        """
        if not self.target_received:
            target_x = self.target_x
            target_y = self.target_y
        else:
            stamped = PointStamped()
            stamped.header.frame_id = self.goal_source_frame
            stamped.header.stamp = rospy.Time(0)
            stamped.point.x = self.goal_point_x
            stamped.point.y = self.goal_point_y
            stamped.point.z = self.goal_point_z
            point_in_base = self.tf_graph.transform_point(self.base_frame, stamped)
            if point_in_base is None:
                rospy.logwarn_throttle(
                    1.0,
                    f"TF missing for goal: {self.goal_source_frame} -> {self.base_frame}; "
                    f"using last known target in base",
                )
                target_x = self.current_target_x
                target_y = self.current_target_y
            else:
                target_x = float(point_in_base.point.x)
                target_y = float(point_in_base.point.y)
                self.current_target_x = target_x
                self.current_target_y = target_y
        
        # В base_link робот находится в (0, 0), поэтому вектор к цели = координаты цели
        dx_local = target_x
        dy_local = target_y
        
        # Расстояние до цели
        distance = np.sqrt(dx_local**2 + dy_local**2)
        
        # Угол к цели
        if distance < 1e-6:
            sin_angle = 0.0
            cos_angle = 1.0
        else:
            sin_angle = dy_local / distance
            cos_angle = dx_local / distance
        
        return distance, sin_angle, cos_angle
    
    def run_inference_loop(self):
        """Основной цикл инференса."""
        rate = rospy.Rate(self.inference_rate)
        
        while not rospy.is_shutdown():
            # Проверяем состояние (старт/стоп)
            is_running = rospy.get_param('/policy_running', False)
            safe_mode = rospy.get_param('/policy_safe_mode', True)
            
            if not is_running:
                # В режиме стоп: показываем нулевой вектор; на робота не шлём если safe_mode=ON
                self.publish_cmd_visualization(0.0, 0.0, 0.0, bool(safe_mode))
                rate.sleep()
                continue
            
            # Проверяем наличие данных
            if self.last_lidar_sectors is None:
                rospy.logwarn_throttle(1.0, "No lidar data received")
                rate.sleep()
                continue
            
            if self.last_high_state is None:
                rospy.logwarn_throttle(1.0, "No high_state data received")
                rate.sleep()
                continue
            
            # Проверяем наличие цели (предупреждение только если никогда не получали)
            if not self.target_received:
                rospy.logwarn_throttle(5.0, 
                    f"No target received. Click a point in RViz on topic {self.clicked_point_topic} "
                    f"or set target via parameters. Using fallback: ({self.target_x}, {self.target_y})")
            
            try:
                lidar_sectors = self.last_lidar_sectors
                
                # Угловая скорость из HighState (unitree_legged_msgs)
                angular_vel = self.last_high_state.rotateSpeed
                
                # Получаем информацию о цели
                distance, sin_angle, cos_angle = self.get_target_info()
                
                # Получаем предыдущее действие (если есть)
                prev_action = None
                if self.inference.action_history is not None and len(self.inference.action_history) > 0:
                    # Берем последнее действие из истории
                    prev_action = list(self.inference.action_history)[-1]
                
                # Выполняем инференс
                raw_action, scaled_action = self.inference.predict(
                    lidar_sectors,
                    angular_vel,
                    distance,
                    sin_angle,
                    cos_angle,
                    prev_action
                )
                print("distance", distance)
                print("sin_angle", sin_angle)
                print("cos_angle", cos_angle)

                vx = float(scaled_action[0]) * 0.3
                vy = float(scaled_action[1]) * 0.3
                w = float(scaled_action[2]) * 0.3

                # Всегда показываем вектор скорости в RViz
                self.publish_cmd_visualization(vx, vy, w, bool(safe_mode))
                
                # Команды на робота: только если safe_mode=OFF
                if not bool(safe_mode):
                    cmd = HighCmd()
                    # A1/Aliengo format
                    cmd.levelFlag = 0x00
                    # cmd.commVersion = 0
                    # cmd.robotID = 0
                    # cmd.SN = 0
                    # cmd.bandWidth = 0
                    cmd.mode = 2  # velocity walking
                    cmd.forwardSpeed = vx
                    cmd.sideSpeed = vy + 0.1
                    cmd.rotateSpeed = w
                    cmd.bodyHeight = 0.0
                    cmd.footRaiseHeight = 0.0

                    cmd.yaw = 0.0
                    cmd.pitch = 0.0
                    cmd.roll = 0.0
                        # LED array (4 LEDs)
                    cmd.led = [LED() for _ in range(4)]
                    for i in range(4):
                        cmd.led[i].r = 0
                        cmd.led[i].g = 0
                        cmd.led[i].b = 0
                    
                    # Remote control data
                    cmd.wirelessRemote = [0] * 40
                    # cmd.reserve = 0
                    cmd.crc = 0
                    self.cmd_pub.publish(cmd)
                
                rospy.logdebug(
                    f"Policy cmd: vx={vx:.3f}, vy={vy:.3f}, w={w:.3f} (SAFE={'ON' if bool(safe_mode) else 'OFF'})"
                )
                
            except Exception as e:
                rospy.logerr(f"Error in inference loop: {e}")
                import traceback
                rospy.logerr(traceback.format_exc())
            
            rate.sleep()


def main():
    """Главная функция."""
    try:
        node = PolicyInferenceNode()
        node.run_inference_loop()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
