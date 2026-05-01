# dog_lplanner - ROS1 Package for SAC Policy Inference

ROS1 пакет для инференса политики SAC Actor на роботе Unitree A1.

## Структура пакета

```
dog_lplanner/
├── CMakeLists.txt
├── package.xml
├── POLICY_SPEC_FOR_AGENTS.txt   # Спека формата наблюдений / лидара
├── scripts/
│   ├── policy_inference_ros1.py  # Основная нода инференса
│   ├── policy_gui.py
│   ├── lidar_processor_ros1.py   # LaserScan → 40 секторов (м)
│   ├── lidar_sectors_node.py     # Опционально: отдельная нода /scan → /lidar_sectors (см. lidar_sectors.launch)
│   ├── inference_onnx.py
│   ├── export_to_onnx.py         # Офлайн: экспорт .pth → .onnx
│   └── tf_tree_ros1.py           # TF из /tf (Python3 без tf2_py)
├── launch/
│   ├── policy_inference.launch
│   └── lidar_sectors.launch
├── config/
│   ├── a1_ros1.yaml
│   └── policy_visualization.rviz
├── sac_actor.onnx
└── README.md
```

## Зависимости

### Системные пакеты (уже установлены в контейнере):
- `python3-tk` - для GUI
- `ros-melodic-*` - ROS пакеты

### Python пакеты (уже установлены в контейнере):
- `numpy<1.20`
- `onnxruntime==1.10.0`
- `pyyaml`
- `rospkg`

### ROS пакеты:
- `unitree_legged_msgs` - сообщения для Unitree

## Сборка пакета

```bash
# В контейнере
cd /root/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make install
source /root/catkin_ws/install/setup.bash
```

## Использование

1. **Запустите ноду инференса и GUI**

   ```bash
   roslaunch dog_lplanner policy_inference.launch
   ```

2. **В RViz:**
   - Визуализация секторов лидара: топик **`/policy_inference/lidar_viz`** (MarkerArray, по умолчанию; параметр `~lidar_viz_topic`)
   - Вектор команды политики: **`/policy_inference/cmd_viz`**
   - Инструмент "Publish Point" → **`/clicked_point`** (фрейм задаётся Fixed Frame в RViz; нода нормализует цель в `map`)

3. **В GUI:** START / STOP политики (`/policy_running`).

4. **Проверка:**

   ```bash
   rostopic echo /high_cmd
   rostopic echo /policy_inference/lidar_viz
   ```

## Параметры

Основные параметры в `config/a1_ros1.yaml`:

- `cmd_scale`, `max_lidar_range`, `min_lidar_range`, `num_sectors`, `goal_reached_distance`
- Топики: `/scan`, `/high_state`, `/clicked_point`, `/high_cmd`

## Визуализация (policy_inference)

- **`/policy_inference/lidar_viz`**: стрелки по секторам лидара, лейблы с индексом и углом
- **`/policy_inference/cmd_viz`**: стрелка скорости (vx, vy) в `cmd_viz_frame` (по умолчанию `base_link`)

## Troubleshooting

### Пакет не найден

```bash
source /root/catkin_ws/install/setup.bash
rospack find dog_lplanner
```

### Ошибки импорта

```bash
pip3 list | grep -E "numpy|onnxruntime|yaml|rospkg"
```

### TF

```bash
rostopic echo /tf
```

### Нет сообщений на `/policy_inference/lidar_viz`, `/policy_inference/cmd_viz`, `/high_cmd`

Проверьте `ROS_IP` / сеть (в Docker см. `docker/ros_detect_ip.sh` и `docker/README.md`).

## Примечания

- Модель `sac_actor.onnx` в корне пакета (опционально при `catkin_make install`)
- Скрипты в `scripts/` должны быть исполняемыми (`chmod +x`)
