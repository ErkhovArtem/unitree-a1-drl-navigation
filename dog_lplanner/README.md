# dog_lplanner - ROS1 Package for SAC Policy Inference

ROS1 пакет для инференса политики SAC Actor на роботе Unitree A1.

## Структура пакета

```
dog_lplanner/
├── CMakeLists.txt          # Конфигурация сборки
├── package.xml             # Описание пакета
├── scripts/                # Python скрипты (исполняемые)
│   ├── policy_inference_ros1.py  # Основная нода инференса
│   ├── policy_gui.py              # GUI для управления
│   ├── lidar_processor_ros1.py    # Обработка LaserScan
│   ├── inference_onnx.py          # Класс инференса ONNX
│   ├── export_to_onnx.py          # Экспорт модели в ONNX
│   └── lidar_2d_processor.py      # ROS2 версия (для справки)
├── launch/                 # Launch файлы
│   └── policy_inference.launch
├── config/                 # Конфигурационные файлы
│   └── a1_ros1.yaml
├── sac_actor.onnx          # ONNX модель политики
└── README.md               # Этот файл
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

### 1. Подготовка workspace

Пакет должен быть размещен в catkin workspace. Если у вас уже есть workspace:

```bash
# В контейнере
cd /root/catkin_ws/src

# Скопируйте или создайте симлинк на dog_lplanner
# Например, если пакет в /workspace/dog_lplanner:
ln -s /workspace/dog_lplanner .
```

Или если пакет уже в нужном месте:

```bash
# Проверьте структуру
cd /root/catkin_ws/src
ls -la dog_lplanner/
```

### 2. Сборка

```bash
# В контейнере
cd /root/catkin_ws

# Источите setup.bash если еще не сделали
source /opt/ros/melodic/setup.bash

# Соберите пакет
catkin_make

# Или если используете catkin tools:
# catkin build dog_lplanner
```

### 3. Установка (опционально)

После сборки пакет автоматически доступен. Но нужно засорсить workspace:

```bash
source /root/catkin_ws/devel/setup.bash
```

### 4. Проверка установки

```bash
# Проверьте что пакет найден
rospack find dog_lplanner

# Должен вывести путь: /root/catkin_ws/src/dog_lplanner
```

## Запуск

### Вариант 1: Через launch файл

```bash
# В контейнере, после source devel/setup.bash
roslaunch dog_lplanner policy_inference.launch
```

### Вариант 2: Через скрипт (если пакет не собран)

```bash
cd /path/to/dog_lplanner
./run_policy.sh
```

### Вариант 3: Вручную

```bash
# Терминал 1: Нода инференса
rosrun dog_lplanner policy_inference_ros1.py

# Терминал 2: GUI
rosrun dog_lplanner policy_gui.py
```

## Использование

1. **Запустите ноду инференса и GUI** (см. выше)

2. **В RViz:**
   - Подпишитесь на топик `/lidar_sectors_viz` для визуализации секторов лидара
   - Используйте инструмент "Publish Point" (2D Nav Goal) для установки целевой точки
   - Точка будет опубликована в `/clicked_point` (фрейм `map`)

3. **В GUI:**
   - Нажмите "START" для запуска политики
   - Нажмите "STOP" для остановки (робот остановится)

4. **Проверка работы:**
   ```bash
   # Проверьте публикацию команд
   rostopic echo /high_cmd
   
   # Проверьте визуализацию лидара
   rostopic echo /lidar_sectors_viz
   ```

## Параметры

Основные параметры настраиваются в `config/a1_ros1.yaml`:

- `cmd_scale`: Масштаб команд [vx, vy, w]
- `max_lidar_range`: Максимальная дальность лидара (м)
- `min_lidar_range`: Минимальная дальность (м)
- `num_sectors`: Количество секторов (40)
- Топики: `/scan`, `/high_state`, `/clicked_point`, `/high_cmd`

## Визуализация

Нода публикует MarkerArray в топик `/lidar_sectors_viz`:
- **Стрелки** показывают направление и расстояние до препятствий по секторам
- **Лейблы** показывают номер сектора (0-39) и угол в радианах

Цвет стрелок:
- **Красный** = близко (препятствие)
- **Зеленый** = далеко (свободно)

## Troubleshooting

### Пакет не найден
```bash
# Убедитесь что workspace засорсен
source /root/catkin_ws/devel/setup.bash

# Проверьте что пакет в src/
rospack find dog_lplanner
```

### Ошибки импорта
```bash
# Убедитесь что все зависимости установлены
pip3 list | grep -E "numpy|onnxruntime|yaml|rospkg"
```

### TF ошибки
```bash
# Проверьте что tf публикуется
rostopic echo /tf
# Должны быть трансформации между map и base_link
```

### Не публикуются сообщения в топики `/policy_inference/lidar_sectors_viz`, /policy_inference/cmd_viz, `/high_cmd`:

```bash
# В контейнере выполнить:
export ROS_IP=<IP машины где запущен контейнер, в сети робота>
```

## Примечания

- Модель `sac_actor.onnx` должна быть в корне пакета
- Конфиг `a1_ros1.yaml` должен быть в `config/`
- Все Python скрипты должны быть исполняемыми (`chmod +x`)
