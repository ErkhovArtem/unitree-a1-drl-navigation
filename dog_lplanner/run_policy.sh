#!/bin/bash
# Скрипт для запуска инференса политики и GUI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Проверяем наличие ROS
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS environment not sourced. Please run: source /opt/ros/melodic/setup.bash"
    exit 1
fi

# Запускаем ноду инференса в фоне
python3 "$SCRIPT_DIR/policy_inference_ros1.py" &
INFERENCE_PID=$!

# Небольшая задержка перед запуском GUI
sleep 1

# Запускаем GUI
python3 "$SCRIPT_DIR/policy_gui.py" &
GUI_PID=$!

echo "Policy inference started (PID: $INFERENCE_PID)"
echo "Policy GUI started (PID: $GUI_PID)"
echo "Press Ctrl+C to stop"

# Ждем завершения
wait $INFERENCE_PID $GUI_PID
