#!/bin/bash

# Разрешаем подключения к X-серверу
xhost +local:docker

# Запускаем контейнер в фоне
echo "Запускаю контейнер ros_melodic_container в фоне..."
docker compose up -d

echo "Контейнер готов. Теперь ты можешь:"
echo "1. Приаттачиться через VS Code (Docker extension -> Right click -> Attach Visual Studio Code)"
echo "2. Открыть терминал вручную: ./attach_container.sh"