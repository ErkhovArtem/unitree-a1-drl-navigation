# ROS Melodic Docker Container

## Управление контейнером

### Первый запуск (сборка)
```bash
./build.sh
```

### Запуск контейнера и подключение
```bash
./run_container.sh  # Запустит контейнер и откроет bash
```

### Открыть дополнительный терминал в контейнере
```bash
./attach_container.sh
# или
docker exec -it ros_melodic_container bash
```

### Из VSCode
В интегрированном терминале VSCode (Ctrl+Shift+~):
```bash
docker exec -it ros_melodic_container bash
```

### Остановка контейнера
```bash
./stop_container.sh
# или
docker compose down
```

## Работа с VSCode

⚠️ **Важно**: ROS Melodic основан на Ubuntu 18.04 с GLIBC 2.27, а VSCode требует GLIBC >= 2.28. 
Поэтому Dev Containers не работает напрямую.

**Рекомендуемый способ работы:**

1. Редактируйте файлы в VSCode на хосте (папка `/home/griga/A1_container`)
2. Запустите контейнер: `./run_container.sh`
3. Выполняйте команды в контейнере:
   - Через терминал: `./attach_container.sh`
   - Или через интегрированный терминал VSCode: `Ctrl+Shift+~` → `./docker/attach_container.sh`
   
## Запуск RViz

В контейнере:
```bash
rviz
```

## Полезные команды

```bash
# Проверка статуса контейнера
docker ps

# Просмотр логов контейнера
docker logs ros_melodic_container

# Перезапуск контейнера
./stop_container.sh && ./run_container.sh
```
