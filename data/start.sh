#!/bin/bash

grep -qxF '#include <pthread.h>' /data/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp || \
sed -i '1s/^/#include <pthread.h>\n/' /data/src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp


if [ ! -f "/data/install/setup.bash" ]; then
    echo "⚠️ ROS2 workspace не собран. Запускаю colcon build..."

    cd /data || exit 1

    source /opt/ros/jazzy/setup.bash

    colcon build \
        --parallel-workers 1 \
        --symlink-install

    if [ $? -ne 0 ]; then
        echo "❌ Ошибка сборки colcon. Завершение."
        exit 1
    fi

    echo "✅ Сборка завершена успешно."
else
    echo "✅ ROS2 workspace уже собран."
fi



source /opt/ros/jazzy/setup.bash
source /data/install/setup.bash

echo "=== 1. Запуск драйвера лидара с автореанимацией ==="
(
    while true; do
        ros2 launch ldlidar_stl_ros2 ld19.launch.py
        echo "⚠️ Лидар упал! Воскрешаю..."
        sleep 2
    done
) &
LIDAR_PID=$!




echo "=== 2. Запуск ROS-ноды камеры ==="
(
    while true; do
        python3 /data/camera_driver.py &
        echo "⚠️ Камера упала! Воскрешаю..."
        sleep 2
    done
) &
CAMERA_DRIVER_PID=$!


echo "=== 3. Запуск Serial моста с Atmega2560 ==="
(
    while true; do
        python3 /data/serial_bridge.py &
        echo "⚠️ Мост упал! Воскрешаю..."
        sleep 2
    done
) &
SERIAL_PID=$!


wait $LIDAR_PID \
     $CAMERA_DRIVER_PID \
     $SERIAL_PID
