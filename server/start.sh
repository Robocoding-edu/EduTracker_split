#!/bin/bash

echo "=== Запуск системы ==="

start_bridge_cmd_vel() {
    echo "=== Запуск cmd_vel relay ==="
    ros2 run topic_tools relay /cmd_vel_nav /cmd_vel &
    BRIDGE_CMD_VEL_PID=$!
}

start_opencv() {
    echo "=== Запуск OpenCV ==="
    python3 ./ros2_ws/openCV.py &
    OPEN_CV_PID=$!
}

start_bridge() {
    echo "=== Запуск Foxglove Bridge ==="
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
    BRIDGE_PID=$!
}

start_slam() {
    echo "=== Запуск Slam Toolbox ==="
    ros2 launch slam_toolbox online_sync_launch.py \
      slam_params_file:=./ros2_ws/my_slam_params.yaml &
    SLAM_PID=$!
}

start_nav2() {
    echo "=== Запуск NAV2 ==="
    ros2 launch ./ros2_ws/nav2_minimal.launch.py &
    NAV_PID=$!
}


stop_process() {
    if [ ! -z "$1" ]; then
        kill $1 2>/dev/null
        wait $1 2>/dev/null
    fi
}


restart_service() {
    case "$1" in

        slam)
            echo "Перезапуск SLAM..."
            stop_process $SLAM_PID
            start_slam
            ;;

        nav2)
            echo "Перезапуск NAV2..."
            stop_process $NAV_PID
            start_nav2
            ;;

        opencv)
            echo "Перезапуск OpenCV..."
            stop_process $OPEN_CV_PID
            start_opencv
            ;;

        bridge)
            echo "Перезапуск Foxglove Bridge..."
            stop_process $BRIDGE_PID
            start_bridge
            ;;

        *)
            echo "Неизвестная служба: $1"
            echo "Доступно: slam nav2 opencv bridge"
            ;;
    esac
}


# === Старт всех сервисов ===

start_bridge_cmd_vel
sleep 1

start_opencv
sleep 2

start_bridge
sleep 3

start_slam
sleep 4

start_nav2
sleep 5


# === Консоль управления ===

echo ""
echo "================================"
echo " Управление:"
echo " restart <slam|nav2|opencv|bridge>"
echo " help"
echo " exit"
echo "================================"
echo ""


while true; do
    read -p "> " CMD ARG

    case "$CMD" in

        restart)
            restart_service "$ARG"
            ;;

        help)
            echo "Команды:"
            echo " restart slam"
            echo " restart nav2"
            echo " restart opencv"
            echo " restart bridge"
            echo " help"
            echo " exit"
            ;;

        exit)
            echo "Выключение..."
            kill $BRIDGE_CMD_VEL_PID \
                 $OPEN_CV_PID \
                 $BRIDGE_PID \
                 $SLAM_PID \
                 $NAV_PID 2>/dev/null
            exit 0
            ;;

        *)
            echo "Неизвестная команда. help"
            ;;
    esac
done
