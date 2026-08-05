# Agents.md — Документация архитектуры робота EduTracker

## Обзор проекта

EduTracker — это мобильный робот на базе ROS 2 (Jazzy), оснащённый лидаром, камерой и дифференциальным приводом. Проект состоит из двух основных частей:
- **Raspberry Pi** — основной вычислительный блок, запускающий ROS 2 ноды в Docker-контейнере
- **Arduino Mega 2560** — микроконтроллер для низкоуровневого управления моторами и чтения датчиков

---

## Архитектура системы

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi (Docker)                        │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ LiDAR       │  │ Camera      │  │ Serial Bridge           │  │
│  │ Driver      │  │ Driver      │  │ (ROS ↔ Arduino)         │  │
│  │ ldlidar_    │  │ camera_     │  │ serial_bridge.py        │  │
│  │ stl_ros2    │  │ driver.py   │  │                         │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘  │
│         │               │                      │                │
│         ▼               ▼                      ▼                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ /scan       │  │ /camera/    │  │ /cmd_vel                │  │
│  │ (LaserScan) │  │ image_raw   │  │ /cmd_vel_head           │  │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘  │
│         │               │                      │                │
│         ▼               ▼                      ▼                │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │              SLAM Toolbox + NAV2 + OpenCV                   ││
│  │         (server/start.sh — Foxglove Bridge)                 ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ UART (/dev/sensors/arduino)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Arduino Mega 2560                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ Motor       │  │ Encoders    │  │ VL53L0X ToF Sensors (4) │  │
│  │ Control     │  │ (77 ticks/  │  │ Front, Rear, Left, Right│  │
│  │ (PWM + H-   │  │  rev)        │  │ → obstacle avoidance    │  │
│  │ Bridge)     │  │             │  │                         │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Компоненты

### 1. Arduino (Низкоуровневое управление)

**Файлы:**
- `arduino/src/main.ino` — основная прошивка
- `arduino/platformio.ini` — конфигурация PlatformIO (ATmega2560)
- `arduino/flash_arduino.sh` — скрипт прошивки

**Функции:**
- Управление двумя моторами через H-мост (PWM + направление)
- Чтение энкодеров (77 тиков/оборот) с аппаратными прерываниями
- 4 датчика расстояния VL53L0X (Front, Rear, Left, Right) для аварийной остановки
- Дифференциальное управление сервоприводами камеры (наклон + поворот)
-Serial-протокол с Raspberry Pi:
  - `#MOVE:linear,angular` — команда движения
  - `#HEAD:pitch,yaw` — команда положения камеры
  - `DAT:encL,encR,distL,distR,btn` — телеметрия

**Параметры движения:**
- `V_MIN = 0.035 м/с`, `V_MAX = 0.340 м/с`
- `MAX_OMEGA = 1.2 рад/с`
- `WHEEL_SEP = 0.231 м` (колея)
- Дистанции аварийной остановки: Front=250мм, Rear=200мм, Side=150мм

---

### 2. Serial Bridge (ROS 2 ↔ Arduino)

**Файл:** `data/serial_bridge.py`

**Функции:**
- Преобразование команд `/cmd_vel` (Twist) в Serial-команды для Arduino
- Публикация одометрии (`/odom`) с расчётом позиции по энкодерам
- Трансляция TF (`odom` → `base_link`)
- Публикация сырых данных датчиков:
  - `/robot/encoder_left`, `/robot/encoder_right` (Int64)
  - `/robot/distance_left`, `/robot/distance_right` (Int32)
  - `/robot/button_status` (Bool)
- Управление камерой через `/cmd_vel_head` (Twist: linear.x=pitch, angular.z=yaw)

**Параметры робота:**
- Диаметр колеса: 0.065 м
- Колея: 0.231 м
- Знаки энкодеров: left=-1, right=-1 (инверсия)

---

### 3. LiDAR Driver

**Файл:** `data/lidar.launch.py`

**Пакет:** `ldlidar_stl_ros2` (LD19)

**Публикуемые топики:**
- `/scan` (LaserScan)

**Параметры:**
- Порт: `/dev/sensors/lidar` (230400 бод)
- Направление сканирования: против часовой стрелки
- Frame ID: `laser_link`

---

### 4. Camera Driver

**Файл:** `data/camera_driver.py`

**Функции:**
- Приём UDP-потока с камеры (`udp://127.0.0.1:8554`)
- Публикация кадров в `/camera/image_raw` (BGR8, ~10 FPS)
- Использование `cv_bridge` для конвертации в ROS Image

---

### 5. Robot Description (URDF)

**Файлы:**
- `data/src/edutracker_description/urdf/robot.urdf.xacro`
- `data/src/edutracker_description/launch/robot_description.launch.py`

**Структура робота:**
- `base_link` — базовая система координат
- `body_link` — корпус (цилиндр, радиус 0.0775м, высота 0.245м)
- `left_wheel_link`, `right_wheel_link` — колёса (радиус 0.0325м)
- `laser_link` — лидар (25см выше базы)
- `camera_link` — камера

---

### 6. SLAM Toolbox

**Запуск:** `data/start.sh` → `slam_toolbox online_sync_launch.py`

**Параметры:** `server/data/my_slam_params.yaml`

**Функции:**
- Построение 2D-карты в реальном времени
- Сохранение карты для последующей навигации

---

### 7. NAV2 (Навигация)

**Файлы:**
- `server/data/nav2_minimal.launch.py`
- `server/data/nav2_params.yaml`

**Компоненты:**
- `controller_server` — локальный планировщик траектории
- `planner_server` — глобальный планировщик (A*/Dijkstra)
- `bt_navigator` — дерево поведения навигации
- `behavior_server` —_recovery behavior_ (разворот, отъезд назад)
- `lifecycle_manager` — автозапуск всех нод

---

### 8. OpenCV Processor

**Файл:** `server/data/openCV.py`

**Функции:**
- Подписка на `/camera/image_raw`
- Детекция красных объектов (HSV-фильтрация)
- Оценка расстояния до объекта по размеру bounding box
- Публикация:
  - `/camera/image_processed` — кадр с разметкой
  - `/robot/detected_objects` (Marker) — позиция объектов в `base_link`

**Алгоритм оценки расстояния:**
```python
distance = (real_object_width * focal_length) / bounding_box_width
angle = offset_x * (fov_h / 2) / center_x
robot_x = distance * cos(angle)
robot_y = -distance * sin(angle)
```

---

### 9. Server (Foxglove Bridge + UI)

**Файл:** `server/start.sh`

**Компоненты:**
- Foxglove Bridge для отладки и визуализации
- Консоль управления с командами:
  - `restart <slam|nav2|opencv|bridge>` — перезапуск сервиса
  - `exit` — остановка всех процессов

---

## Развёртывание

### Сборка и запуск на Raspberry Pi

```bash
# Обновление проекта
make update

# Запуск Docker-контейнера с ROS 2 нодами
make start-rpi   # docker compose up -d

# Контейнер монтирует:
# - ./data:/data
# - /dev/sensors:/dev/sensors (Arduino + LiDAR)
# - /dev/ttyUSB0, /dev/ttyUSB1 (камера/другие устройства)
```

### Прошивка Arduino

```bash
make arduino   # pio run --target upload --upload-port /dev/sensors/arduino
```

### Запуск серверной части (отладка/UPC)

```bash
make start-server   # cd ./server && ./start.sh
```

---

## Топики ROS 2

| Топик | Тип | Описание |
|-------|-----|----------|
| `/scan` | LaserScan | Данные лидара LD19 |
| `/camera/image_raw` | Image | Сырые кадры с камеры |
| `/camera/image_processed` | Image | Кадры с детекцией объектов |
| `/cmd_vel` | Twist | Команды движения (linear.x, angular.z) |
| `/cmd_vel_head` | Twist | Команды камеры (linear.x=pitch, angular.z=yaw) |
| `/odom` | Odometry | Одометрия по энкодерам |
| `/robot/encoder_left` | Int64 | Левый энкодер (тики) |
| `/robot/encoder_right` | Int64 | Правый энкодер (тики) |
| `/robot/distance_left` | Int32 | Левый ToF-датчик (мм) |
| `/robot/distance_right` | Int32 | Правый ToF-датчик (мм) |
| `/robot/button_status` | Bool | Статус кнопки |
| `/robot/detected_objects` | Marker | Визуализация обнаруженных объектов |

---

## TF-дерево

```
odom
  └── base_link
      ├── body_link
      ├── laser_link
      ├── camera_link
      ├── left_wheel_link
      └── right_wheel_link
```

---

## Переменные окружения

- `ROS_DOMAIN_ID=123` — изоляция ROS-сети
- `OPENCV_FFMPEG_CAPTURE_OPTIONS` — оптимизация захвата видео

---

## Ссылки

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [NAV2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [PlatformIO](https://platformio.org/)
