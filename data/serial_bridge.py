#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int64, Bool
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
import serial
import threading
import time
import math
import os
from tf2_ros import TransformBroadcaster

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.last_vx = 0.0
        self.last_vth = 0.0

        self.odom_timer = self.create_timer(
            0.1,
            self.publish_odom_timer
        )

        # --- ФИЗИЧЕСКИЕ ПАРАМЕТРЫ РОБОТА ---
        self.wheel_diameter = 0.065  # 65 мм в метрах
        self.wheel_separation = 0.195 # 195 мм в метрах
        self.ticks_per_rev = 77.0     # 77 тиков на оборот

        self.last_cmd = None
        self.last_cmd_time = time.time()
        self.cmd_repeat_interval = 0.5

        # Метров на один тик энкодера
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # Переменные одометрии (позиция робота в пространстве)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Хранение предыдущих значений тиков для вычисления дельты
        self.prev_enc_left = None
        self.prev_enc_right = None
        self.last_time = self.get_clock().now()

        # Настройки Serial-порта
        self.port_name = '/dev/sensors/arduino'
        self.baud_rate = 115200

        # Издатели (Publishers)
        self.enc_left_pub = self.create_publisher(Int64, '/robot/encoder_left', 10)
        self.enc_right_pub = self.create_publisher(Int64, '/robot/encoder_right', 10)
        self.dist_left_pub = self.create_publisher(Int32, '/robot/distance_left', 10)
        self.dist_right_pub = self.create_publisher(Int32, '/robot/distance_right', 10)
        self.button_pub = self.create_publisher(Bool, '/robot/button_status', 10)

        # ИЗДАТЕЛЬ ОДОМЕТРИИ (Важно для SLAM)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Подписчики (Subscribers)
        self.servo_sub = self.create_subscription(Twist, '/cmd_vel_head', self.head_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Подключение к порту с фиксом CH340
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port_name
            self.ser.baudrate = self.baud_rate
            self.ser.timeout = 1.0
            self.ser.rtscts = False
            self.ser.dsrdtr = False
            self.ser.open()
            time.sleep(2.0)
            self.ser.flushInput()
            self.ser.flushOutput()
            self.get_logger().info(f"Успешное подключение к Atmega2560 на порту {self.port_name}!")
        except Exception as e:
            self.get_logger().error(f"Не удалось открыть порт {self.port_name}: {e}")
            raise e

        # Поток чтения
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()

        self.cmd_timer = self.create_timer(
            self.cmd_repeat_interval,
            self.repeat_last_cmd
        )

    def repeat_last_cmd(self):
        if self.last_cmd is None:
            return

        try:
            linear_x, angular_z = self.last_cmd

            self.ser.write(
                f"#MOVE:{linear_x},{angular_z}\n".encode()
            )

        except Exception:
            pass

    def _read_serial_loop(self):
        buffer = ""
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line.startswith("DAT:"):
                            self._parse_and_publish(line[4:])
                        elif line.startswith("ACK:"):
                            self.get_logger().debug(line)
                else:
                    time.sleep(0.005)

            except Exception as e:
                time.sleep(1.0)

    def _parse_and_publish(self, data_str):
        try:
            parts = data_str.split(',')
            if len(parts) < 5: return

            # ИСПРАВЛЕНО: Инвертируем знак (-), чтобы движение вперед давало ПЛЮС
            enc_L = -int(parts[0])
            enc_R = -int(parts[1]) # Правое колесо проверь, если едет назад - тоже добавь минус

            dist_L = int(parts[2])
            dist_R = int(parts[3])
            btn = bool(int(parts[4]))

            # Публикуем сырые топики датчиков
            self.enc_left_pub.publish(Int64(data=enc_L))
            self.enc_right_pub.publish(Int64(data=enc_R))
            self.dist_left_pub.publish(Int32(data=dist_L))
            self.dist_right_pub.publish(Int32(data=dist_R))
            self.button_pub.publish(Bool(data=btn))

            # --- РАСЧЕТ КОЛЕСНОЙ ОДОМЕТРИИ ---
            current_time = self.get_clock().now()

            if self.prev_enc_left is not None and self.prev_enc_right is not None:
                # Сколько тиков проехало каждое колесо с прошлого пакета
                d_left = enc_L - self.prev_enc_left
                d_right = enc_R - self.prev_enc_right

                # Переводим тики в реальные метры
                dist_left_meters = d_left * self.meters_per_tick
                dist_right_meters = d_right * self.meters_per_tick

                # Средний путь робота и угол поворота
                d_center = (dist_left_meters + dist_right_meters) / 2.0
                d_theta = (dist_right_meters - dist_left_meters) / self.wheel_separation

                # Вычисляем дельту координат X и Y по тригонометрии
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt > 0:
                    v_x = d_center / dt
                    v_th = d_theta / dt
                else:
                    v_x = v_th = 0.0

                self.last_vx = v_x
                self.last_vth = v_th

                # Обновляем абсолютные координаты робота на карте
                self.x += d_center * math.cos(self.th)
                self.y += d_center * math.sin(self.th)
                self.th += d_theta

                # Публикуем одометрию в систему ROS
                self._publish_odom_data(current_time, v_x, v_th)

            self.prev_enc_left = enc_L
            self.prev_enc_right = enc_R
            self.last_time = current_time

        except (ValueError, IndexError):
            pass

    def publish_odom_timer(self):

        current_time = self.get_clock().now()

        self._publish_odom_data(
            current_time,
            0.0,
            0.0
        )

    def _publish_odom_data(self, current_time, v_x, v_th):
        # Переводим угол Эйлера (Theta) в кватернион вращения ROS
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.th / 2.0)
        q.w = math.cos(self.th / 2.0)

        # 1. Публикуем TF Трансформацию (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2. Публикуем сообщение Одометрии
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.angular.z = v_th
        self.odom_pub.publish(odom)

    def head_callback(self, msg):
        pitch = int(msg.linear.x * 50)
        yaw = int(msg.angular.z * 50)
        try: self.ser.write(f"#HEAD:{pitch},{yaw}\n".encode('utf-8'))
        except: pass

    def cmd_vel_callback(self,msg):

        linear_x = round(msg.linear.x,2)
        angular_z = round(msg.angular.z,2)

        cmd = (linear_x, angular_z)

        self.last_cmd = cmd

        try:
            self.ser.write(
                f"#MOVE:{linear_x},{angular_z}\n".encode()
            )

        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
