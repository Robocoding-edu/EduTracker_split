#include <Arduino.h>
#include <Servo.h> // Подключаем стандартную библиотеку сервоприводов
#include <VL53L0X.h>
#include <Wire.h>

#define SERIAL_BAUD 115200

// ================= VL53L0X =================

#define SENSOR_COUNT 4

const uint8_t XSHUT_PINS[SENSOR_COUNT] = {
  23, // RIGHT
  25, // FRONT
  24, // LEFT
  22  // REAR
};

const uint8_t SENSOR_ADDR[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33};

VL53L0X sensors[SENSOR_COUNT];

uint16_t distances[SENSOR_COUNT];

const uint16_t STOP_DISTANCE_FRONT = 250; // мм
const uint16_t STOP_DISTANCE_SIDE = 150;
const uint16_t STOP_DISTANCE_REAR = 200;

unsigned long lastSensorUpdate = 0;
const unsigned long sensorInterval = 80;


constexpr int MIN_SPEED = 30;

float currentLinear = 0;
float currentAngular = 0;

// --- КОНФИГУРАЦИЯ ПИНОВ МОТОРОВ КОЛЕС (Уже настроено) ---
const int pinPWMA = 8;
const int pinAIN2 = 7;
const int pinAIN1 = 6;
const int pinBIN1 = 9;
const int pinBIN2 = 10;
const int pinPWMB = 11;

// --- КОНФИГУРАЦИЯ ПИНОВ ЭНКОДЕРОВ (ПО СХЕМЕ) ---
const int pinEncLeftA = 3;   // J8 пин 3
const int pinEncLeftB = 15;  // J8 пин 4
const int pinEncRightA = 2;  // J1 пин 3
const int pinEncRightB = 14; // J1 пин 4

// --- КОНФИГУРАЦИЯ СЕРВОПРИВОДОВ КАМЕРЫ ---
const int pinServoLeft = 5;   // Левая серва дифференциала
const int pinServoRight = 13; // Правая серва дифференциала

Servo servoL;
Servo servoR;


const float V_MIN = 0.035;     // м/с, минимальная реальная скорость
const float V_MAX = 0.340;     // м/с, максимальная реальная скорость
const float WHEEL_SEP = 0.231; // м, из serial_bridge.py
const int PWM_MIN = 30;        // минимальный PWM, при котором моторы крутятся
const int PWM_MAX = 255;
const float MAX_OMEGA = 1.2;   // рад/с, безопасный максимум поворота

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Текущие углы сервоприводов (изначально выставляем в центр — 90 градусов)
int currentPitch = 90; // Наклон головы
int currentYaw = 90;   // Поворот головы

// Настройки телеметрии (как было)
long encoderLeft = 0;
long encoderRight = 0;
int distanceLeft = 45;
int distanceRight = 60;
int buttonStatus = 0;
unsigned long previousMillis = 0;
unsigned long tmr_m = 0;
const long interval = 100;
String inputBuffer = "";

long lastSentLeft = 0;
long lastSentRight = 0;
int lastSentButton = -1;

unsigned long lastCommandTime = 0;
const unsigned long connectionTimeout = 1500;


int velocityToPwm(float v) {
  float av = fabs(v);

  // Мертвая зона, чтобы около нуля моторы не дергались
  if (av < 0.01) {
    return 0;
  }

  if (av < V_MIN) av = V_MIN;
  if (av > V_MAX) av = V_MAX;

  float ratio = (av - V_MIN) / (V_MAX - V_MIN);
  int pwm = PWM_MIN + (int)(ratio * (PWM_MAX - PWM_MIN));

  if (v < 0) pwm = -pwm;
  pwm = constrain(pwm, MIN_SPEED, 255);
  return pwm;
}

void initVL53L0X() {
  Wire.begin();
  Wire.setClock(400000);

  // выключаем все
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }

  delay(100);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(50);

    if (!sensors[i].init()) {
      Serial.print("VL53 FAILED ");
      Serial.println(i);
      continue;
    }

    sensors[i].setAddress(SENSOR_ADDR[i]);
    sensors[i].setTimeout(100);

    sensors[i].setMeasurementTimingBudget(30000);

    Serial.print("VL53 OK ");
    Serial.println(i);
  }
}

void isrLeft() {
  // Если фазы одинаковые — крутимся в одну сторону, если разные — в другую
  if (digitalRead(pinEncLeftA) == digitalRead(pinEncLeftB)) {
    encoderLeft--; // Меняй знак ++ или -- если одометрия поедет назад
  } else {
    encoderLeft++;
  }
}

void isrRight() {
  if (digitalRead(pinEncRightA) == digitalRead(pinEncRightB)) {
    encoderRight++; // Меняй знак если одометрия поедет назад
  } else {
    encoderRight--;
  }
}

int leftMotorSpeedT = 0;
int rightMotorSpeedT = 0;

void setMotor(int motorNum, int speed);
void stopMotors();
void parseCommand(String cmd);

void setup() {
  Serial.begin(SERIAL_BAUD);
  initVL53L0X();
  inputBuffer.reserve(64);

  // Инициализация колес
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  stopMotors();

  // Инициализация и привязка сервоприводов камеры
  servoL.attach(pinServoLeft);
  servoR.attach(pinServoRight);

  // Устанавливаем камеру ровно по центру при старте
  servoL.write(currentPitch);
  servoR.write(currentYaw);

  // ИНСТРУКЦИЯ НА СХЕМУ: Настройка пинов энкодеров на вход с подтяжкой
  pinMode(pinEncLeftA, INPUT_PULLUP);
  pinMode(pinEncLeftB, INPUT_PULLUP);
  pinMode(pinEncRightA, INPUT_PULLUP);
  pinMode(pinEncRightB, INPUT_PULLUP);

  // Привязка аппаратных прерываний по изменению фронта (RISING)
  attachInterrupt(digitalPinToInterrupt(pinEncLeftA), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncRightA), isrRight, RISING);
}

int sign(int x)
{
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}


int updateSpeed(int current, int target)
{
  int direction = sign(target);

  // Остановка
  if (target == 0) {
    if (abs(current) > MIN_SPEED)
      return smoothStep(current, direction * MIN_SPEED);

    return 0;
  }

  // Старт из остановки
  if (current == 0)
    return direction * MIN_SPEED;

  // Если надо поменять направление
  // сначала останавливаемся
  if (sign(current) != direction) {
    return smoothStep(current, 0);
  }

  // Обычное изменение скорости
  return smoothStep(current, target);
}


int smoothStep(int current, int target)
{
  int diff = target - current;

  if (abs(diff) <= 1)
    return target;

  int step = diff / 4;

  if (step == 0)
    step = diff > 0 ? 1 : -1;

  return current + step;
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - tmr_m > 1) {
    tmr_m = currentMillis;

    leftMotorSpeedT = updateSpeed(leftMotorSpeedT, leftMotorSpeed);
    rightMotorSpeedT = updateSpeed(rightMotorSpeedT, rightMotorSpeed);

    setMotor(1, leftMotorSpeedT);
    setMotor(2, rightMotorSpeedT);
  }



  if (currentMillis - lastSensorUpdate >= sensorInterval) {
    lastSensorUpdate = currentMillis;
    updateVL53();

    if (emergencyStop()) {
      stopMotors();
    }
  }
  if (millis() - lastCommandTime > connectionTimeout) {
    stopMotors();
  }
  // Отправка данных в ROS 2 (Каждые 100 мс)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    lastSentLeft = encoderLeft;
    lastSentRight = encoderRight;
    lastSentButton = buttonStatus;

    Serial.print("DAT:");
    Serial.print(encoderLeft);
    Serial.print(",");
    Serial.print(encoderRight);
    Serial.print(",");
    Serial.print(distanceLeft);
    Serial.print(",");
    Serial.print(distanceRight);
    Serial.print(",");
    Serial.println(buttonStatus);
  }

  // Прием команд от Малинки
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      parseCommand(inputBuffer);
      inputBuffer = "";
    } else if (inChar != '\r') {
      inputBuffer += inChar;
    }
  }
}

void parseCommand(String cmd) {
  lastCommandTime = millis();
  // НОВЫЙ БЛОК: Дифференциальное управление камерой (#HEAD:наклон,поворот)
  if (cmd.startsWith("#HEAD:")) {
    String valStr = cmd.substring(6);
    int commaIndex = valStr.indexOf(',');
    if (commaIndex != -1) {
      String pitchStr = valStr.substring(0, commaIndex);
      String yawStr = valStr.substring(commaIndex + 1);

      int deltaPitch = pitchStr.toInt(); // Смещение наклона от пульта
      int deltaYaw = yawStr.toInt();     // Смещение поворота от пульта

      // Изменяем текущую виртуальную позицию головы робота
      // (добавляем смещение, чтобы камера двигалась плавно, пока зажата кнопка)
      currentPitch +=
        deltaPitch / 5; // Деление на 5 уменьшает резкость движения
      currentYaw += deltaYaw / 5;

      // Ограничиваем виртуальные углы в безопасные физические лимиты (от 20 до
      // 160 градусов)
      currentPitch = constrain(currentPitch, 20, 160);
      currentYaw = constrain(currentYaw, 20, 160);

      // --- ФОРМУЛА ДИФФЕРЕНЦИАЛА ДЛЯ СЕРВОПРИВОДОВ ---
      // Смешиваем наклон и поворот для левой и правой сервы
      int servoLAngle = currentPitch + (currentYaw - 90);
      int servoRAngle = currentPitch - (currentYaw - 90);

      // Зажимаем итоговые углы в строгие лимиты сервоприводов (0-180)
      servoLAngle = constrain(servoLAngle, 0, 180);
      servoRAngle = constrain(servoRAngle, 0, 180);

      // Отправляем физические углы на исполнительные механизмы d5 и d13
      servoL.write(servoLAngle);
      servoR.write(servoRAngle);

      // Эхо-ответ для проверки
      Serial.print("ACK:HEAD_SERVO_ANGLES:");
      Serial.print(servoLAngle);
      Serial.print(",");
      Serial.println(servoRAngle);
    }
  }

  // Управление колесами (Оставляем как было)
  else if (cmd.startsWith("#MOVE:")) {
    String valStr = cmd.substring(6);
    int commaIndex = valStr.indexOf(',');

    if (commaIndex != -1) {
      float linearX = valStr.substring(0, commaIndex).toFloat();
      float angularZ = valStr.substring(commaIndex + 1).toFloat();

      // Ограничиваем команды
      linearX = constrain(linearX, -V_MAX, V_MAX);
      angularZ = constrain(angularZ, -MAX_OMEGA, MAX_OMEGA);

      currentLinear = linearX;
      currentAngular = angularZ;

      // Дифференциальный привод:
      // positive angular.z в ROS = поворот влево
      float leftVel  = linearX - angularZ * WHEEL_SEP / 2.0;
      float rightVel = linearX + angularZ * WHEEL_SEP / 2.0;

      // Если одно колесо выходит за максимум, пропорционально режем обе скорости
      float maxWheel = max(fabs(leftVel), fabs(rightVel));
      if (maxWheel > V_MAX) {
        float k = V_MAX / maxWheel;
        leftVel *= k;
        rightVel *= k;
      }

      leftMotorSpeed = velocityToPwm(leftVel);
      rightMotorSpeed = velocityToPwm(rightVel);

      Serial.print("ACK:MOTORS_PWM:");
      Serial.print(leftMotorSpeed);
      Serial.print(",");
      Serial.println(rightMotorSpeed);
    }
  }
}


// Функции моторов (как были)
void setMotor(int motorNum, int speed) {
  boolean in1State = LOW;
  boolean in2State = LOW;
  if (speed > 0) {
    in1State = HIGH;
    in2State = LOW;
  } else if (speed < 0) {
    in1State = LOW;
    in2State = HIGH;
    speed = -speed;
  }
  if (motorNum == 1) {
    digitalWrite(pinAIN1, in1State);
    digitalWrite(pinAIN2, in2State);
    analogWrite(pinPWMA, speed);
  } else if (motorNum == 2) {
    digitalWrite(pinBIN1, in1State);
    digitalWrite(pinBIN2, in2State);
    analogWrite(pinPWMB, speed);
  }
}
void stopMotors() {
  setMotor(1, 0);
  setMotor(2, 0);
}

void updateVL53() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    uint16_t d = sensors[i].readRangeSingleMillimeters();

    if (sensors[i].timeoutOccurred() || d > 2000)
      distances[i] = 9999;
    else
      distances[i] = d;
  }
}

bool obstacleDetected() {
  // bool movingForward = false;

  // если есть команда движения вперёд
  // можно сделать через переменную
  return false;
}

bool emergencyStop() {
  // вперёд
  if (currentLinear > 0.05) {
    if (distances[1] < STOP_DISTANCE_FRONT)
      return true;
  }
  // назад
  if (currentLinear < -0.05) {
    if (distances[3] < STOP_DISTANCE_REAR)
      return true;
  }
  // вращение на месте
  if (abs(currentAngular) > 0.1) {
    if (distances[0] < STOP_DISTANCE_SIDE || distances[2] < STOP_DISTANCE_SIDE)
      return true;
  }
  return false;
}
