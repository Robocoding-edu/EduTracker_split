#pragma once
#include <Arduino.h>
// --- КОНФИГУРАЦИЯ ПИНОВ МОТОРОВ КОЛЕС (Уже настроено) ---
constexpr int pinPWMA = 8;
constexpr int pinAIN2 = 7;
constexpr int pinAIN1 = 6;
constexpr int pinBIN1 = 9;
constexpr int pinBIN2 = 10;
constexpr int pinPWMB = 11;
void setMotor(int motorNum, int speed);
void stopMotors();
