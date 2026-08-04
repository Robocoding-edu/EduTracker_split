#!/bin/bash
if [ -z "$1" ]; then
    echo "Ошибка: Укажите путь к вашему .ino файлу!"
    echo "Пример: ./flash_arduino.sh my_code.ino"
    exit 1
fi

# Выдаем права на порт перед прошивкой
sudo chmod 666 /dev/sensors/arduino

# Копируем твой ino файл в рабочую папку компилятора
cp "$1" ~/atmega_project/src/main.ino

# Шьем строго в порт /dev/ttyUSB1
pio run --target upload --upload-port /dev/ttyUSB1
