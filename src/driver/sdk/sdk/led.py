#!/usr/bin/python3
# coding=utf8
import Jetson.GPIO as GPIO

LED_PIN = 24  # LED对应引脚号 (pin number corresponding to LED)

mode = GPIO.getmode()
if mode == 1 or mode is None:  # 是否已经设置引脚编码(whether the pin code is set)
    GPIO.setmode(GPIO.BCM)  # 设为BCM编码(set as BCM code)

GPIO.setwarnings(False)  # 关闭警告打印(close alarm print)

GPIO.setup(LED_PIN, GPIO.OUT)  # 设置引脚为输出模式(set pin as output mode)

def on():
    GPIO.output(LED_PIN, 0)

def off():
    GPIO.output(LED_PIN, 1)

def set(new_state):
    GPIO.output(LED_PIN, new_state)

if __name__ == "__main__":
    import time
    while True:
        on()
        time.sleep(1)
        off()
        time.sleep(1)
