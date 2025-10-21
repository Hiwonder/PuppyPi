#!/usr/bin/python3
# coding=utf8
import Jetson.GPIO as GPIO

KEY1_PIN = 25  # key1对应引脚号(The pin number corresponding to key1)
KEY2_PIN = 4   # key2对应引脚号(The pin number corresponding to key2)

mode = GPIO.getmode()
if mode == 1 or mode is None:  # 是否已经设置引脚编码(whether the pin number is set)
    GPIO.setmode(GPIO.BCM)  # 设为BCM编码(set as BCM code)

GPIO.setwarnings(False)  # 关闭警告打印(close the alarm print)

GPIO.setup(KEY1_PIN, GPIO.IN)  # 设置引脚为输入模式(set the pin as input)
GPIO.setup(KEY2_PIN, GPIO.IN)  # 设置引脚为输入模式(set the pin as input)

key_dict = {"key1": KEY1_PIN,
            "key2": KEY2_PIN}

def get_button_status(key):
    if key in key_dict:
        return GPIO.input(key_dict[key])
    else:
        return None

if __name__ == "__main__":
    import time
    while True:
        print(get_button_status('key2'))
        time.sleep(0.01)
