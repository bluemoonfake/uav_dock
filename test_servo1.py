
import RPi.GPIO as GPIO
import threading
import time
import os
import sys
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from gpiozero import DigitalInputDevice, Servo
from time import sleep

DIR_PIN1 = 20
ENA_PIN1 = 21
STEP_PIN1 = 19

DIR_PIN2 = 24
ENA_PIN2 = 25
STEP_PIN2 = 13

dir_ctrl1 = DigitalOutputDevice(DIR_PIN1)
ena_ctrl1 = DigitalOutputDevice(ENA_PIN1)
step_ctrl1 = PWMOutputDevice(STEP_PIN1, frequency=700)

dir_ctrl2 = DigitalOutputDevice(DIR_PIN2)
ena_ctrl2 = DigitalOutputDevice(ENA_PIN2)
step_ctrl2 = PWMOutputDevice(STEP_PIN2, frequency=5000)

right = 1

def start_motor():
    global right

    if right == 0:
        dir_ctrl1.on()
        step_ctrl1.value = 0.5
        right = 1
    else:
        dir_ctrl1.off()
        step_ctrl1.value = 0.5
        right = 0

def keyboard_thread():
    while True:
        print("Enter to start")
        input()
        start_motor()

threading.Thread(target=keyboard_thread, daemon=True).start()

try:
    ena_ctrl1.off()

    while True:
        sleep(0.2)
        print("right: ", right)
        
finally:
    step_ctrl1.value = 0.0
    ena_ctrl1.on()