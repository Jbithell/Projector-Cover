#!/usr/bin/python
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO
import StepperMotor


GPIO.setmode(GPIO.BOARD)
m = Motor([18,23,24,25])
m.rpm = 5
m.move_to(90)
sleep(1)
m.move_to(0)
sleep(1)
m.mode = 2
m.move_to(90)
sleep(1)
m.move_to(0)
GPIO.cleanup()