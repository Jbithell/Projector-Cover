#!/usr/bin/python
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

StepPins = [18,23,24,25]

# Set all pins as output
for pin in StepPins:
    print
    "Setup pins"
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Define advanced sequence
# as shown in manufacturers datasheet
Seq = [[1, 0, 0, 1],
       [1, 0, 0, 0],
       [1, 1, 0, 0],
       [0, 1, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 1, 0],
       [0, 0, 1, 1],
       [0, 0, 0, 1]]

StepCount = len(Seq)
StepDir = 2  # Set to 1 or 2 for clockwise
# Set to -1 or -2 for anti-clockwise

# Read wait time from command line
WaitTime = 10 / float(1000)

# Initialise variables
StepCounter = 0

# Start main loop
while True:

    print
    StepCounter,
    print
    Seq[StepCounter]

    for pin in range(0, 4):
        xpin = StepPins[pin]  # Get GPIO
        if Seq[StepCounter][pin] != 0:
            print
            " Enable GPIO %i" % (xpin)
            GPIO.output(xpin, True)
        else:
            GPIO.output(xpin, False)

    StepCounter += StepDir

    # If we reach the end of the sequence
    if (StepCounter >= StepCount):
        StepCounter = 0
    if (StepCounter < 0):
        StepCounter = StepCount + StepDir

    # Wait before moving on
    time.sleep(WaitTime)
