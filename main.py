import RPi.GPIO as GPIO #Motor/Buttons
import time
import serial #LCD

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

#                                                               BUTTONS
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#                                                               STEPPER MOTOR
motorGpioPins = [17, 18, 27, 22] #17 = 1 etc.
for pin in motorGpioPins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

#http://www.bitsbox.co.uk/data/motor/Stepper.pdf - Each step is a list containing GPIO pins that should be set to High
motorStepSequence = list(range(0, 4))
motorStepSequence[0] = [1,0,0,0]
motorStepSequence[1] = [0,1,0,0]
motorStepSequence[2] = [0,0,1,0]
motorStepSequence[3] = [0,0,0,1]
motorStepCount = 0

#                                                                  LCD
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
ser.write(('testtesttest$').encode('utf-8'))
time.sleep(5)
ser.write(('Hi$').encode('utf-8'))

def motor(anticlockwise): #Run the motor for an "instant"
    global motorGpioPins, motorStepSequence, motorStepCount
    if (anticlockwise):
        motorStepSequence.reverse()
    for pin in range(0, 4):
        xpin = motorGpioPins[pin]
        if motorStepSequence[motorStepCount][pin] != 0:
            GPIO.output(xpin, True)
        else:
            GPIO.output(xpin, False)
        motorStepCount += 1

    #Reset the counter if we get to end
    if (motorStepCount == 4):
        motorStepCount = 0
    if (motorStepCount < 0):
        motorStepCount = 4


    if (anticlockwise):
        motorStepSequence.reverse() #Put it back!

    time.sleep(0.0015)

while True:
    input_state = GPIO.input(23)
    if input_state == False:
        print('Button Pressed')
        motor(False)
