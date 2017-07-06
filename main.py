import RPi.GPIO as GPIO
import time

#           BUTTONS
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#           STEPPER MOTOR
motorGpioPins = [17, 18, 27, 22] #17 = 1 etc.
for pin in motorGpioPins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

#http://www.bitsbox.co.uk/data/motor/Stepper.pdf - Each step is a list containing GPIO pins that should be set to High
motorStepSequence = range(0, 8)
motorStepSequence[0] = [GpioPins[0]]
motorStepSequence[1] = [GpioPins[0], GpioPins[1]]
motorStepSequence[2] = [GpioPins[1]]
motorStepSequence[3] = [GpioPins[1], GpioPins[2]]
motorStepSequence[4] = [GpioPins[2]]
motorStepSequence[5] = [GpioPins[2], GpioPins[3]]
motorStepSequence[6] = [GpioPins[3]]
motorStepSequence[7] = [GpioPins[3], GpioPins[0]]

def motor(anticlockwise):
    global motorGpioPins, motorStepSequence
    if (anticlockwise):
        motorStepSequence.reverse()
    for pinList in motorStepSequence:
        for pin in motorGpioPins:
            if pin in pinList:
                GPIO.output(pin, True)
            else:
                GPIO.output(pin, False)
            time.sleep(0.001)
    if (anticlockwise):
        motorStepSequence.reverse() #Put it back!

while True:
    input_state = GPIO.input(18)
    if input_state == False:
        print('Button Pressed')
        motor()
        time.sleep(0.2)
