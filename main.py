import RPi.GPIO as GPIO
import time

#                                                               BUTTONS
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#                                                               STEPPER MOTOR
motorGpioPins = [17, 18, 27, 22] #17 = 1 etc.
for pin in motorGpioPins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

#http://www.bitsbox.co.uk/data/motor/Stepper.pdf - Each step is a list containing GPIO pins that should be set to High
motorStepSequence = list(range(0, 8))
motorStepSequence[0] = [motorGpioPins[0]]
motorStepSequence[1] = [motorGpioPins[0], motorGpioPins[1]]
motorStepSequence[2] = [motorGpioPins[1]]
motorStepSequence[3] = [motorGpioPins[1], motorGpioPins[2]]
motorStepSequence[4] = [motorGpioPins[2]]
motorStepSequence[5] = [motorGpioPins[2], motorGpioPins[3]]
motorStepSequence[6] = [motorGpioPins[3]]
motorStepSequence[7] = [motorGpioPins[3], motorGpioPins[0]]

#                                                                  LCD
ser = serial.Serial('/dev/tty.usbserial', 9600, timeout=0.5)
ser.write('testtesttest$$')
time.sleep(5)
ser.write('Hi$')

def motor(anticlockwise): #Run the motor for an "instant"
    global motorGpioPins, motorStepSequence
    if (anticlockwise):
        motorStepSequence.reverse()
    for pinList in motorStepSequence:
        for pin in motorGpioPins:
            if pin in pinList:
                GPIO.output(pin, True)
            else:
                GPIO.output(pin, False)
            time.sleep(0.0001)
    if (anticlockwise):
        motorStepSequence.reverse() #Put it back!

while True:
    input_state = GPIO.input(23)
    if input_state == False:
        print('Button Pressed')
        motor(False)
