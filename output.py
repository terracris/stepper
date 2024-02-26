import Jetson.GPIO as GPIO

# set the GPIO mode to BCM
GPIO.setmode(GPIO.BOARD)

pin_1 = 32
pin_2 = 18

# set up GPIO pin 8, 10 as output
GPIO.setup(pin_1, GPIO.OUT)
# GPIO.setup(pin_2, GPIO.OUT)

GPIO.output(pin_1, GPIO.HIGH)
# GPIO.output(pin_2, GPIO.HIGH)

while True:
    pass
