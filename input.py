import Jetson.GPIO as GPIO
import time

# Set the GPIO mode to BCM numbering
GPIO.setmode(GPIO.BOARD)

input_1 = 15
input_2 = 23
input_3 = 33
input_4 = 40

# Set up GPIO pin 18 as an input
GPIO.setup(input_4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        # Read the state of the GPIO pin
        input_state = GPIO.input(input_4)
        print("GPIO pin ", input_4, " state: ", input_state)
        time.sleep(0.1)  # Wait for a short time to avoid flooding the console

except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()

