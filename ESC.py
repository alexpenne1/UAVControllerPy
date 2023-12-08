import logging
import sys
import time
import RPi.GPIO as GPIO 

from Adafruit_BNO055 import BNO055

def connectMotors(pins):
    # Setup and connect motors.
    print("Connecting motors...")
    # Pins are as set on BOARD not by GPIO ordering.

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    # Set frequencies.
    motors = [GPIO.PWM(pins[0], 50), GPIO.PWM(pins[1], 50), GPIO.PWM(pins[2], 50), GPIO.PWM(pins[3], 50)]
    # Disconnect battery.
    input("Check that battery is disconnected from the base, then press enter.")
    # Set lowest PWM signal (4 for some reason).
    for motor in motors:
        motor.start(4)

    input("Connect the battery. Press enter when done.")
    print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
    print("End arming sequence.\n\n")

    return motors
    
    




