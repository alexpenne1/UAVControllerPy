import logging
import sys
import time
import RPi.GPIO as GPIO
import pigpio as pigpio
import subprocess
from Adafruit_BNO055 import BNO055

def connectMotors(pins):
    # Setup and connect motors.
    print("Connecting motors...")
    # Pins are as set on BOARD not by GPIO ordering.

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pins[0],GPIO.OUT)
    GPIO.setup(pins[1],GPIO.OUT)
    GPIO.setup(pins[2],GPIO.OUT)
    GPIO.setup(pins[3],GPIO.OUT)

    # Set frequencies.
    motors = [GPIO.PWM(pins[0], 50), GPIO.PWM(pins[1], 50), GPIO.PWM(pins[2], 50), GPIO.PWM(pins[3], 50)]
    #motors = [GPIO.PWM(pins[0], 50)]
    # Disconnect battery.
    input("Check that battery is disconnected from the base, then press enter.")
    # Set lowest PWM signal (4 for some reason).
    for motor in motors:
        motor.start(4)

    input("Connect the battery. Press enter when done.")
    print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
    print("End arming sequence.\n\n")

    return motors
    
def connectMotorsPigpio(pins):
    # Setup and connect motors.
    print("Connecting motors...")
    # Pins are as set on BOARD not by GPIO ordering.
    
    mypi = pigpio.pi()
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, 0)
    
    maxval = 1900
    minval = 1100
    
    input("Check that battery is disconnected from the base, then press enter.")
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, minval)
        
    input("Connect the battery. Press enter when done.")
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, minval)
    print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
    print("End arming sequence.\n\n")
    time.sleep(5)
    return mypi




