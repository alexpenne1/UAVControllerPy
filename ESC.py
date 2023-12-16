import logging
import sys
import time
import RPi.GPIO as GPIO
import pigpio as pigpio
import subprocess
from Adafruit_BNO055 import BNO055

def init():
    # Set pin numbers and connect the motors.
    pins = [17, 24, 16, 19] # using GPIO.BCM numbering
    mypi = connectMotorsPigpio(pins)
    print("Motors connected and callibrated!")
    return pins, mypi

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
    for pin in pins[0:2]:
        print(pin)
        mypi.set_servo_pulsewidth(pin, minval) # yellow ESCs can arm from minval
    print(pins[3])
    mypi.set_servo_pulsewidth(pins[3],maxval) # black ESCs need maxval before plugged in to arm correctly
        
    input("Connect the battery. Press enter when done.")
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, minval)
    print("4 beeps indicates drone is armed. Otherwise, needs callibration.")
    print("End arming sequence.\n\n")
    print("Starting program. To kill drone, kill the program using Ctrl+C.")
    time.sleep(1)
    return mypi

def writeMotors(mypi,pins,inputs):
    for i in range(0,4):
                mypi.set_servo_pulsewidth(pins[i], inputs[i])
    return

def StopMotors(mypi,pins):
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, 1100)
    print("Motors Stopped.")