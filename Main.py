# Main.py
# Ethan LoCicero and Alex Penne
# Run on Raspberry Pi to connect to 9DOF BNO055 sensor and quadcopter motors.
# Dependencies: BNOSensor.py, ESC.py

# Import needed libraries.
import logging
import sys
import time
import RPi.GPIO as GPIO 
from Adafruit_BNO055 import BNO055
# Import functions from other files.
# BNOSensor.py files.
from BNOSensor import connectSensor
from BNOSensor import callibrateSensor
from BNOSensor import getStates
# ESC.py files.
from ESC import connectMotors


# CONNECT AND CALLIBRATE
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
# UART mode must be turned on (PS1 pin = HIGH).
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
# Connect the sensor.
connectSensor(bno)
print("Sensor connected!")
# Callibrate the sensor.
callibrateSensor(bno)
print("Sensor callibrated!\n\n")
# Set pin numbers and connect the motors.
pins = [18, 37, 11, 36]
motors = connectMotors(pins)
print("Motors connected and callibrated!")


# START PROGRAM
print("Starting program. To kill drone, kill the program using Ctrl+C.")
# Turn on motors.
for motor in motors:
    motor.ChangeDutyCycle(5.5)
# Print yaw.
while True:
    yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z = getStates(bno)
    print(yaw)








