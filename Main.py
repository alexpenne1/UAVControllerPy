# Main.py
# Ethan LoCicero and Alex Penne
# Run on Raspberry Pi to connect to 9DOF BNO055 sensor and quadcopter motors.
# Dependencies: BNOSensor.py, ESC.py

# Import needed libraries.
import numpy as np
import logging
import sys
import time
import subprocess
import csv
import RPi.GPIO as GPIO
import pigpio as pigpio
from Adafruit_BNO055 import BNO055
# Import functions from other files.
# BNOSensor.py files.
from BNOSensor import connectSensor
from BNOSensor import callibrateSensor
from BNOSensor import getStates
# ESC.py files.
from ESC import connectMotors # Don't use this one for now. Software pwm. 
from ESC import connectMotorsPigpio
# Vicon.py files
from Vicon import connectVicon
from Vicon import GetLinearStates
# Controller.py files
from Controller import EstimateRates
from Controller import CalculateControlAction_LQR
#from Controller import CalculateControlAction_TestFeedback



maxval = 1900
minval = 1100
# CONNECT AND CALLIBRATE
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
# UART mode must be turned on (PS1 pin = HIGH).
bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
# Connect the sensor.
connectSensor(bno)
print("Sensor connected!")
# Callibrate the sensor.
#callibrateSensor(bno)
print("Sensor callibrated!\n\n")
# Connect to Vicon
vicon_client, mytracker = connectVicon("192.168.0.101")
OBJECT_NAME = "LoCicero_RPI_Drone"

# Get Object position
position = mytracker.get_position(OBJECT_NAME) # (latency, frame number, [[object_name,object_name,x,y,z,roll,pitch,yaw]]) (mm, rad)
print(f"Position: {position}")



# Set pin numbers and connect the motors.
pins = [24, 26, 17, 16] # using GPIO.BCM numbering
mypi = connectMotorsPigpio(pins)
print("Motors connected and callibrated!")


# START PROGRAM
print("Starting program. To kill drone, kill the program using Ctrl+C.")
# Turn on motors.


# get initial lineaer position from VICON.
init_x, init_y, init_z = GetLinearStates(mytracker, OBJECT_NAME)
# get initial orientation from BNO
init_yaw, init_roll, init_pitch, init_dyaw, init_droll, init_dpitch, init_a_x, init_a_y, init_a_z = getStates(bno) 
# Set setpoint.
target_height = .3 # .3m = 1ft
setpoint = np.transpose(np.array([[init_x, init_y, init_z+target_height, init_roll, init_pitch, init_yaw, 0, 0, 0, 0, 0, 0]]))
# Get initial time.
init_time = time.time()
# Make previous state vector.
prev_state = [init_x, init_y, init_z, init_time]
# Initialize Vicon filters
xfilterstate = 0
yfilterstate = 0
zfilterstate = 0
dxfilterstate = 0
dyfilterstate = 0
dzfilterstate = 0
# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            # Get x, y, z from VICON.
            x, y, z = GetLinearStates(mytracker, OBJECT_NAME)
            # Get current time.
            cur_time = time.time()
            # Calculate latency
            dt = cur_time - prev_state[3]
            # Filter Position
            x, y, z, fstatex, fstatey, fstatez = FilterViconPosition(x, y, z, dt, fstatex, fstatey, fstatez)
            # Estimate rates.
            dxdt, dydt, dzdt = EstimateRates(x, y, z, dt, prev_state)
            # Filter Rates
            dxdt, dydt, dzdt, fstatedx, fstatedy, fstatedz = FilterViconRates(dxdt, dydt, dzdt, dt, fstatedx, fstatedy, fstatedz)
            # Get attitude and rates from sensor.
            yaw, pitch, roll, dyaw, dpitch, droll, a_x, a_y, a_z = getStates(bno)
            # Make state vector.
            state =np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
            # Get input from state.
            inputs = CalculateControlAction_LQR(state,setpoint)
            # Change motor speeds.
            for i in range(0,4):
                mypi.set_servo_pulsewidth(pins[i], inputs[i])
            # write time, states, and inputs to a csv file
            save_vec = np.transpose(np.concatenate((np.array([[cur_time]]), state, inputs),axis=0))
            np.savetxt(myfile, save_vec, delimiter=',', fmt='%f')
            # Make current state the previous.                
            prev_state = [x, y, z, cur_time]
        
    except KeyboardInterrupt: # This should allow us to exit the while loop by pressing Ctrl+C
        pass
    
    
# Sets drone to zero speed at end of program.
for pin in pins:
    mypi.set_servo_pulsewidth(pin, 1100)







### EXAMPLE CODES. Remove triple quotations to run. 

# Turn on max speed and constantly ask for next speed.
"""
for pin in pins:
    mypi.set_servo_pulsewidth(pin, maxval)
while True:
    speed = input("enter speed: ")
    for pin in pins:
        mypi.set_servo_pulsewidth(pin, speed)
"""
# Print yaw.
"""
while True:
    yaw, roll, pitch, w_x, w_y, w_z, a_x, a_y, a_z = getStates(bno)
    print(yaw)
"""







