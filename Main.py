# Main.py
# Ethan LoCicero and Alex Penne

# Import libraries
import numpy as np
import logging
import sys
import time
import subprocess
import csv
import RPi.GPIO as GPIO
import pigpio as pigpio
from Adafruit_BNO055 import BNO055
# Import functions
import Sensors
import BNOSensor as BNO
import Vicon
import ESC
import Controller as ctrl

calibrate = False
bno, mytracker, object_name = Sensors.init(calibrate)

pins, mypi = ESC.init()

# START PROGRAM
print("Starting program. To kill drone, kill the program using Ctrl+C.")
# Turn on motors.

# get initial lineaer position from VICON.
init_x, init_y, init_z = Vicon.GetLinearStates(mytracker, object_name)
# get initial orientation from BNO
init_yaw, init_roll, init_pitch, init_dyaw, init_droll, init_dpitch, init_a_x, init_a_y, init_a_z = BNO.getStates(bno)
# get time
cur_time = time.time() 
# Make initial state vector
state = np.transpose(np.array([[init_x, init_y, init_z, init_roll, init_pitch, init_yaw, 0, 0, 0, init_droll, init_dpitch, init_dyaw]]))
# Set setpoint.
target_height = .3 # .3m = 1ft
setpoint = np.transpose(np.array([[init_x, init_y, init_z+target_height, init_roll, init_pitch, init_yaw, 0, 0, 0, 0, 0, 0]]))
# Initialize Vicon filters
filter_states = np.array([0, 0, 0, 0, 0, 0])
# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            # Get x, y, z from VICON.
            x, y, z = Vicon.GetLinearStates(mytracker, object_name)
            # Get attitude and rates from sensor.
            yaw, pitch, roll, dyaw, dpitch, droll, a_x, a_y, a_z = BNO.getStates(bno)
            # Get current time.
            prev_time = cur_time
            cur_time = time.time()
            dt = cur_time - prev_time
            # Filter Position
            x, y, z, filter_states[0:3] = ctrl.FilterViconPosition(x, y, z, dt, filter_states[0:3])
            # Estimate rates.
            dxdt, dydt, dzdt = ctrl.EstimateRates(x, y, z, dt, state[0:3])
            # Filter Rates
            dxdt, dydt, dzdt, filter_states[3:6] = ctrl.FilterViconRates(dxdt, dydt, dzdt, dt, filter_states[3:6])
            
            #yaw = BNO.RectifyYaw(yaw,prev_state[4])
            
            # Make state vector.
            state =np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])



            # Get input from state.
            inputs = ctrl.CalculateControlAction_LQR(state,setpoint)
            # Change motor speeds.
            for i in range(0,4):
                mypi.set_servo_pulsewidth(pins[i], inputs[i])
            # write time, states, and inputs to a csv file
            save_vec = np.transpose(np.concatenate((np.array([[cur_time]]), state, inputs),axis=0))
            np.savetxt(myfile, save_vec, delimiter=',', fmt='%f')
        
    except KeyboardInterrupt: # This should allow us to exit the while loop by pressing Ctrl+C
        pass
    
# Sets drone to zero speed at end of program.
for pin in pins:
    mypi.set_servo_pulsewidth(pin, 1100)