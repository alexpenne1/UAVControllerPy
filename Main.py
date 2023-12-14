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

setpoint, state, filter_states, cur_time = ctrl.init(bno, mytracker, object_name)

# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            state, cur_time = Sensors.getState(bno, mytracker, object_name, filter_states, state, cur_time)

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