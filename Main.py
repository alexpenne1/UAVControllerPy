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

setpoint, state, cur_time, K, ue, vmax, rho, sigma, filter_states, filter_T, filter_K, yaw_looper = ctrl.init(bno, mytracker, object_name)

# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            state, dx, cur_time, filter_states = Sensors.getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, filter_T, filter_K, yaw_looper)
            inputs          = ctrl.CalculateControlAction_LQR(dx, K, ue, vmax, rho, sigma)
            ESC.writeMotors(mypi,pins,inputs)
            ctrl.SaveData(myfile, cur_time, state, inputs, dx)
        
    except KeyboardInterrupt:
        pass
    
ESC.StopMotors(mypi,pins)