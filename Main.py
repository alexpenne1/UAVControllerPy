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

setpoint, state, filter_states, cur_time, K, ue, vmax, rho, sigma = ctrl.init(bno, mytracker, object_name)

# Controller loop.
with open('data.csv', 'w', newline='') as myfile:
    #csvwriter = csv.writer(myfile)
    try:
        while True:
            state, dx, cur_time = Sensors.getState(bno, mytracker, object_name, filter_states, state, setpoint, cur_time)
            inputs          = ctrl.CalculateControlAction_LQR(dx, K, ue, vmax, rho, sigma)
            ESC.writeMotors(mypi,pins,inputs)
            ctrl.SaveData(myfile, cur_time, state, inputs, dx)
        
    except KeyboardInterrupt:
        pass
    
ESC.StopMotors(mypi,pins)