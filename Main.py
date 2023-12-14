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
            inputs          = ctrl.CalculateControlAction_LQR(state, setpoint)
            ESC.writeMotors(mypi,pins,inputs)
            ctrl.SaveData(cur_time, state, inputs)
        
    except KeyboardInterrupt:
        pass
    
ESC.StopMotors(mypi,pins)