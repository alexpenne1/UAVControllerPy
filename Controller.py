from math import pi
import numpy as np
import logging
import sys
import time
import BNOSensor as BNO
import Vicon
import csv

def init(bno, mytracker, object_name):
    # Initial localization
    x, y, z = Vicon.GetLinearStates(mytracker, object_name)
    yaw, pitch, roll, dyaw, droll, dpitch, a_x, a_y, a_z = BNO.getStates(bno)
    yaw_looper = 0
    rawyaw = yaw
    cur_time = time.time() 
    state = np.transpose(np.array([[x, y, z, roll, pitch, yaw, 0, 0, 0, droll, dpitch, dyaw]]))
    # Create Setpoint
    target_height = .3
    setpoint = np.transpose(np.array([[x, y, z+target_height, roll, pitch, yaw, 0, 0, 0, 0, 0, 0]]))
    # Initialize Vicon filter states and parameters
    filter_states = [x, y, z, 0, 0, 0]
    filter_T      = [.1, .1, .1, .25, .25, .25]
    filter_K      = [1, 1, 1, 1, 1, 1]
    # Controller Parameters
    reader = csv.reader(open("ControlDesign/Controllers/LQRcontroller.csv", "r"), delimiter=",")
    K = list(reader)
    K = np.array(K).astype("float")
    '''
    K = np.array([[-707.11, 0, 500, 0, -4183.61, -500, -1050.29, 0, 689.22, 0, -841.9, -766.82],
         [0, -707.11, 500, -4195.64, 0, 500, 0, -1051.12, 689.22, -846.73, 0, 766.82],
         [707.11, 0, 500, 0, 4183.61, -500, 1050.29, 0, 689.22, 0, 841.9, -766.82],
         [0, 707.11, 500, 4195.64, 0, 500, 0, 1051.12, 689.22, 846.73, 0, 766.82]])
    '''
    ue = np.transpose(4414.91*np.array([[1,1,1,1]]))
    vmax = 12.5
    rho = 7.77e7
    sigma = 7.19e-4 
    return setpoint, state, cur_time, K, ue, vmax, rho, sigma, filter_states, filter_T, filter_K, yaw_looper, rawyaw

def FilterSignal(signal_in,dt,filter_state,T,K):
    signal_out = filter_state
    filter_state = (1-dt/T)*filter_state + K*dt/T*signal_in
    return signal_out, filter_state

def EstimateRates(x, y, z, dt, prev_state):
    dxdt = (x - prev_state[0]) / dt
    dydt = (y - prev_state[1]) / dt
    dzdt = (z - prev_state[2]) / dt
    return dxdt.item(), dydt.item(), dzdt.item()

def RectifyYaw(yaw,prev_yaw,yaw_looper):
    if yaw > prev_yaw + np.pi:
        # Assume yaw has increased past 2pi, so 2pi must be subtracted
        yaw_looper = yaw_looper - 2*np.pi
    elif yaw < prev_yaw - np.pi:
        # Assume yaw has decreased past 2pi, so 2pi must be added
        yaw_looper = yaw_looper + 2*np.pi
    yaw = yaw + yaw_looper
    return yaw, yaw_looper 

def SaveData(myfile, cur_time, state, inputs, dx):
    save_vec = np.transpose(np.concatenate((np.array([[cur_time]]), state, inputs, dx),axis=0))
    np.savetxt(myfile, save_vec, delimiter=',', fmt='%f')
    return   

def CalculateControlAction_LQR(dx, K, ue, vmax, rho, sigma):
    u = ue - np.matmul(K,dx)
    PW = 800/(rho*vmax)*(np.power((u + rho*sigma),2) - np.power(rho*sigma,2)) + 1100
    for index in range(4):
        if PW[index] > 1900:
            PW[index] = 1900
        elif PW[index] < 1100:
            PW[index] = 1100
    return PW
