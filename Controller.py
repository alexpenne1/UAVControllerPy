from math import pi
import numpy as np
import logging
import sys
import time
import BNOSensor as BNO
import Vicon

def init(bno, mytracker, object_name):
    # State initialization
    x, y, z = Vicon.GetLinearStates(mytracker, object_name)
    yaw, roll, pitch, dyaw, droll, dpitch, a_x, a_y, a_z = BNO.getStates(bno)
    cur_time = time.time() 
    state = np.transpose(np.array([[x, y, z, roll, pitch, yaw, 0, 0, 0, droll, dpitch, dyaw]]))
    target_height = .3 # setpoint 1ft above initial
    setpoint = np.transpose(np.array([[x, y, z+target_height, roll, pitch, yaw, 0, 0, 0, 0, 0, 0]]))
    filter_states = np.array([0, 0, 0, 0, 0, 0]) # Initialize Vicon filters
    # Controller Parameters
    K = np.array([[-707.11, 0, 500, 0, -4183.61, -500, -1050.29, 0, 689.22, 0, -841.9, -766.82],
         [0, -707.11, 500, -4195.64, 0, 500, 0, -1051.12, 689.22, -846.73, 0, 766.82],
         [707.11, 0, 500, 0, 4183.61, -500, 1050.29, 0, 689.22, 0, 841.9, -766.82],
         [0, 707.11, 500, 4195.64, 0, 500, 0, 1051.12, 689.22, 846.73, 0, 766.82]])
    ue = np.transpose(4414.91*np.array([[1,1,1,1]]))
    vmax = 12.5
    rho = 7.77e7
    sigma = 7.19e-4 
    return setpoint, state, filter_states, cur_time, K, ue, vmax, rho, sigma

def FilterViconPosition(x, y, z, dt, filter_states):
    T = .1 # filter time constant
    K = 1  # filter gain
    xf      = filter_states[0]                     # output current filter state
    filter_states[0] = (1-dt/T)*filter_states[0] + K*dt/T*x # update filter state
    yf      = filter_states[1]                     # output current filter state
    filter_states[1] = (1-dt/T)*filter_states[1] + K*dt/T*y # update filter state
    zf      = filter_states[2]                     # output current filter state
    filter_states[2] = (1-dt/T)*filter_states[2] + K*dt/T*z # update filter state
    return xf, yf, zf, filter_states

def EstimateRates(x, y, z, dt, prev_state):
    # Naive Differentiation (amplifies noise)
    # grab x, y, z and return dx, dy, dz
    dxdt = (x - prev_state[0]) / dt
    dydt = (y - prev_state[1]) / dt
    dzdt = (z - prev_state[2]) / dt
    return dxdt, dydt, dzdt

def FilterViconRates(dxdt, dydt, dzdt, dt, filter_states):
    T = .25 # filter time constant
    K = 1  # filter gain
    dxf      = filter_states[0]                     # output current filter state
    filter_states[0] = (1-dt/T)*filter_states[0] + K*dt/T*dxdt # update filter state
    dyf      = filter_states[1]                     # output current filter state
    filter_states[1] = (1-dt/T)*filter_states[1] + K*dt/T*dydt # update filter state
    dzf      = filter_states[2]                     # output current filter state
    filter_states[2] = (1-dt/T)*filter_states[2] + K*dt/T*dzdt # update filter state
    return dxf, dyf, dzf, filter_states

def RectifyYaw(yaw,prev_yaw):
    if yaw > prev_yaw + pi:
        yaw = yaw - 2*pi
    elif yaw < prev_yaw - pi:
        yaw = yaw + 2*pi
    return yaw 

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
