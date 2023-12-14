import numpy as np
import logging
import sys
import time

def FilterViconPosition(x, y, z, dt, fstatex, fstatey, fstatez):
    T = .1 # filter time constant
    K = 1  # filter gain
    xf      = fstatex                     # output current filter state
    fstatex = (1-dt/T)*fstatex + K*dt/T*x # update filter state
    yf      = fstatey                     # output current filter state
    fstatey = (1-dt/T)*fstatey + K*dt/T*y # update filter state
    zf      = fstatez                     # output current filter state
    fstatez = (1-dt/T)*fstatez + K*dt/T*z # update filter state
    return xf, yf, zf, fstatex, fstatey, fstatez

def EstimateRates(x, y, z, dt, prev_state):
    # Naive Differentiation (amplifies noise)
    # grab x, y, z and return dx, dy, dz
    dxdt = (x - prev_state[0]) / dt
    dydt = (y - prev_state[1]) / dt
    dzdt = (z - prev_state[2]) / dt
    return dxdt, dydt, dzdt

def FilterViconRates(dxdt, dydt, dzdt, dt, fstatedx, fstatedy, fstatedz):
    T = .25 # filter time constant
    K = 1  # filter gain
    dxf      = fstatedx                     # output current filter state
    fstatedx = (1-dt/T)*fstatedx + K*dt/T*dxdt # update filter state
    dyf      = fstatedx                     # output current filter state
    fstatedy = (1-dt/T)*fstatedy + K*dt/T*dydt # update filter state
    dzf      = fstatedx                     # output current filter state
    fstatedz = (1-dt/T)*fstatedz + K*dt/T*dzdt # update filter state
    return dxf, dyf, dzf, fstatedx, fstatedy, fstatedz

def CalculateControlAction_LQR(x,xe):
    K = np.array([[-707.11, 0, 500, 0, -4183.61, -500, -1050.29, 0, 689.22, 0, -841.9, -766.82],
         [0, -707.11, 500, -4195.64, 0, 500, 0, -1051.12, 689.22, -846.73, 0, 766.82],
         [707.11, 0, 500, 0, 4183.61, -500, 1050.29, 0, 689.22, 0, 841.9, -766.82],
         [0, 707.11, 500, 4195.64, 0, 500, 0, 1051.12, 689.22, 846.73, 0, 766.82]])
    ue = np.transpose(4414.91*np.array([[1,1,1,1]]))
    vmax = 12.5
    rho = 7.77e7
    sigma = 7.19e-4 
    u = ue - np.matmul(K,(x - xe))
    PW = 800/(rho*vmax)*(np.power((u + rho*sigma),2) - np.power(rho*sigma,2)) + 1100
    return PW

'''
def CalculateControlAction_TestFeedback(state):
    # Initialize input array.
    inputs = np.zeros(4)
    # First two motors proportional to yaw.
    inputs[0] = DutyCycle2PulseWidth((state[3])*.05/(360) + 0.05)
    inputs[1] = DutyCycle2PulseWidth((state[3])*.05/(360) + 0.05)
    # Second two motors proportional to z.
    inputs[2] = DutyCycle2PulseWidth((state[2] - 40)/1740*.05 + 0.05)
    inputs[3] = DutyCycle2PulseWidth((state[2] - 40)/1740*.05 + 0.05)
    # Return inputs array.
    return inputs

def DutyCycle2PulseWidth(DC):
    return (((DC - 0.05) / 0.05) * 800) + 1100

'''