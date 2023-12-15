from Adafruit_BNO055 import BNO055
import BNOSensor as BNO
import Vicon
import time
import Controller as ctrl
import numpy as np

# CONNECT AND CALLIBRATE
def init(calibrate):
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18: UART mode must be turned on (PS1 pin = HIGH).
    # Connect to IMU.
    bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
    BNO.connectSensor(bno)
    print("Sensor connected!")
    if calibrate: # Optional
        BNO.callibrateSensor(bno)
    print("Sensor callibrated!\n\n")
    # Connect to Vicon
    vicon_client, mytracker = Vicon.connectVicon("192.168.0.101")
    object_name = "LoCicero_RPI_Drone"
    return bno, mytracker, object_name

def getState(bno, mytracker, object_name, state, setpoint, cur_time, filter_states, filter_T, filter_K):
    rawx, rawy, rawz                                     = Vicon.GetLinearStates(mytracker, object_name)
    yaw, pitch, roll, dyaw, dpitch, droll, a_x, a_y, a_z = BNO.getStates(bno)
    yaw       = ctrl.RectifyYaw(yaw,state[5])
    prev_time = cur_time
    cur_time  = time.time()
    dt        = cur_time - prev_time
    x, filter_states[0]       = ctrl.FilterSignal(rawx, dt, filter_states[0], filter_T[0], filter_K[0])
    y, filter_states[1]       = ctrl.FilterSignal(rawy, dt, filter_states[1], filter_T[1], filter_K[1])
    z, filter_states[2]       = ctrl.FilterSignal(rawz, dt, filter_states[2], filter_T[2], filter_K[2])
    rawdxdt, rawdydt, rawdzdt = ctrl.EstimateRates(x, y, z, dt, state[0:3])
    dxdt, filter_states[3]    = ctrl.FilterSignal(rawdxdt, dt, filter_states[3], filter_T[3], filter_K[3])
    dydt, filter_states[4]    = ctrl.FilterSignal(rawdydt, dt, filter_states[4], filter_T[4], filter_K[4])
    dzdt, filter_states[5]    = ctrl.FilterSignal(rawdzdt, dt, filter_states[5], filter_T[5], filter_K[5])
    '''
    print(x)
    print(y)
    print(z)
    print(roll)
    print(pitch)
    print(yaw)
    print(dxdt)
    '''
    state = np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
    dx = state - setpoint
    return state, dx, cur_time, filter_states