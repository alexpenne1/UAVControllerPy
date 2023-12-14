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

def getState(bno, mytracker, object_name, filter_states, state):
    x, y, z                                              = Vicon.GetLinearStates(mytracker, object_name)
    yaw, pitch, roll, dyaw, dpitch, droll, a_x, a_y, a_z = BNO.getStates(bno)
    prev_time = cur_time
    cur_time  = time.time()
    dt        = cur_time - prev_time
    x, y, z, filter_states[0:3]          = ctrl.FilterViconPosition(x, y, z, dt, filter_states[0:3])
    dxdt, dydt, dzdt                     = ctrl.EstimateRates(x, y, z, dt, state[0:3])
    dxdt, dydt, dzdt, filter_states[3:6] = ctrl.FilterViconRates(dxdt, dydt, dzdt, dt, filter_states[3:6])
    yaw                                  = ctrl.RectifyYaw(yaw,state[5])
    state = np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
    return state, cur_time