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

def getState(bno, mytracker, object_name, state, setpoint, cur_time, xfilter, yfilter, zfilter, dxfilter, dyfilter, dzfilter, Tfilter, Kfilter, dTfilter, dKfilter):
    rawx, rawy, rawz                                     = Vicon.GetLinearStates(mytracker, object_name)
    yaw, pitch, roll, dyaw, dpitch, droll, a_x, a_y, a_z = BNO.getStates(bno)
    yaw       = ctrl.RectifyYaw(yaw,state[5])
    prev_time = cur_time
    cur_time  = time.time()
    dt        = cur_time - prev_time
    x, xfilter       = ctrl.FilterSignal(rawx, dt, xfilter, Tfilter, Kfilter)
    y, yfilter       = ctrl.FilterSignal(rawy, dt, yfilter, Tfilter, Kfilter)
    z, zfilter       = ctrl.FilterSignal(rawz, dt, zfilter, Tfilter, Kfilter)
    rawdxdt, rawdydt, rawdzdt = ctrl.EstimateRates(x, y, z, dt, state[0:3])
    dxdt, dxfilter = ctrl.FilterSignal(rawdxdt, dt, dxfilter, dTfilter, dKfilter)
    dydt, dyfilter = ctrl.FilterSignal(rawdydt, dt, dyfilter, dTfilter, dKfilter)
    dzdt, dzfilter = ctrl.FilterSignal(rawdzdt, dt, dzfilter, dTfilter, dKfilter)
    state = np.array([[x],[y],[z],[roll],[pitch],[yaw],[dxdt],[dydt],[dzdt],[droll],[dpitch],[dyaw]])
    dx = state - setpoint
    return state, dx, cur_time