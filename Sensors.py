from Adafruit_BNO055 import BNO055
import BNOSensor as BNO
import Vicon

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