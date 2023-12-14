from Adafruit_BNO055 import BNO055
import BNOSensor as BNO
import Vicon

def init():
    # CONNECT AND CALLIBRATE
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
    # UART mode must be turned on (PS1 pin = HIGH).
    bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
    # Connect the sensor.
    BNO.connectSensor(bno)
    print("Sensor connected!")
    # Callibrate the sensor.
    #BNO.callibrateSensor(bno)
    print("Sensor callibrated!\n\n")
    # Connect to Vicon
    vicon_client, mytracker = Vicon.connectVicon("192.168.0.101")
    object_name = "LoCicero_RPI_Drone"
    return bno, mytracker, object_name