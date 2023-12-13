import time
import pyvicon_datastream as pv
from pyvicon_datastream import tools

def connectVicon(VICON_TRACKER_IP):
    print(f"Connecting to Vicon at {VICON_TRACKER_IP}...")
    vicon_client = pv.PyViconDatastream()
    print(f"{vicon_client}")
    ret = vicon_client.connect(VICON_TRACKER_IP)
    if ret != pv.Result.Success:
        print(f"Connection to {VICON_TRACKER_IP} failed")
    else:
        print(f"Connection to {VICON_TRACKER_IP} successful")
    mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
    return vicon_client, mytracker
    #VICON_TRACKER_IP = "192.168.0.101"

def GetLinearStates(tracker, object_name):
# (latency, frame number, [[object_name,object_name,x,y,z,roll,pitch,yaw]]) (mm, rad)
    positions = tracker.get_position(object_name)
    # Extract x, y, z and return
    x = positions[2][0][2]
    y = positions[2][0][3]
    z = positions[2][0][4]
    return x, y, z


    
