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


    
