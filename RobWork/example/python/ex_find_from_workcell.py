from sdurw import *

def findFromWorkCell(wc):
    # get the default state
    state = wc.getDefaultState();
    worldFrame = wc.getWorldFrame();
    # find a frame by name, remember NULL is a valid return
    frame = wc.findFrame("FixedFrameName");
    # find a frame by name, but with a specific frame type
    fframe = wc.findFixedFrame("FixedFrameName");
    mframe = wc.findMovableFrame("MovableFrameName");
    # find a device by name
    device = wc.findDevice("SerialDeviceName");
    sdevice = wc.findSerialDevice("SerialDeviceName");