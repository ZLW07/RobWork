from sdurw import Q
from sdurwsim import *

import sys,os

if __name__ == '__main__':
    dwc = DynamicWorkCellLoader.load(os.path.dirname(sys.argv[0]) + "/../../gtest/testfiles/ur_control_test_scene/cup_pg70_table.dwc.xml")
    cd = ownedPtr(ContactDetector(dwc.getWorkcell()))
    odesim = ODESimulator(dwc)
    state = dwc.getWorkcell().getDefaultState()
    odesim.initPhysics(state)

    devctrl = dwc.findSerialDeviceController("URController")
    ur = dwc.getWorkcell().findDevice("UR-6-85-5-A")

    target = Q(6,0,-0.2,0,0,0,0)
    devctrl.movePTP(target,100)

    for i in range(0,200):
        odesim.step(0.01,state)
        print(i,": ",ur.getQ(state))