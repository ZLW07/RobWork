from sdurw import *

def fwdKinematicsDevice(sdevice, mframe, state):
    # get device base frame
    base = sdevice.getBase();
    # get device end effector
    end = sdevice.getEnd();
    # calculate base to end transform
    bTe = sdevice.baseTend(state);
    # or just base to any frame
    bTmf = sdevice.baseTframe(mframe, state);
    # get device name
    sdevicename = sdevice.getName();
    # the degrees of freedom of this device
    dof = sdevice.getDOF();
    # set the configuration of the device to zero
    sdevice.setQ( Q(dof,0.0) , state );
