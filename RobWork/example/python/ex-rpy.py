from sdurw import *

if __name__ == '__main__':
    rpy = RPYd(Pi,Pi/2,0);
    print("RPY: " + str(rpy));
    rotationFromRPY = rpy.toRotation3D();
    print("Rotation from RPY: " + str(rotationFromRPY));

    rot = Rotation3Dd(-1,0,0,0,0,1,0,1,0);
    print("Rotation: " + str(rot));
    rpyFromRotation = RPYd(rot);
    print("RPY from Rotation: " + str(rpyFromRotation));
    rot = Rotation3Dd(1,0,0,0,1,0,0,0,1)
