from sdurw import *
import math

if __name__ == '__main__':
    eaa = EAAd(math.sqrt(2)/2*Pi,math.sqrt(2)/2*Pi,0);
    print("EAA: " + str(eaa))
    print(" angle: " + str(eaa.angle()))
    print(" axis: " + str(eaa.axis()));
    rotationFromEAA = eaa.toRotation3D();
    print("Rotation from EAA: " + str(rotationFromEAA));

    rot = Rotation3Dd(-1,0,0,0,0,1,0,1,0);
    print("Rotation: " + str(rot));
    eaaFromRotation = EAAd(rot);
    print("EAA from Rotation: " + str(eaaFromRotation));
    print(" angle: " + str(eaaFromRotation.angle()));
    print(" axis: " + str(eaaFromRotation.axis()));