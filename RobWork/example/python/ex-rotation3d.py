from sdurw import *

if __name__ == '__main__':
    rotd = Rotation3Dd(1,0,0,0,0,-1,0,1,0);
    rotf = Rotation3Df(1,0,0,0,0,-1,0,1,0);
    
    print("Rotation double:");
    print(str(rotd));
    print("Rotation float:");
    print(str(rotf));
    print("Rotation inverse:");
    print(str(inverse(rotd)));
    print("Identity:");
    print(str(rotd*inverse(rotd)));

    rotd[2,2] = 25;
    print("Corner = 25")
    print(str(rotd))

    if rotd[2,2] == 25:
        exit(0)
    else:
        exit(1)