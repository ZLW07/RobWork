from sdurw_proximity import *
from sdurw import *
from sdurw_proximitystrategies import*

import time

CDQueryType = ["AllContactsFullInfo","AllContanctsNoInfo","FirstContactFullInfo","FirstContactNoInfo"]
CSQueryType = ["FirstContact","AllContacts"]

def printPSD(data:"ProximityStrategyData"):
    print("    ProximityStrategyData")
    print("        Abs error  : ",data.abs_err)
    print("        Rel error  : ",data.rel_err)

    try:
        print("        QueryType  : ",CSQueryType[data.getCollisionQueryType()])
    except IndexError:
        print("        QueryType  : ",data.getCollisionQueryType(), "index out of range")

    print("        inCollision: ",data.inCollision())
    CSResult = data.getCollisionData()

    try:
        print("\n        First Model Geometries: ")
        for geo in CSResult.a.getGeometryIDs():
            print("            Geometry: ",geo)
        print("\n        Second Model Geometries: ")
        for geo in CSResult.b.getGeometryIDs():
            print("            Geometry: ",geo)
    except RuntimeError as e:
        print("        ",e)
        

    print("\n        Transform(aTb): ",CSResult._aTb)
    print("        PrimTests      : ",CSResult.getNrPrimTests())
    print("        BVTests        : ",CSResult.getNrBVTests())
    print("        GeoVertex ids  : ")
    for ids in CSResult._geomPrimIds:
        print("            Vertex: ",ids[0], " and ", ids[1], " colliding")

    print("        CollisionPairs : ")    
    for ids in CSResult._collisionPairs:
        print("            ColSize: ",ids.size)
        print("            StartID: ",ids.startIdx)
        print("            ColPair: ",ids.geoIdxA, " and ", ids.geoIdxB, " colliding")

def printColInfo(name,detector:"CollisionDetector",wc:"WorkCellPtr",state:"State"):
    
    print("#########################################")
    print("DetectingCollisions using: ",name,"\n")

    # done to pre call to initialize cache
    res1 = ProximityData()
    ret1 = detector.inCollision(state,res1)

    start = time.time()
    detector.resetComputationTimeAndCount()
    res1 = ProximityData()
    ret1 = detector.inCollision(state,res1)
    time1 = detector.getComputationTime()
    end = time.time()
    time1t =(end-start)*pow(10,3)

    start = time.time()
    detector.resetComputationTimeAndCount()
    res2 = ProximityData()
    res2.setCollisionQueryType(CollisionDetectorQueryType.AllContactsFullInfo) 
    ret2 = detector.inCollision(state,res2)
    time2 = detector.getComputationTime()
    end = time.time()
    time2t =(end-start)*pow(10,3)

    start = time.time()
    detector.resetComputationTimeAndCount()
    res3 = CollisionDetectorQueryResult()
    ret3 = detector.inCollision(state,res3)
    time3 = detector.getComputationTime()
    end = time.time()
    time3t =(end-start)*pow(10,3)

    start = time.time()
    detector.resetComputationTimeAndCount()
    res4 = FramePairVector()
    ret4 = detector.inCollision(state,res4)
    time4 = detector.getComputationTime()
    end = time.time()
    time4t =(end-start)*pow(10,3)

    print("Time(ProximityData std)  : ",round(time1,3), "ms - ", round(time1t,4),"ms")
    print("Time(ProximityData full) : ",round(time2,3), "ms - ", round(time2t,4),"ms")
    print("Time(QueryResult)        : ",round(time3,3), "ms - ", round(time3t,4),"ms")
    print("Time(FramePairVector)    : ",round(time4,3), "ms - ", round(time4t,4),"ms")
    print("")

    print("inCollision(ProximityData std)  : ",ret1)
    print("inCollision(ProximityData full) : ",ret2)
    print("inCollision(QueryResult)        : ",ret3)
    print("inCollision(FramePairVector)    : ",ret4)
    print("")

    print("QueryType(ProximityData std) : ", CDQueryType[res1.getCollisionQueryType()])
    print("QueryType(ProximityData full): ", CDQueryType[res2.getCollisionQueryType()])
    print("QueryType(QueryResult)       : ","Not Available")
    print("QueryType(FramePairVector)   : ","Not Available")

    res1CD = res1._collisionData
    res2CD = res2._collisionData

    print("\nColliding Frames (ProximityData std)")
    for frameP in res1CD.getFramePairVector():
        print(frameP)
        print("    ", frameP[0].getName(), " and ", frameP[1].getName())
    print("\nColliding Frames (ProximityData full)")
    for frameP in res2CD.getFramePairVector():
        print("    ", frameP[0].getName(), " and ", frameP[1].getName())
    print("\nColliding Frames (QueryResult)")
    for frameP in res3.getFramePairVector():
        print("    ", frameP[0].getName(), " and ", frameP[1].getName())
    print("\nColliding Frames (FramePairVector)")
    for frameP in res4:
        print("    ", frameP[0].getName(), " and ", frameP[1].getName())

    print("\nNo more data in (FramePairVector)")

    print("\nProximityStrategyData (ProximityData std)")
    for i in range(res1CD._fullInfo.size()): #loop with indexes else you might get segfault
        printPSD(res1CD._fullInfo[i])
    
    print("\nProximityStrategyData (ProximityDataFULL)")
    for i in range(res2CD._fullInfo.size()): #loop with indexes else you might get segfault
        printPSD(res2CD._fullInfo[i])
    
    print("\nProximityStrategyData (QueryType)")
    for i in range(res3._fullInfo.size()): #loop with indexes else you might get segfault
        printPSD(res3._fullInfo[i])

if __name__ == '__main__':
    wc = WorkCellLoaderFactory.load("../ModelData/XMLScenes/RobotOnTable/Scene.xml")
    state = wc.getDefaultState()
    box1 = wc.findMovableFrame("m_box")
    box2 = wc.findMovableFrame("m_box2")
    ur = wc.findSerialDevice("UR-6-85-5-A")
    
    pos =  Vector3Dd(0.3525, 0.3525, 0.1025)
    rot = RPYd(0, 0, 3.14159)
    trans = Transform3Dd(pos,rot.toRotation3D())
    box2.setTransform(trans,state)
    ur.setQ(Q(6,1.46819, -1.02748, 2.55523, -3.10998, -1.56556, -0.429299),state)

    rw_detector = sdurw_proximity.ownedPtr(CollisionDetector(wc))
    printColInfo("Default Collision Detector", rw_detector,wc,state)

    for id in ProximityStrategyFactory.getCollisionStrategyIDs():
        #needs to be ownedPtr to avoid segfault
        prox_detector = sdurw_proximity.ownedPtr(CollisionDetector(wc,ProximityStrategyFactory.makeCollisionStrategy(id)))
        printColInfo(id + " Collision Detector", prox_detector,wc,state)





