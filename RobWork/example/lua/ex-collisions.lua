require("sdurw_core")
require("sdurw_math")
require("sdurw_kinematics")
require("sdurw_models")
require("sdurw_proximity")
require("sdurw")
require("sdurw_proximitystrategies")

using("sdurw_core")
using("sdurw_math")
using("sdurw_kinematics")
using("sdurw_models")
using("sdurw_proximity")
using("sdurw")
using("sdurw_proximitystrategies")

CDQueryType = {"AllContactsFullInfo","AllContanctsNoInfo","FirstContactFullInfo","FirstContactNoInfo"}
CSQueryType = {"FirstContact","AllContacts"}

function round(number,decimals)
    decimals = math.pow(10,decimals)
    return math.floor(number*decimals+0.5)/decimals
end

function printPSD(data,name)
    print("    ProximityStrategyData")
    print("        Abs error  : "..data.abs_err)
    print("        Rel error  : "..data.rel_err)

    local num = tonumber(data:getCollisionQueryType())+1
    if (num == 2 or num == 1) then
        print("        QueryType  : "..CSQueryType[num])
    else
        print("        QueryType  : ".. (num-1) .. " index out of range")
    end 

    print("        inCollision: ".. tostring(data:inCollision()))
    CSResult = data:getCollisionData()

    if name ~= "YAOBI Collision Detector" then
        print("\n        First Model Geometries: ")
        for i=0,CSResult.a:getGeometryIDs():size()-1 do
            print("            Geometry: "..CSResult.a:getGeometryIDs()[i])
        end
        print("\n        Second Model Geometries: ")
        for i=0,CSResult.b:getGeometryIDs():size()-1 do
            print("            Geometry: "..CSResult.b:getGeometryIDs()[i])
        end  
    end

    print("\n        Transform(aTb): " .. tostring(CSResult._aTb))
    print("        PrimTests      : " .. tostring(CSResult:getNrPrimTests()))
    print("        BVTests        : " .. tostring(CSResult:getNrBVTests()))
    print("        GeoVertex ids  : ")

    for i=0,CSResult._geomPrimIds:size()-1 do
        local ids = CSResult._geomPrimIds[i]
        print("            Vertex: ".. ids.first .. " and " ..  ids.second .. " colliding")
    end

    print("        CollisionPairs : ")    
    for i=0,CSResult._collisionPairs:size()-1 do
        local ids = CSResult._collisionPairs[i]
        print("            ColSize: "..ids.size)
        print("            StartID: "..ids.startIdx)
        print("            ColPair: "..ids.geoIdxA .. " and " .. ids.geoIdxB .. " colliding")
    end

end

function printColInfo(name,detector,wc,state)
    print("#########################################")
    print("DetectingCollisions using: " .. name .."\n")

    local start_t = os.clock()
    detector:resetComputationTimeAndCount()
    local res1 = ProximityData()
    local ret1 = detector:inCollision(state,res1)
    local time1 = detector:getComputationTime()
    local end_t = os.clock()
    local time1t =(end_t-start_t)*1000

    start_t = os.clock()
    detector:resetComputationTimeAndCount()
    local res2 = ProximityData()
    res2:setCollisionQueryType(CollisionDetectorQueryType_AllContactsFullInfo) 
    local ret2 = detector:inCollision(state,res2)
    local time2 = detector:getComputationTime()
    end_t = os.clock()
    local time2t =(end_t-start_t)*1000

    start_t = os.clock()
    detector:resetComputationTimeAndCount()
    local res3 = CollisionDetectorQueryResult()
    local ret3 = detector:inCollision(state,res3)
    local time3 = detector:getComputationTime()
    end_t = os.clock()
    local time3t =(end_t-start_t)*1000

    start_t = os.clock()
    detector:resetComputationTimeAndCount()
    local res4 = FramePairVector()
    local ret4 = detector:inCollision(state,res4)
    local time4 = detector:getComputationTime()
    end_t = os.clock()
    local time4t =(end_t-start_t)*1000

    print("Time(ProximityData std)  : "..round(time1,3).. "ms - ".. round(time1t,4).."ms")
    print("Time(ProximityData full) : "..round(time2,3).. "ms - ".. round(time2t,4).."ms")
    print("Time(QueryResult)        : "..round(time3,3).. "ms - ".. round(time3t,4).."ms")
    print("Time(FramePairVector)    : "..round(time4,3).. "ms - ".. round(time4t,4).."ms")
    print("")

    print("inCollision(ProximityData std)  : ".. tostring(ret1))
    print("inCollision(ProximityData full) : ".. tostring(ret2))
    print("inCollision(QueryResult)        : ".. tostring(ret3))
    print("inCollision(FramePairVector)    : ".. tostring(ret4))
    print("")

    print("QueryType(ProximityData std) : " .. CDQueryType[tonumber(res1:getCollisionQueryType())+1])
    print("QueryType(ProximityData full): " .. CDQueryType[tonumber(res2:getCollisionQueryType())+1])
    print("QueryType(QueryResult)       : " .. "Not Available")
    print("QueryType(FramePairVector)   : " .. "Not Available")

    local res1CD = res1._collisionData
    local res2CD = res2._collisionData
    
    print("\nColliding Frames (ProximityData std)")
    for i=0,res1CD:getFramePairVector():size()-1 do
        local frameP = res1CD:getFramePairVector()[i]
        print("    " .. frameP.first:getName() .. " and " .. frameP.second:getName())
    end
    print("\nColliding Frames (ProximityData full)")
    for i=0,res2CD:getFramePairVector():size()-1 do
        local frameP = res2CD:getFramePairVector()[i]
        print("    " .. frameP.first:getName() .. " and " .. frameP.second:getName())
    end
    print("\nColliding Frames (QueryResult)")
    for i=0,res3:getFramePairVector():size()-1 do
        local frameP = res3:getFramePairVector()[i]
        print("    " .. frameP.first:getName() .. " and " .. frameP.second:getName())
    end
    print("\nColliding Frames (FramePairVector)")
    for i=0,res4:size()-1 do
        local frameP = res4[i]
        print("    " .. frameP.first:getName() .. " and " .. frameP.second:getName())
    end

    print("\nNo more data in (FramePairVector)")

    print("\nProximityStrategyData (ProximityData std)")
    for i=0,res1CD._fullInfo:size()-1 do
        printPSD(res1CD._fullInfo[i],name)
    end 

    print("\nProximityStrategyData (ProximityDataFULL)")
    for i=0,res2CD._fullInfo:size()-1 do 
        printPSD(res2CD._fullInfo[i],name)
    end 

    print("\nProximityStrategyData (QueryType)")
    for i=0,res3._fullInfo:size()-1 do
        printPSD(res3._fullInfo[i],name)
    end

end

wc =  WorkCellLoaderFactory.load("../ModelData/XMLScenes/RobotOnTable/Scene.xml")
state = wc:getDefaultState()
box1 = wc:findMovableFrame("m_box")
box2 = wc:findMovableFrame("m_box2")
ur = wc:findSerialDevice("UR-6-85-5-A")

pos =  Vector3Dd(0.3525, 0.3525, 0.1025)
rot = RPYd(0, 0, 3.14159)
trans = Transform3Dd(pos, rot:toRotation3D())
box2:setTransform(trans,state)
ur:setQ(Q(6,1.46819, -1.02748, 2.55523, -3.10998, -1.56556, -0.429299),state)

rw_detector = sdurw_proximity.ownedPtr(CollisionDetector(wc))
printColInfo("Default Collision Detector", rw_detector,wc,state)

for i=0,sdurw_proximitystrategies.ProximityStrategyFactory.getCollisionStrategyIDs():size()-1 do
    id = sdurw_proximitystrategies.ProximityStrategyFactory.getCollisionStrategyIDs()[i]
    prox_detector = sdurw_proximity.ownedPtr(CollisionDetector(wc,sdurw_proximitystrategies.ProximityStrategyFactory.makeCollisionStrategy(id)))
    printColInfo(id .. " Collision Detector", prox_detector,wc,state)
end

