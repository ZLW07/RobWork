#include <rw/kinematics.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/proximity.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/core/Exception.hpp>

#include <iostream>
#include <math.h>
#include <omp.h>
#include <string>

using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rwlibs::proximitystrategies;
using namespace std;

string CDQueryType[4] = {
    "AllContactsFullInfo", "AllContanctsNoInfo", "FirstContactFullInfo", "FirstContactNoInfo"};
string CSQueryType[2] = {"FirstContact","AllContacts"};

void printPSD (ProximityStrategyData data)
{
    cout << "    ProximityStrategyData" << endl;
    cout << "        Abs error  : " << data.abs_err << endl;
    cout << "        Rel error  : " << data.rel_err << endl;
    cout << "        QueryType  : " << CSQueryType[data.getCollisionQueryType ()] << endl;
    cout << "        inCollision: " << data.inCollision () << endl;
    CollisionStrategy::Result CSResult = data.getCollisionData ();

    try {
        cout << "\n        First Model Geometries: " << endl;
        for (string& geo : CSResult.a->getGeometryIDs ()) {
            cout << "            Geometry: " << geo << endl;
        }
        cout << "\n        Second Model Geometries: " << endl;
        for (string& geo : CSResult.b->getGeometryIDs ()) {
            cout << "            Geometry: " << geo << endl;
        }
    }
    catch (rw::core::Exception& e) {
        cout << "            " << e.what () << endl;
    }

    cout << "\n        Transform(aTb): " << CSResult._aTb << endl;
    cout << "        PrimTests      : " << CSResult.getNrPrimTests () << endl;
    cout << "        BVTests        : " << CSResult.getNrBVTests () << endl;
    cout << "        GeoVertex ids  : " << endl;
    for (pair< int, int >& ids : CSResult._geomPrimIds) {
        cout << "            Vertex: " << ids.first << " and " << ids.second << " colliding"
             << endl;
    }
    cout << "        CollisionPairs : " << endl;
    for (auto& ids : CSResult._collisionPairs) {
        cout << "            ColSize: " << ids.size << endl;
        cout << "            startID: " << ids.startIdx << endl;
        cout << "            ColPair: " << ids.geoIdxA << " and " << ids.geoIdxB << " colliding"
             << endl;
    }
}

void printColInfo (string name, CollisionDetector& detector, WorkCell::Ptr wc, State& state)
{
    cout << "#########################################" << endl;
    cout << "DetectingCollisions using: " << name << "\n" << endl;

    // done to pre call to initialize cache
    ProximityData res = ProximityData ();
    bool ret          = detector.inCollision (state, res);

    double start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    ProximityData res1 = ProximityData ();
    bool ret1          = detector.inCollision (state, res1);
    double time1       = detector.getComputationTime ();
    double end         = omp_get_wtime ();
    double time1t      = (end - start) * pow (10, 3);

    start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    ProximityData res2 = ProximityData ();
    res2.setCollisionQueryType (CollisionDetector::QueryType::AllContactsFullInfo);
    bool ret2     = detector.inCollision (state, res2);
    double time2  = detector.getComputationTime ();
    end           = omp_get_wtime ();
    double time2t = (end - start) * pow (10, 3);

    start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    CollisionDetector::QueryResult res3;
    bool ret3     = detector.inCollision (state, &res3);
    double time3  = detector.getComputationTime ();
    end           = omp_get_wtime ();
    double time3t = (end - start) * pow (10, 3);

    cout << "Time(ProximityData std)  : " << round (time1 * 1000) / 1000 << "ms - "
         << round (time1t * 10000) / 10000 << "ms" << endl;
    cout << "Time(ProximityData full) : " << round (time2 * 1000) / 1000 << "ms - "
         << round (time2t * 10000) / 10000 << "ms" << endl;
    cout << "Time(QueryResult)        : " << round (time3 * 1000) / 1000 << "ms - "
         << round (time3t * 10000) / 10000 << "ms" << endl;
    cout << endl;

    cout << "inCollision(ProximityData std)  : " << ret1 << endl;
    cout << "inCollision(ProximityData full) : " << ret2 << endl;
    cout << "inCollision(QueryResult)        : " << ret3 << endl;
    cout << endl;

    cout << "QueryType(ProximityData std) : " << CDQueryType[res1.getCollisionQueryType ()] << endl;
    cout << "QueryType(ProximityData full): " << CDQueryType[res2.getCollisionQueryType ()] << endl;
    cout << "QueryType(QueryResult)       : "
         << "Not Available" << endl;

    CollisionDetector::QueryResult res1QR = res1._collisionData;
    CollisionDetector::QueryResult res2QR = res2._collisionData;

    cout << "\nColliding Frames (ProximityData std)" << endl;
    for (const FramePair& frameP : res1QR.collidingFrames) {
        cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << endl;
    }

    cout << "\nColliding Frames (ProximityData full)" << endl;
    for (const FramePair& frameP : res2QR.collidingFrames) {
        cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << endl;
    }
    cout << "\nColliding Frames (QueryResult)" << endl;
    for (const FramePair& frameP : res3.collidingFrames) {
        cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << endl;
    }

    cout << "\nProximityStrategyData (ProximityData std)" << endl;
    for (ProximityStrategyData& PSD : res1QR._fullInfo) {
        printPSD (PSD);
    }

    cout << "\nProximityStrategyData (ProximityDataFULL)" << endl;
    for (ProximityStrategyData& PSD : res2QR._fullInfo) {
        printPSD (PSD);
    }

    cout << "\nProximityStrategyData (QueryType)" << endl;
    for (ProximityStrategyData& PSD : res3._fullInfo) {
        printPSD (PSD);
    }
}

int main (int argc, char** argv)
{
    WorkCell::Ptr wc   = WorkCellFactory::load ("../../ModelData/XMLScenes/RobotOnTable/Scene.xml");
    State state        = wc->getDefaultState ();
    MovableFrame* box1 = wc->findFrame< MovableFrame > ("m_box");
    MovableFrame* box2 = wc->findFrame< MovableFrame > ("m_box2");
    SerialDevice::Ptr ur = wc->findDevice< SerialDevice > ("UR-6-85-5-A");

    Vector3D< double > pos (0.3525, 0.3525, 0.1025);
    RPY< double > rot (0, 0, 3.14159);
    Transform3D< double > trans (pos, rot.toRotation3D ());
    box2->setTransform (trans, state);
    ur->setQ (Q (6, 1.46819, -1.02748, 2.55523, -3.10998, -1.56556, -0.429299), state);

    CollisionDetector rw_detector (wc);
    printColInfo ("Default Collision Detector", rw_detector, wc, state);

    for (string& id : ProximityStrategyFactory::getCollisionStrategyIDs ()) {
        CollisionDetector::Ptr prox_detector =
            new CollisionDetector (wc, ProximityStrategyFactory::makeCollisionStrategy (id));
        printColInfo (id + " Collision Detector", *prox_detector, wc, state);
    }

    return 0;
}