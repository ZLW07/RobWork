*******************
Collision Detection
*******************

The rw::proximity package provides functionality for collision checking. When using a WorkCell and Frames the primary interface will be the rw::proximity::CollisionDetector. To each frames there can be zero or more geometries associated. When stating the two frames are checked against each other, it is in reality the geometries of these which are tested. Notice, that two geometries associated to the same frame are never tested against each other.

Inside the CollisionDetector the checking is divided into two phases:
* Broad phase: Provides a filtering which determines which pairs of frames that need to be tested in the narrow phase.
* Narrow phase: Performs the actual collision checking between geometries.

The default broad phase filter is the rw::proximity::BasicFilterStrategy which contains the rules specified in the CollisionSetup associated to a rw::models::WorkCell. The BasicFilterStrategy maintains a list of frame pairs to check, which can be modified at runtime through include and exclude methods. 

The narrow phase is implemented through a rw::proximity::CollisionStrategy, which may wrap external libraries such as Yaobi or PQP. These wrappers for external libraries are placed in the rwlibs::proximitystrategies package. The CollisionStrategy buffers collision models and maintains a map of relations between frames and models. 
	

Collision Checking - Workcells
==============================

This program shows how to construct a collision detector for the default collision setup of a workcell. The example program then calls the collision detector to see if the workcell is in collision in its initial state:

.. literalinclude:: ../../../RobWork/example/snippets/ex-collisions.cpp
   :language: c++
   :linenos:

Adding/Removing Geometries
========================== 
The content of the collision detector can be modified online by using the addModel and removeModel methods on rw::proximity::CollisionDetector. If for instance a new object is detected in a frame it can added and remove it by:

.. code-block:: c++

   using namespace rw::proximity;
   using namespace rw::kinematics;
   using namespace rw::geometry;
	
   void addGeometryToDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry)	
   {
       cd->addModel(myframe, mygeometry);
   }

   void removeGeometryFromDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry) 
   {		
       cd->removeModel(myframe, mygeometry->getId())
   }

Modifying Broad Phase Filter
============================
When simulating a robot picking up an objects from a table it is necessary to modify the broad phase filter such that collision detection is disable between the object and the robot tool and enables between object and table.

To do this we can modify the broad phase filter as follows

.. code-block:: c++

   BasicFilterStrategy::Ptr broadphase = ownedPtr(new BasicFilterStrategy(workcell));
   CollisionDetector::Ptr collisionDetector = ownedPtr(new CollisionDetector(workcell, ProximityStrategyYaobi::make(), broadphase));

   ...

   //Tool frame of the robot
   Frame* toolFrame = workcell->findFrame("Robot.TCP");
   //Frame of the object picked up
   Frame* objectFrame = workcell->findFrame("Object");
   //Frame of the table on which the object previously was located.
   Frame* tableFrame = workcell->findFrame("Table");

   //Remove checking between objectFrame and toolFrame
   broadphase->exclude(rw::kinematics::FramePair(objectFrame, toolFrame);
   //Add checking between the objectFrame and the tableFrame
   broadphase->include(rw::kinematics::FramePair(objectFrame, tableFrame);


.. TODO:
   Bullet
   FCL
   PQP
   Yaobi

   Distance detection
   ===================

   Contact detection
   ===================
   PQP
   Primitive Pairs
   Analytic Geometries
