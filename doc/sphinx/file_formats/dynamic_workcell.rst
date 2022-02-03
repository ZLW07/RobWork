***********************
Dynamic WorkCell Format
***********************

Introduction
============

The dynamic workcell (DWC) XML file format have suffix .dwc.xml and follow the rules of standard XML.
See also the :ref:`section about dynamic workcells <workcell_format>` in the manual.
A dynamic workcell can be loaded with rwsim::loaders::DynamicWorkCellLoader::load() :

.. code-block:: cpp

   DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load("DynamicWorkCellFile.dwc.xml");


The dynamic workcell file referes to an existing workcell file, and adds dynamic concepts
such as dynamic bodies, dynamic devices, mass, inertia, material properties, constraints,
gravity, controllers, and sensors.

The DynamicWorkCell element is the root element in the fileformat and refers to the underlying WorkCell file:

.. code-block:: xml

   <DynamicWorkcell workcell="pih.wc.xml">
       ...
   </DynamicWorkcell>


XML Elements
============

In the following the different possible elements in the DWC format will be described.

DynamicWorkCell
---------------

**Element** DynamicWorkCell

**Attributes**


* **workcell:** a string identifying the workcell file.

**Child elements:**


* `Include`_
* `PhysicsEngine`_
* `Gravity`_
* `MaterialData`_
* `FrictionMap`_
* `ObjectTypeData`_
* `ContactMap`_
* `FixedBody`_
* `KinematicBody`_
* `RigidBody`_
* `KinematicDevice`_
* `RigidDevice`_
* `SuctionCup`_
* `Constraints`_
* `Springs`_
* `TactileArraySensor`_
* `BodyContactSensor`_
* `TactileMultiAxisSensor`_
* `FTSensor`_
* `PDDeviceController`_
* `PoseDeviceController`_
* `SerialDeviceController`_
* `SpringJointController`_

**Example**

.. code-block:: xml

   <DynamicWorkcell workcell="pih.wc.xml">
       ...
   </DynamicWorkcell>`

Include
-------

The dynamic workcell can be split into multiple files.
The extension of a included file should be .xml
(and not dwc.xml as it is not a valid dynamic workcell on its own).

**Element** Include

**Attributes**


* **file:** a string identifying the file to include.

**Example**

.. code-block:: xml

   <Include file="include.xml" />

The included file must have a root element called IncludeData: 

.. code-block:: xml

   <IncludeData>
       ...
   </IncludeData>

Everything inside the IncludeData element is interpreted as if it was inserted directly in the .dwc.xml
file instead of the Include tag.

PhysicsEngine
-------------

For dynamic simulation different Physics Engines can be used.
The PhysicsEngine element allows modification of parameters that are specific to the PhysicsEngine used.
See also rwsim::simulator::PhysicsEngine .

**Element** PhysicsEngine

**Child elements:**

* **Property**

------------

**Child element** Property

**Attributes**


* **name:** a string identifying the property.
* **type:** string (default), int, float or Q.

------------

**Example**
The following example shows an example with properties for the Open Dynamics Engine
(see rwsim::simulator::ODESimulator and ODE documentation for further information).
All properties are optional, and the example shows the default values used for ODE.

.. code-block:: xml

   <PhysicsEngine>
       <Property name="StepMethod">WorldStep</Property> <!-- WorldQuickStep/WorldStep/WorldFast1 -->
       <Property name="WorldCFM" type="float">0.0000001</Property>
       <Property name="WorldERP" type="float">0.2</Property>
       <Property name="SpaceType">Simple</Property> <!-- QuadTree/Simple/HashTable -->
       <Property name="MaxIterations" type="int">20</Property>
       <Property name="ContactSurfaceLayer" type="float">0.0001</Property>
       <Property name="MaxSepDistance" type="float">0.0005</Property>
       <Property name="MaxPenetration" type="float">0.0005</Property>
       <Property name="MaxCorrectingVelocity" type="float">0.1</Property>
   </PhysicsEngine>

For similar options for the RobWorkPhysicsEngine (RWPE), please see rwsimlibs::rwpe::RWPEIsland::getDefaultPropertyMap documentation.

Gravity
-------

Set the gravity in world coordinates.

**Element** Gravity

**Example**

.. code-block:: xml

   <Gravity>0 0 -9.82</Gravity>

Materials
---------

Each body in a dynamic workcell must be associated to a material and object type.
These determine how bodies interact with each other by defining friction and restitution.
The material database is made of the following four elements: 


* `MaterialData`_
* `FrictionMap`_
* `ObjectTypeData`_
* `ContactMap`_

In practice the material database can be big, and often the materials are defined in a separate file
(often named DynamicMaterialDataBase.xml) using the `Include`_.

MaterialData
^^^^^^^^^^^^

Defines names and an optional descriptions of all materials used in the dynamic workcell.
See also rwsim::dynamics::MaterialDataMap .

**Element** MaterialData

**Child elements:**


* **Default:** a string defining the default material for bodies where no material is defined explicitly.
* **Material**

------------

**Child element** Material

**Attributes**

* **id:** a string giving the name of the material.

**Child elements:**


* **Description:** (optional) a string with a description of the material.

------------

**Example**

The following example shows the definition of a single material.

.. code-block:: xml

   <MaterialData>
       <Default>Plastic</Default>
       <Material id="Plastic">
           <Description>Optional description</Description>
       </Material>
   </MaterialData>

FrictionMap
^^^^^^^^^^^

Definition of friction between different materials defined in `MaterialData`_.
See also rwsim::dynamics::FrictionData .

**Element** FrictionMap

**Child elements:**


* **Pair**

------------

**Child element** Pair

**Attributes**


* **first:** the material id of the first material.
* **second:** the material id of the second material.

**Child elements:**

* **FrictionData**

------------

**Child element** FrictionData

**Attributes**


* **type:** only Coulomb supported currently.

**Child elements:**


* **Mu:** a float giving the Coulomb friction coefficient.

------------

**Example**

.. code-block:: xml

   <FrictionMap>
       <Pair first="Plastic" second="Plastic">
           <FrictionData type="Coulomb">
               <Mu>0.4</Mu>
           </FrictionData>
       </Pair>
   </FrictionMap>

ObjectTypeData
^^^^^^^^^^^^^^

Definition of different object types.
This is primarily used for defining restitution between objects.
See also rwsim::dynamics::ContactDataMap .

**Element** ObjectTypeData

**Child elements:**


* **Default:** a string defining default object type for objects where no object types are set explicitly.
* **ObjectType**

------------

**Child element** ObjectType

**Attributes**

* **id:** a string identifying the type of object.

**Child elements:**


* **Description:** (optional) a string with a description of the type.

------------

**Example**

.. code-block:: xml

   <ObjectTypeData>
       <Default>hardObj</Default>
       <ObjectType id="hardObj">
           <Description>A hard object. with low elasticity</Description>
       </ObjectType>
   </ObjectTypeData>

ContactMap
^^^^^^^^^^

Definition of restitution coefficients between different object types
defined in `ObjectTypeData`_.
See also rwsim::dynamics::ContactDataMap::NewtonData .

**Element** ContactMap

**Child elements:**

* **Pair**

------------

**Child element** Pair

**Attributes**


* **first:** the type id of the first object type.
* **second:** the type id of the second object type.

**Child elements:**


* **ContactData**

------------

**Element** ContactData

**Attributes**


* **type:** only Newton supported currently.

**Child elements:**


* **cr:** a float giving the coefficient of restitution.

------------

**Example**

.. code-block:: xml

   <ContactMap>
       <Pair first="hardObj" second="hardObj">
           <ContactData type="Newton">
               <cr>0.0</cr>
           </ContactData>
       </Pair>
   </ContactMap>

Bodies
------

Bodies that should be a part of the simulation must be defined in the dynamic workcell.
There are three available types:


* `FixedBody`_:
  The FixedBody specifies bodies that are static and does not move. Such a body influences the motion
  of other bodies, but other bodies can not influence a FixedBody.
* `KinematicBody`_:
  A KinematicBody can be moved during simulation, and can affect other bodies. Other bodies can
  however not affect the velocity of the KinematicBody.
* `RigidBody`_:
  Rigid bodies has mass and inertia and the motion is determined by the forces acting on the body.

FixedBody
^^^^^^^^^

Fixed bodies will typically be static environment, such as floors and walls.
See also rwsim::dynamics::FixedBody .

**Element** FixedBody

**Attributes**


* **frame:** a string associating the body to a Frame defined in the workcell.

**Child elements:**


* **MaterialID:** (optional) a string giving the name of a material defined in `MaterialData`_.
* **ObjectID:** (optional) a string giving the name of a object type defined in `ObjectTypeData`_.
* **Property:** (optional) for additional data (Property tag defined in `PhysicsEngine`_)

**Example**

.. code-block:: xml

   <FixedBody frame="Floor" />

KinematicBody
^^^^^^^^^^^^^

Kinematic bodies can be controlled directly with velocities. See also rwsim::dynamics::KinematicBody .

**Element** KinematicBody

**Attributes**


* **frame:** a string associating the body to a *MovableFrame* defined in the workcell.

**Child elements:**


* **MaterialID:** (optional) a string giving the name of a material defined in `MaterialData`_.
* **ObjectID:** (optional) a string giving the name of a object type defined in `ObjectTypeData`_.

**Example**

.. code-block:: xml

   <KinematicBody frame="MovingBody">
       <MaterialID>Plastic</MaterialID>
   </KinematicBody>

RigidBody
^^^^^^^^^

Rigid bodies moves due to forces acting on the bodies. Hence they require specification
of dynamic parameters as mass and inertia. See also rwsim::dynamics::RigidBody .

**Element** RigidBody

**Attributes**


* **frame:** a string associating the body to a *MovableFrame* defined in the workcell.

**Child elements:**


* **Mass**: a float with the mass of the body.
* **EstimateInertia:** (optional) calculate Inertia and COG from geometry (geometry must be present in this case).
  If COG is given this will be used when calulating the Inertia.
* **COG:** (required if EstimateInertia is not used, else optional) the center of gravity.
* **Inertia:** (required if EstimateInertia is not used) the 3x3 inertia matrix of the body.
* **Integrator:** the integrator used (not used in ODE).
* **Associate:** (optional) allows associating geometry that is not attached to the main body frame to this body.
* **MaterialID:** (optional) a string giving the name of a material defined in `MaterialData`_.
* **ObjectID:** (optional) a string giving the name of a object type defined in `ObjectTypeData`_.
* **Property:** (optional) for additional data (Property tag defined in `PhysicsEngine`_)

------------

**Child element** Associate

**Attributes**


* **object:** a string identifying a object in the workcell to associate to this body.
  The geometry of the object is then added to this body.

------------

**Example**

.. code-block:: xml

   <RigidBody frame="DynBodyFrame">
       <Mass>0.1</Mass>
       <EstimateInertia />
       <Integrator>Euler</Integrator>
       <Associate object="DynBodyGeometry" />
   </RigidBody>

Devices
-------

Devices that should be a part of the simulation must be defined in the dynamic workcell.
There are two available types:


* `KinematicDevice`_:
  The KinematicDevice specifies a device that is composed of kinematic bodies. If dynamic simulation
  is not required this is the most efficient method to simulate a device.
* `RigidDevice`_:
  A RigidDevice is composed of rigid bodies, which have their motion constrained.

Please note that a Body that is part of a device is called a Link. A Body in the dynamic workcell format
referes to a body that is not part of a device (it is free). A Link is part of a device and has its motion
constrained. Because of this, the syntax for a Link and a Body is equivalent in practice. 

KinematicDevice
^^^^^^^^^^^^^^^

See also rwsim::dynamics::KinematicDevice .

**Element** KinematicDevice

**Attributes**

* **device:** a string associating the dynamic device to a JointDevice defined in the workcell.

**Child elements:**


* **FixedBase:** (one base element required)
  equivalent to `FixedBody`_.
* **KinematicBase:** (one base element required)
  equivalent to `KinematicBody`_.
* **RefBase:** (one base element required) use an existing body as base.
* **KinematicJoint/Link:** equivalent to `RigidBody`_ - note that mass parameters can just be set to zero.

------------

**Child element** RefBase

**Attributes**


* **body:** a string identifying an existing body.

------------

**Example**

.. code-block:: xml

   <KinematicDevice device="Robot">
       <KinematicBase frame="Base" />
       <KinematicJoint object="Joint0">
           <Mass>0</Mass> 
           <COG>0 0 0</COG>
           <Inertia>0 0 0 0 0 0 0 0 0</Inertia>
           <MaterialID>Plastic</MaterialID> 
       </KinematicJoint>
       ...
    </KinematicDevice>

RigidDevice
^^^^^^^^^^^

See also rwsim::dynamics::RigidDevice .

**Element** RigidDevice

**Attributes**

* **device:** a string associating the dynamic device to a JointDevice defined in the workcell.

**Child elements:**


* **FixedBase:** (one base element required)
  equivalent to `FixedBody`_.
* **KinematicBase:** (one base element required)
  equivalent to `KinematicBody`_.
* **RigidBase:** (one base element required)
  equivalent to `RigidBody`_.
* **RefBase:** (one base element required) use an existing body as base.
* **RigidJoint/Link:** equivalent to `RigidBody`_.
* **ForceLimit:** the force or torque applied by each motor (depending on the joint it refers to)

------------

**Child element** ForceLimit

**Attributes**

* **joint:** the joint to set limit for.

------------

**Example**

.. code-block:: xml

   <RigidDevice device="UR1">
       <ForceLimit joint="Joint0">1000</ForceLimit>
       <FixedBase frame="Base">
           <MaterialID>Plastic</MaterialID> 
       </FixedBase> 
       <Link object="Joint0">
           <Mass>3.8</Mass>
           <EstimateInertia/>
           <MaterialID>Plastic</MaterialID> 
       </Link> 
       ...
    </RigidDevice>

SuctionCup
----------

The SuctionCup element is not documented yet.

Constraints
-----------

The only way to constraint bodies is to use devices. In some cases it might however be
useful to constrain arbitrary bodies without requiring that the bodies are part of the same
device. This could for instance be if one wants to model a spring.
The Constraint element is the equivalent of a rwsim::dynamics::Constraint objects.

**Element** Constraint

**Attributes**


* **name:** a unique name for this constraint.
* **type:** one of the ContraintType values defined in rwsim::dynamics::Constraint (Fixed, Prismatic, Revolute, Universal, Spherical, Piston, PrismaticRotoid, PrismaticUniversal, Free)
* **parent:** the parent body.
* **child:** the child body.

**Child elements:**


* **Transform3D:** (optional) where the constraint acts relative to the parent body frame.
* **Spring:** (optional) adds springs in the non-constrained directions (see `Springs`_).

Note the Spring element can be defined under the Constraint or after the Constraint with a reference to the name
of the constraint.

**Example** of a Fixed constraint:

.. code-block:: xml

   <Constraint name="FixedConstraint" type="Fixed" parent="Parent" child="Child" />

**Example** of a non-fixed constraint (the spring works for the one linear and two angular directions that are not constrained by the PrismaticUniversal constraint):

.. code-block:: xml

   <Constraint name="ComplianceConstraint" type="PrismaticUniversal" parent="Parent" child="Child">
       <Transform3D>
           <Pos>0 0 0.01</Pos>
           <RPY>0 0 0</RPY>
       </Transform3D>
       <Spring>
           <Compliance>
               0.0005 0 0
               0 0.5 0
               0 0 0.5
           </Compliance>
           <Damping>
               50 0 0
               0 0.1 0
               0 0  0.1
           </Damping>
       </Spring>
   </Constraint>

Springs
-------

It is possible to attach a spring to a constraint as described in `Constraints`_.
If the spring is defined outside the Constraint tags, it must refer to a constraint by name.

**Element** Spring

**Attributes**


* **constraint:** the name of the constraint to attach the string to (the string works in the non-constrained directions).

**Child elements:**


* **Compliance:** n times n compliance matrix where n is the number of non-constrained dimensions (between 0 and 6 according to constraint type).
* **Damping:** matrix of same dimensions as the compliance matrix.

**Example** of a spring attached to a PrismaticUniversal constraint
(the spring works for the one linear and two angular directions that are not constrained by the PrismaticUniversal constraint):

.. code-block:: xml

   <Spring constraint="ComplianceConstraint">
       <Compliance>
           0.0005 0 0
           0 0.5 0
           0 0 0.5
       </Compliance>
       <Damping>
           50 0 0
           0 0.1 0
           0 0  0.1
       </Damping>
   </Spring>

Sensors
-------

Not documented yet.

TactileArraySensor
^^^^^^^^^^^^^^^^^^

BodyContactSensor
^^^^^^^^^^^^^^^^^

TactileMultiAxisSensor
^^^^^^^^^^^^^^^^^^^^^^

FTSensor
^^^^^^^^

Controllers
-----------

Not documented yet.

PDDeviceController
^^^^^^^^^^^^^^^^^^

PoseDeviceController
^^^^^^^^^^^^^^^^^^^^

SerialDeviceController
^^^^^^^^^^^^^^^^^^^^^^

SpringJointController
^^^^^^^^^^^^^^^^^^^^^
