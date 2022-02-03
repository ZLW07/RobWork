User Manual (C++)  {#page_rw_manual}
=================
[TOC]

This manual contains the remaning Doxygen documentation not yet moved to Sphinx documentation.

# RobWork Task Format # {#sec_rw_manual_task}
RobWork includes an abstract task format which can be used to represent, save and 
load tasks. The basic rwlibs::task::Task is templated and can either store 
rw::math::Q or rw::math::Transform3D as targets. 

A task in RobWork is basically a 2-tuple which can be described as
~~~~~{.cpp}
Task={(Target)*, (Motion|Action|Task)*}
~~~~~

The elements in a task are
- *Target*: Typically representing either a Cartesian pose or a robot configuration using rw::math::Transform3D and rw::math::Q, respectively.
- *Motion*: Describes a motion/transition between targets. A target may be shared among any number of motions.
- *Action*: Has no fixed interpretation and can be used to specify events such as open/close gripper, acquire image or as synchronization point.
- *Task*: Tasks are recursive. Subtasks may be shared among multiple tasks.


The example below illustrated how to construct a small task, prints out the task, saves it to file, reloads it and prints it once again.

\include ex-task.cpp



# RobWorkStudio # {#sec_rws_manual_intro}
Se Sphinx documentation, this is remaning Doxygen documentation not yet moved to Sphinx documentation.

## RobWorkStudio specific frame properties ## {#sec_rws_properties}

Through generic properties in the XML workcell file format, RobWork allows
for adding user specific information to frames. In this section RobWorkStudio specific
properties will be listed. Meaning properties that only makes sence for RobWorkStudio and
not RobWork.

### Camera property ### {#sec_rwstudio_camera_property}
A property describing a camera pinhole model can be added to a frame. The camera view can
then be visualized in RobWorkStudio. The property string looks like this:
\verbatim
"<Field of view Y> <width> <height>"
\endverbatim
example:
\verbatim
<Property name="Camera">60 640 480</Property>
\endverbatim

You can currently only change views between cameras using Ctrl + the key [1-9], were 1 is the default
3rd person view.

\b Important!
- Multiple cameras are supported but only one camera property per frame!
- The width and height has no real dimension its the proportion between them that matters
- The camera looks in the negative Z-axis direction of the frame
- Field of view is in degree and is defined in the Y-axis

## Usefull examples ## {#sec_rws_examples}

### Adding new frames to the workcell from a plugin ### {#subsec_rws_examples_adding_new_frames}
 This example describe how one can add his own frames to the workcell through
 a user plugin.

 Adding frames to the workcell is possible through the StateStructure instance that
 is located in a WorkCell. It is important to understand that adding frames to the
 state structure will change the static state structure of the workcell (the dynamic state is that
 which is located in the State object). Changing the static structure will not directly influence
 State objects, that is they are still valid for all frames except the newly added frames.
 There exist two methods of making an old state valid for new frames. One is to just assign
 the old state with a new one. Though, this will also overwrite any state information that was
 saved in the old state, say the configuration of tour robot. If you want to preserve the information
 in the old state and still make it valid for newly added frames you would need to upgrade it. You
 can upgrade a state \b oldstate using either StateStructure instance \b stateStruct or another
 state \b newstate. Following is an example of how:
~~~~~{.cpp}
    // using another state to upgrade
    oldstate.upgradeTo(newstate); // oldstate is upgraded to the structure of the newstate
    // using state structure to upgrade
    oldstate = stateStruct.upgrade( oldstate );
~~~~~

Following is an example of how to add a new frame to the workcell from your own plugin
~~~~~{.cpp}
    State oldState; // this is your old state
    Frame *newFrame = make_new_frame(); // create your frame
    getRobWorkStudio()->getWorkcell()->getStructure()->addFrame(newFrame,parentFrame);
    // now update the oldState with the new state
    oldState = getRobWorkStudio()->getWorkCell()->getStructure()->upgradeState(oldState);
    // now this is VERY important, remember to update the RobWorkStudio state
    getRobWorkStudio()->setState(oldState);
~~~~~


# RobWorkSim #

## Introduction ##
\b RobWorkSim is a dynamic simulation framework in C++ developed as an add-on for RobWork and RobWorkStudio.
RobWorkSim is used for research and education as well
as for practical robot applications. Features of the library include:

- Dynamic modeling of various types of industrial manipulators.
- Grasp table generation for grasp planning applications.
- Simulation of tactile sensors as well as all sensors supported by RobWork (Vision and Range scanners)
- Resting pose calculations
- Stable configuration calculations
.

Target audience of RobWorkSim is:
- Implementers who needs a framework for process simulation or validation of algorithms

RobWorkSim is developed at the <a
href="http://www.mip.sdu.dk/robotics">robotics department</a> of the
<a href="http://www.sdu.dk/mmmi">Maersk McKinney Moller Institute</a>
at the <a href="http://www.sdu.dk">University of Southern
Denmark</a>. The focus of the department is on industrial robots and
their applications.


### Namespaces ### {#sec_rwsim_namespaces}

The header files of RobWorkSim are distributed across a number of
directories each having its own namespace. The structure of namespaces
reflects the directory containing the code. For example

~~~~~{.cpp}
// Include header files:
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/control/PDController.hpp>

using namespace rwsim::dynamics;
using namespace rwsim::PDController;
~~~~~

This structure is the same as RobWork and RobWorkStudio.

### Libraries ### {#sec_rwsim_libraries}


### Install and Use ### {#sec_rwsim_install_and_use}

Functionality in RobWorkSim depends heavilly on RobWork and RobworkStudio for GUI and specific plugins.
As such, it is recommended to install these before installing RobWorkSim.

## Concepts and Overview ## {#sec_rwsim_consepts}

The primary use of RobWorkSim evolves around specifying a DynamicWorkCell (scene with
dynamic information) from which a Simulator instance is created which then is used
to do the actual simulation.

The DynamicWorkCell is conceptually the same as the RobWork WorkCell class and extends
the WorkCell description
with focus on describing the dynamic properties of the scene. It is basically a container
that includes a hierarchy description of the scene including: bodies, obstacles, frames,
devices, controllers, sensors and their mutual attachment to each other.

The DynamicWorkCell is "stateless" in the same sense that the WorkCell is stateless, which
means that typical state values such as force of a rigid body are saved in a state structure
and not in the actual object. The following code snippet exemplifies this:

~~~~~{.cpp}
RigidBody *b1 = getBody1(); // illustrative function "getBody1()"
State stateA = getState();
State stateB = getState();
b1->setForce( Vector3D<>(0,0,1), stateA );
b1->setForce( Vector3D<>(2,2,2), stateB );
std::cout << b1->getForce(stateA); // prints (0,0,1)
std::cout << b1->getForce(stateB); // prints (2,2,2)
~~~~~

Not all variables of our "stateless" objects are saved in the state structure since
they are considered to change infrequently. An example of this is getMass() on RigidBody.
As such a rule of thumb is that frequently changing variables such as position, velocity
and force will allways be saved in the state structure. Infrequently changing variables
will be saved in the object instance, e.g. mass, material info, geometry, nr of joints,
position limits, force limits and so on.

The stateless nature of DynamicWorkCell makes it possible to use it in multiple threads
or methods at the same time and without bothering with cloning and copying of the
DynamicWorkCell. However, one should be carefull to change the "static" variables when
using multiple threads since these changes will influence all uses of the variable. For
more indepth description of the StateStructure the reader is directed to the RobWork manual.

Now the DynamicWorkCell can be constructed in c++ or as is done more often through the
XML based DynamicWorkCell file format described in section \ref sec_rwsim_xml_fileformat.
A Simulator is created with an instance of the DynamicWorkCell and is then ready for use.
A typical use is exemplified below:

~~~~~{.cpp}
// create and initialize simulator
DynamicWorkCell::Ptr dwc = getDynamicWorkCell();
DynamicSimulator *sim = makeSimulator( );
sim->initPhysics( dwc );
// set the current state
sim->resetState( initState );
// now do a simulation
while( someStopCriteria ){
  // apply forces/velocities to bodies and devices using controllers
  sim->step( 0.01, state);
  // monitor contacts and states using sensors or the State
}
// do something usefull with "resting" state
~~~~~

The Simulator is not stateless and to do simulations in parallel you should create
multiple instances of the simulator.

The simulation is run one step at the time using relatively small timesteps e.g.
[0.001s;0.01s]. The "best" timestep depends on the underlying physics engine,
the current scene, and the application. Please look at section \ref sec_rwsim_simulator
for more information.

There are two constructs designed for getting feedback and influencing the simulation.
These are the SimulatedSensor and the SimulatedController. The controller enables
"control" of bodies and devices or other states in the simulation, where as the sensor
enables getting appropriate feedback, e.g. tactile, visual or other states. Typically
used methods such as applying forces to bodies or setting the velocity of a device
are available on the Body/Device interface and does not require controllers or sensors.

## DynamicWorkCell ## {#sec_rwsim_dynamic_workcell}

## The DynamicSimulator ## {#sec_rwsim_simulator}
Timestep, ThreadSimulator, PhysicsEngine, PhysicsEngineFactory, EnableBody,

The simulation loop

