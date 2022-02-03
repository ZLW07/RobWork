.. _plugins_sdurws:

**************************
GUI Plugins: RobWorkStudio
**************************

The RobWorkStudio GUI is based on Qt, and through the use of Qt plugins,
it is possible to extend RobWorkStudio with user-defined features and behavior.
Notice that this is very different from the RobWork plugins that makes it possible to extend
the core libraries with new features.
Please see :ref:`plugins_sdurw` for more information about that subject.

GUI plugins must inherit the rws::RobWorkStudioPlugin class
(see `C++ API <../../apidoc/cpp/doxygen/classrws_1_1RobWorkStudioPlugin.html>`__)
and implement at least some of its functions.
In this section we present two common project templates for creating a GUI plugin:

- A simple template where Qt gui functionality needs to be added in the hpp/cpp plugin files.
- An example where Qt designer is used to create an ui file which describe the graphical layout.

Common to both templates is how to load the plugin into RobWorkStudio when they have compiled.
The compiled output will be a dynamically linkable file (in Windows it is a .dll file and in Linux it is .so).
To use the plugin in your RobWorkStudio installation add the following lines to the
RobWorkStudio.ini file in the directory where you start RobWorkStudio from::
 
	SamplePlugin\DockArea=2
	SamplePlugin\Filename=libSamplePlugin
	SamplePlugin\Path=PATH_TO_PLUGIN
	SamplePlugin\Visible=true  

Without Qt Designer
===================

This template can be found in the folder RobWorkStudio/example/pluginapp . The files include:

- **SamplePlugin.hpp**: the header file of the plugin
- **SamplePlugin.cpp**: the source file of the plugin
- **SamplePlugin.json**: used by Qt5 for metainformation about the plugin (name, version etc.)
- **resources.qrc**: a Qt resource file that enables compiling images directly into exe/dll/so
- **pa_icon.png**: a sample icon used in the toolbar of RobWorkStudio
- **CMakeLists.txt**: the CMake project file

This example shows how to create a plugin without the use of a ui file from Qt Designer.
The header file for the plugin is shown below. The plugin must inherit the RobWorkStudioPlugin class.
The *open* and *close* functions is called when a WorkCell is opened or closed in RobWorkStudio.
The *initialize* function is called initially when the RobWorkStudio instance is valid.
It can be used to initialize values that depend on the RobWorkStudio instance.

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.hpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.hpp

In the hpp file the Q_PLUGIN_METADATA macro refers to the file SamplePlugin.json.
This file must look like the following:

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.json
   :language: json
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.json

The implementation of the plugin is shown below.
In the constructor two pushbuttons are added to the plugin. Each button is connected to the clickEvent function.
Qt operates with signals and slots, and in this case the clicked() signal from the pushbutton is connect to the clickEvent() slot defined in the plugin.
In the initialize function, a *stateChangedListener* is registered in RobWorkStudio.
This function will be called every time the state changes.
In the clickEvent() function we can do different computations depending on the which button was pushed. This can be determined based on the Qt sender().

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/SamplePlugin.cpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/SamplePlugin.cpp

In the cpp file the constructor refers to the path ":/pa_icon.png".
This is using the Qt resource system. A resources.qrc file is used to define the files that can be used in the plugin.
By using this system, the graphics will be compiled into the plugin binary (the .dll or .so files).
It is up to the user to define a plugin icon.

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/resources.qrc
   :language: xml
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/resources.qrc

The CMake file used for compiling the plugin is shown below:

.. literalinclude:: ../../../RobWorkStudio/example/pluginapp/CMakeLists.txt
   :language: cmake
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/CMakeLists.txt

The CMake script will look for RobWork and RobWorkStudio, and it will set the output directories for libraries and executables.
In the last few lines, the CMAKE_AUTOMOC and CMAKE_AUTORCC is enabled, before the library is added and linked to RobWork and RobWorkStudio.
AUTOMOC and AUTORCC are CMake features that takes care of some Qt specific stuff automatically.

Before running CMake, you should set the RW_ROOT and RWS_ROOT environment variables to the path for RobWork and RobWorkStudio respectively.

To compile the plugin you should first create a separate build directory and run CMake from there::

   cmake -DCMAKE_BUILD_TYPE=Release path/to/plugin

On Windows you should specify the generator::

   cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 15 2017 Win64" path/to/plugin

You need to set the build type to the same build type that you used for compiling RobWork. Insert the correct path to the plugin code.

Build the plugin with make (on Linux)::

   make

In Windows, open the SamplePlugin solution in Visual Studio (solution was generated with CMake).

When the .dll or .so file is generated it can be loaded from the RobWorkStudio menu Plugins->Load Plugin.
Alternatively it can be added to RobWorkStudio.ini to be automatically loaded at startup.

With Qt Designer
================

This template can be found in the folder RobWorkStudio/example/pluginUIapp.
It involves the same files as the template `Without Qt Designer`_, but with the additional file SamplePlugin.ui.
The .ui file is a Qt Designer file.
Qt Designer makes it possible to develop the graphical layout of the plugin in a drag and drop fashion.
For more complex plugins this is much more intuitive, than programming it up in C++.

In the following we will discuss the changes compared to the template `Without Qt Designer`_,
so please refer to this section for more details.

The header file for the plugin is shown below.
Compared to the template `Without Qt Designer`_, the header file does not have QPushButton objects defined as private members.
These buttons are instead defined in the .ui file.
By including *ui_SamplePlugin.hpp* and inheriting from *Ui::SamplePlugin*, these buttons can be accessed from our C++ code.

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/SamplePlugin.hpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginUIapp/SamplePlugin.hpp

The implementation of the plugin is shown below.
The constructor is more simple, since the buttons are easily set up with the setupUi(this) call.
All we need to do is to connect the signals and slots.

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/SamplePlugin.cpp
   :language: c++
   :linenos:
   :caption: RobWorkStudio/example/pluginUIapp/SamplePlugin.cpp

The CMake file used for compiling the plugin is shown below:

.. literalinclude:: ../../../RobWorkStudio/example/pluginUIapp/CMakeLists.txt
   :language: cmake
   :linenos:
   :caption: RobWorkStudio/example/pluginapp/CMakeLists.txt

The CMake script is almost identical to the one for the `Without Qt Designer`_ template.
Since we have .ui files, we also enable the CMAKE_AUTOUIC feature.
This way CMake and Qt automatically generates the ui_SamplePlugin.h file that we include in the SamplePlugin.hpp file.

With Python
===========

Plugins can also be made, in form of a python script. 
To Enable this feature Python3-dev must be installed on the system,
before compiling RobWorkStudio.
Python plugins are in actuality a c++ plugin with a python interpreter loaded into it.
To allow modification of the pluginGUI the c++ plugin starts by running a script in the python interpreter,
before calling the user written python plugin.

.. code-block:: python

    from sdurws import *
    import sys
    from PySide2 import QtCore

    class cpp_link(QtCore.QObject): ... #Class used by the cpp plugin to make calls to python plugins
    rws_cpp_link = cpp_link #           #Object used to facilitate the calls

    class rwsplugin(QtCore.QObject):    #inherit from this class to access RobworkStudio and QtWidget
        def __init__(self,link): 
        def getWidget(self):            #Returns a widget to be used by PiSide2
        def getRobWorkStudio(self):     #Returns a pointer to RobworkStudio
        def getWorkCell(self):          #Returns a pointer to the currently loaded WorkCell
        def log(self):                  #Returns a pointer to the current Log




As can be seen from the code it relies on PySide2 as a wrapper to qt. So to make a python plugin
PySide must be installed.

.. code-block:: bash

    pip3 install PySide2

PySide2 often relies on the newest version of QT.
To test if you have the right version open a python interpreter and run:

.. code-block:: python

    from PySide2 import QtCore

If something is wrong this should give you an error with incompatible QT versions.
To get the correct Version of QT you need to do a proper install and get the required QT version.
This can be done by downloading the QT installer from https://www.qt.io/download/ .
At the Select component phase of the installation,
make sure that you are installing the version required by PySide2.

Ones the correct version of QT is installed, if PySide still gives the same error,
it means that it can't find the correct QT installation.
For Linux users the path can be specified by the QT library path to LD_LIBRARY_PATH.
This can be done by adding the following line to .bashrc or calling the line from the terminal

.. code-block:: shell

    #example -------------||----------------/Path/To/Qt/<Version_number>/<compiler>/lib
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/Qt/5.13.1/gcc_64/lib

With this, the setup is complete and you should be able to run your own or one of the example plugins for python.


Designing a python plugin
--------------------------

Make sure that the Python file used is called something unique,
as currently the pluginWidget is named the same as the python file,
and is found by searching for it's name when calling rwsplugin.getWidget().

The simplest plugin, that can be made with full functionality, can be seen in the code below.
Using the rws_cpp_link, three different methods is called, in the registered plugin(if they exist).
Inheriting from the rwsplugin class, gives you the some of the same functionality as a cpp plugin

.. code-block:: python

    class samplePlugin(rwsplugin):
      def __init__(self,link):
        super().__init__(link)              #initialize the underlying rwsplugin
        self.widget = super().getWidget()   #get the widget
      def open(self,workcell):              #this method is called by rws_cpp_link when a new workcell is opened in RobWorkStudio
        pass
      def close(self):                      #this method is called by rws_cpp_link when current workcell is closed
        pass
      def stateChangedListener(self,state): #this method is called by rws_cpp_link when the state changes in RobWorkStudio
      pass

    if __name__ == '__main__':
      pl = Plugin(rws_cpp_link)
      rws_cpp_link.register_plugin(pl)

Due to the rwsplugin and rws_cpp_link is automatically imported upon loading a python plugin,
the above plugin runs without importing any modules

Tips & Examples
===============

Here are some small useful examples that can be used from a plugin

Get CollisionDetector
---------------------------------------

Get the collision detector that is currently used in your RobWorkStudio instance:

.. code-block:: c++

   CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();

Communication between plugins
---------------------------------------

RobWorkStudio has a number of events which can be used by the plugins. A plugin can register for an event, for example by:

.. code-block:: c++

   getRobWorkStudio()->stateChangedEvent().add(boost::bind(&MyPlugin::stateChangedListener, this,_1), this);

which binds the stateChangedListener method of MyPlugin to listen for state changed event.

To see more information about the different event please consult the RobWorkStudio
`API documentation <../../apidoc/cpp/doxygen/classrws_1_1RobWorkStudio.html>`__.

Adding new frames to the workcell from a plugin
-----------------------------------------------

This example describe how one can add his own frames to the workcell through
a user plugin.

Adding frames to the workcell is possible through the StateStructure instance that
is located in a WorkCell. It is important to understand that adding frames to the
state structure will change the static state structure of the workcell (the dynamic state is that
which is located in the State object). Changing the static structure will not directly influence
State objects, that is they are still valid for all frames except the newly added frames.
There exist two methods of making an old state valid for new frames. One is to just assign
the old state with a new one. Though, this will also overwrite any state information that was
saved in the old state, say the configuration of your robot. If you want to preserve the information
in the old state and still make it valid for newly added frames you would need to upgrade it. You
can upgrade a state **oldstate** using either StateStructure instance **stateStruct** or another
state **newstate**. Following is an example of how:

.. code-block:: c++

   // using another state to upgrade
   oldstate.upgradeTo(newstate); // oldstate is upgraded to the structure of the newstate
   // using state structure to upgrade
   oldstate = stateStruct.upgrade( oldstate );

Following is an example of how to add a new frame to the workcell from your own plugin

.. code-block:: c++

   State oldState; // this is your old state
   Frame *newFrame = make_new_frame(); // create your frame
   getRobWorkStudio()->getWorkcell()->getStructure()->addFrame(newFrame,parentFrame);
   // now update the oldState with the new state
   oldState = getRobWorkStudio()->getWorkCell()->getStructure()->upgradeState(oldState);
   // now this is VERY important, remember to update the RobWorkStudio state
   getRobWorkStudio()->setState(oldState);


Adding Drawable from a plugin
-----------------------------

Before considering the example, a bit of background is needed about the RobWork and RobWorkStudio scene model.

The most abstract model of a scene in RobWork is defined in rw::graphics. It is based on the SceneGraph model.
SceneGraph defines elements in a scene for visualization.
It is independent from the usual RobWork structure of frames. As such it is a more generic model of a scene.
A SceneGraph contains:

- CameraGroups: A group of cameras can be used, for instance, to render background, render scene elements and render foreground.
  Each camera in the group is a SceneCamera (see below).
- Tree of SceneNodes with a root GroupNode.

  - GroupNode: Groups other SceneNodes (it is a non-leaf node)
  - DrawableNode: A node with something that can be drawn in the scene.
    It has a draw mask to distinguish between different groups to be shown in the scene.

    - DrawableGeometryNode: A DrawableNode that draws a rw::geometry::Geometry.
    - DrawableNodeClone: A node that reuses another DrawableNodes data, but has its own visualization settings.

  - SceneCamera: A node representing a camera in the scene.
  - Other user defined node types

WorkCellScene is a wrapper for the more generic SceneGraph.
It uses the Frame structure from a WorkCell and State, to maintain a corresponding underlying SceneGraph.
In the WorkCellScene, DrawableNodes can be added and retrieved for each Frame in the WorkCell.
It has convenience functions, for instance, for adding Renders or Drawables and Geometries from files.
WorkCellScene keeps track of the visualization of frames and visibility of frames in the WorkCell.

SceneViewer is an interface for the actual visualization of a SceneGraph. Implementations should allow for:

  - A main view: The SceneCamera that is mainly used to visualize the scene.
  - A current view: The currently selected view.
  - Function for updating view(s).
  - Zoom functions.
  - WorkCellScene (optional): if the SceneGraph is wrapped by a WorkCellScene.

The code for adding a Drawable from a plugin could look like the following (here we make a DrawableNode for a Render):

.. code-block:: c++

   using namespace rw::graphics;
   using rw::kinematics::Frame;
   using rws::RWStudioView3D;

   const RWStudioView3D::Ptr rwsview = getRobWorkStudio()->getView();
   const SceneViewer::Ptr viewer = rwsview->getSceneViewer();
   const SceneGraph::Ptr graph = viewer->getScene();
   const WorkCellScene::Ptr wcscene = rwsview->getWorkCellScene();

   const Render::Ptr render = ownedPtr(new MyRender());
   Frame* const myFrame = getRobWorkStudio()->getWorkCell()->findFrame("myFrame");
   
   // Method 1
   const DrawableNode::Ptr drawableNode = graph->makeDrawable("Render", render, DrawableNode::Physical);
   wcscene->addDrawable(drawableNode, myFrame);
   
   // Method 2 (use convenience method on WorkCellScene)
   wcscene->addRender("Render", render, myFrame, DrawableNode::Physical);

The RWStudioView3D is the main visualization area in RobWorkStudio.
This visualization uses the SceneViewer model (with the underlying SceneGraph).
The SceneGraph model is wrapped in a WorkCellScene that can also be retrieved from RWStudioView3D.
The WorkCellScene takes care of maintaining the underlying SceneGraph.
In RobWorkStudio, getWorkCellScene actually returns a SceneOpenGLViewer.
SceneOpenGLViewer is a specialization that uses Qt and OpenGL.
It adds background to the scene, the pivot point and the default RobWorkStudio view.
It also controls which DrawableNode types to draw (Physical, Virtual etc) based on the chosen settings in RobWorkStudio.
First step is to retrieve these entities from the RWStudioView3D.

Next, we create a render. Render is an interface that defines a draw function.
It is up to the user to implement the Render class.
In rwlibs::opengl, many Render types are implemented for drawing in OpenGL (as we currently use in RobWorkStudio).
This is types for rendering images, geometries, point-clouds, lines, vectors, and much more.

A DrawableNode is then created in the SceneGraph for the Render.
This node is not yet connected to any child or parent nodes.
By calling addDrawable on the WorkCellScene,
we let the WorkCellScene take care of attaching the DrawableNode correctly to the tree,
based on the Frame given to addDrawable.

Notice that the code is completely independent from OpenGL (except if we create the Render from types in rwlibs::opengl).
The framework would allow RobWorkStudio to change to something else than OpenGL, and our plugin would still work.

Getting Drawables from a Frame
------------------------------

This code snippet will copy all Drawables associated with the Frame **frameWithDrawables**
into the vector **drawables**:

.. code-block:: c++

   using namespace rw::graphics;
   using rw::kinematics::Frame;
   using rws::RWStudioView3D;

   const RWStudioView3D::Ptr rwsview = getRobWorkStudio()->getView();
   const WorkCellScene::Ptr wcscene = rwsview->getWorkCellScene();

   Frame *frameWithDrawables; // specify the frame where your drawables are placed
   std::vector<DrawableNode::Ptr> drawables = wcscene->getDrawables(frameWithDrawables);

The next code snippet will copy all drawables associated to any frame in the WorkCell
into the vector **drawables**.

.. code-block:: c++

   using namespace rw::graphics;
   using rws::RWStudioView3D;

   const RWStudioView3D::Ptr rwsview = getRobWorkStudio()->getView();
   const WorkCellScene::Ptr wcscene = rwsview->getWorkCellScene();

   std::vector<DrawableNode::Ptr> drawables = wcscene->getDrawables();
