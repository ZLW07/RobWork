***********************
Programming Conventions
***********************

In the following different programming conventions are introduced.
These apply mainly to C++ code. For details about other languages, see also the :ref:`interfaces` section.

Namespaces
==========

The header files of RobWork are distributed across a number of
directories each having its own namespace. The structure of namespaces 
reflects the directory containing the code. For example

.. code-block:: c++

   // Include header files:
   #include <rw/models/WorkCell.hpp>
   #include <rw/kinematics/Frame.hpp>
   using namespace rw::models; //Namespace for WorkCell included by #include<rw/models/WorkCell.hpp>
   using namespace rw::kinematics; //Namespace for Frame included by #include <rw/kinematics/Frame.hpp>

All classes related to the RobWorkStudio package are placed in a namespace rws. All classes related to RobWorkHardware are in the namespace rwhw;

Libraries
=========

All classes of the *rw* directory are provided in a single library named *rw*.

The subdirectories of the *rwlibs* directory each correspond to a
different library. The subdirectory *rwlibs/xyz* corresponds to the
library named *rw_xyz* and contains the objects in the namespace rwlibs::xyz. For example, suppose your program contains
the following include statement:

.. code-block:: c++

   #include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>

To build this program, you should link with *rw_pathplanners*.

C++ shared pointer conventions
==============================

The **RobWork** libraries make extensive use of non-copyable objects
(such as object referred to by interface) shared by pointer between
different algorithms. Ownership of objects is managed by the shared
pointer type rw::common::Ptr. If an object needs access to a
non-copyable object, the constructor of the object will conventionally
take a rw::common::Ptr type as parameter.

Classes that are commonly referred to by shared pointer define a
shortcut for this pointer type. If the class is named *T*, the name
of the pointer type will be *T::Ptr*, and the type of the pointer will
be rw::common::Ptr<T>. Often there will also be a *T::CPtr* defined
for pointers to constant objects that can not be modified.
This will give the type rw::common::Ptr<const T>.

Here are some examples of these such pointer types:

- rw::math::QMetric::Ptr
- rw::models::WorkCell::Ptr
- rw::proximity::CollisionDetector::Ptr
- rw::pathplanning::QSampler::Ptr
- rw::pathplanning::QToQPlanner::Ptr

Here are some examples of constructor functions for such objects:

- rw::math::MetricFactory::makeEuclidean()
- rw::proximity::CollisionDetector::make()
- rw::pathplanning::QSampler::makeUniform()
- rwlibs::pathplanners::RRTPlanner::makeQToQPlanner()

The rw::common::Ptr type differs from standard shared pointer
implementations by allowing the pointer to refer to a stack allocated
object or another object for which an entity has already taken
the full ownership. To make the use of such objects easy, a
pointer to *T* can be implicitly converted to Ptr<T>, but the
implicitly constructed rw::common::Ptr type *does not* take
ownership of the object. If the rw::common::Ptr type should take
ownership of the entity, you must explicitly call the
rw::common::ownedPtr() function. This example illustrates the idiom:

.. literalinclude:: ../../../RobWork/example/snippets/ex-owned-ptr.cpp
   :language: c++
   :linenos:

In everyday programming, the construction of rw::common::Ptr types is
managed by the constructor functions for the various objects. Only if
you write your own extensions for interfaces in **RobWork** will you
need to explicitly call rw::common::ownedPtr().

Serialization
=============

There are two main structures for performing serialization in RobWork. 

First, there is a XML DOM
parser interface that lets you easily read and write XML data. This interface is 
used throughout RobWork for reading and writing well defined data exchange formats. The XML DOM
parser should not be used as a class serialization structure but rather in properly defined 
loader and saver factories. The XML DOM parser is especially useful as an extension mechanism 
for current parsing formats, eg. the user can write a plugin that plugs into an existing parsing 
implementation such as the workcell format. 

Secondly, there is 
a generic serialization framework which enables serialization of classes. This framework enables 
serialization to different archive types and can be used for serializing many different formats, including
binary and user defined formats.  

XML DOM parsing
---------------

The backend to the XML DOM parser is 
pluginable and the full features such as validation is therefore depending on the backend. 
Both boost property tree and xercess backends are supported.     

Here is a small example on creating a DOM parser which parses from an input stream (instream)  

.. code-block:: c++

   DOMParser::Ptr parser = DOMParser::make();
   parser->setSchema(schemaFileName);
   parser->load(instream);
   DOMElem::Ptr root = parser->getRootElement();

The **root** class can now be used for parsing the xml data. Lets assume we have a xml file that looks like:

.. code-block:: xml

   <group someboolproperty="true" someintproperty="10">
     <elem-string> </elem-string>
     <elem-int> </elem-int>
     <elem-double-list>0.1 0.2 0.1</elem-double-list> 
   </group>

.. code-block:: c++

   DOMELem::Ptr gelem = root->getElement("group"); // throws an error if "group" is not there
   bool boolprop = gelem->getAttributeValueAsBool("someboolproperty", true); // defaults to true
   int intprop = element->getAttributeValueAsInt("someintproperty", 2); // defaults to 2
   // iterate over all elements in group
   for(DOMElem::Ptr child : gelem->getChildren()) {
       if(child->isName("elem-string")) {
           std::string str-value = child->getValue();
       } else if (child->isName("elem-int")) {
           int int-val = child->getValueAsInt();
       } else if (child->isName("elem-double-list")) {
           std::vector<double> double-val-list = child->getValueAsDoubleList(); // default seperator is space ' '			
       }
    }

Generic serialization
---------------------

The generic serialization is not a centralized controlled serialization as the XML
serialization described above. There are two main concepts to the generic
serialization: Archive, Serializable

An Archive is an instance that formats data that can be read or written to it from a 
Serializable object/class. As such interfaces for both InputArchive and 
OutputArchive exists. A serializable class is a class that either inherits from 
the rw::common::Serializable (intrusive) or using an extrusive way that overwrites the functions

.. code-block:: c++

   void rw::common::serialization::write(const THE_CLASS&, OutputArchive& oar);
   void rw::common::serialization::read(THE_CLASS&, InputArchive& iar);

In any case save and load methods should be determined for the individual 
serializable classes. In these methods read/write operations on the archive should
be performed such as to store or restore the state of the class. The use of 
an archive can be quite simple:

.. code-block:: c++

   INIArchive ar(std::cout);
   ar.write(1, "length");
   ar.write(1.120, "width");
   ar.write(5.120, "height");
   produce:
   length : 1
   width : 1,120
   height : 5.120

As can be seen an identifier can be associated to the value that is to be serialized. This 
id makes type checking possible and should allways be used/defined. The archive interface 
define serialization of primitives (int, float, double, string) and vectors of primitives.
More complex types will need to implement their own serialization pieced together from 
the primitives.

Finally, the archive also defines a way to enter scopes. A scope is simply a grouping of 
value-identifier pairs. 

.. code-block:: c++

   void write(const KDTreeQ<VALUE_TYPE>& out, OutputArchive& oarchive) {
       oarchive.write(out._dim, "dim");
       oarchive.write((int)out._nodes->size(), "nrNodes");
       oarchive.write((boost::uint64_t)out._root, "rootId");
       RW_ASSERT(out._nrOfNodes==out._nodes->size());
       for(int i=0;i<out._nodes->size();i++) {
           const TreeNode &node = (*out._nodes)[i];
           oarchive.write((boost::uint64_t)&node, "id");
           oarchive.write( node._axis, "axis");
           oarchive.write( node._deleted, "del");
           oarchive.write( node._left, "left");
           oarchive.write( node._right, "right");
           oarchive.write( node._kdnode->key, "Q");
           oarchive.write( node._kdnode->value, "value");
       }
   }

Archive types
-------------

There is currently:
* an rw::common::INIArchive which prints in INI format
* an rw::common::BINArchive which prints in binary compressed format 


Threading
=========

When developing code that can run in parallel it is encouraged that the RobWork rw::common::ThreadPool concept is used.
The ThreadPool provides a flexible way for the user to specify the number of threads that should be used in multi-threaded applications.
By adding work to a ThreadPool instead of launching separate threads, the user has the ability to limit the resources used by the application.
At the same time the application is able to efficiently utilize the given resources.

The following example shows how a rw::common::ThreadPool can be created, and how work is added to the queue.
The work added to the pool is handled in the order it is added. In this example a list of arguments can be given from commandline.
The first argument is the number of threads to use (beside the main thread itself). Followed by that is an arbitrary number of image filenames.
The example will then try to load the image files in parallel according to the number of threads given by the user. Note that zero threads
is valid, as this would cause all work to be executed directly in the main thread. Work will in this case be executed directly in the addWork function.
To avoid that the program ends before work has finished, the waitForEmptyQueue() function will block until there is no more work available.

In the loadFile function the isStopping() method is checked. If the rw::common::ThreadPool is stopped before work has finished,
the work added to the pool should check the isStopping() function at regular intervals to allow the application to shutdown gracefully.
This is especially important in long running tasks.

.. literalinclude:: ../../../RobWork/example/snippets/ex-threadpool.cpp
   :language: c++
   :linenos:

When writing multi-threaded applications one often need to branch out into many parallel tasks, then wait for the tasks to finish and finally combine
the results. Then one might need to branch out yet again into new parallel tasks based on the result. Also each parallel task might be able to
parallize its work into even more subtasks. This can in principle happen in a hierachy where each subtask can have its own parallel subtasks and so forth.

To facilitate easy programming of such types of tasks, the rw::common::ThreadTask can be used. The following shows the same example as before but
instead implemented by the use of the rw::common::ThreadTask type.

Consider the LoadTask class that inherits from rw::common::ThreadTask. The LoadTask class is constructed with a filename that should be loaded.
In the run function the main work takes place - here the file is actually loaded, and the result can afterwards be retrieved with the getImage function.

The MainTask class also inherits from rw::common::ThreadTask. This task is responsible for launching a LoadTask for each file given as input to the
program. This class does not do much work in its run function. It only constructs all LoadTasks and add these as subtasks to the MainTask. The subtasks
starts running as soon as they are added, and when each subtask finishes, the subTaskDone function is called. Here the result of the subtask is simply
stored. If it was desired, new tasks could also be launched based on the achieved result. When there are no more subtasks running, the idle function is
called. The idle function is the last chance to add new subtasks, otherwise the task will end and the done function will be called. When done is called
the task is finished, and it is not possible to add more subtasks. If this task then has a parent task, the parent tasks subTaskDone will be executed.
It is possible to change the behaviour of a rw::common::ThreadTask such that it does not finish after the idle function is called. By enabling the keep-alive
option the task will stay in the idle state until the keep-alive is disabled or new subtasks are added. Be careful with this option. If the task does
not end, the parents will not end either.

The main function is almost as before. This time a MainTask is created that uses the ThreadPool to add its work to. When the execute function is called
the work is added to the pool and execution starts. By using the waitUntilDone function the main loop will not end before the MainTask has finished.

Note that multiple separate tasks and worker function can use the same pool at the same time.

As the ThreadPool is able to run with zero threads (executing directly in the main thread), so is the ThreadTask. If zero threads are used, the complete
task will be done at the time execute returns. It is however not recommended to use zero threads with the ThreadTask. Internally this will cause a
large recursion depth, and will inevitably cause problems with the stack size at some point. 

.. literalinclude:: ../../../RobWork/example/snippets/ex-threadtask.cpp
   :language: c++
   :linenos:
