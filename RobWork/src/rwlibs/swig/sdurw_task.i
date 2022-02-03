%module sdurw_task

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
using rwlibs::task::Task;
%}

%include <exception.i>

%import <rwlibs/swig/sdurw.i>
%import <rwlibs/swig/sdurw_core.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
%}



template <class T>
class Task
{
};

%template (TaskSE3) Task<rw::math::Transform3D<double> >;
%template (TaskSE3Ptr) rw::core::Ptr<Task<rw::math::Transform3D<double> > >;
OWNEDPTR(Task<rw::math::Transform3D<double> > );

class GraspTask {
public:
    GraspTask():
    GraspTask(rw::core::Ptr<Task<rw::math::Transform3D<double> > > task);
    rw::core::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
    std::string getGripperID();
    std::string getTCPID();
    std::string getGraspControllerID();
    void setGripperID(const std::string& id);
    void setTCPID(const std::string& id);
    void setGraspControllerID(const std::string& id);
    static std::string toString(GraspResult::TestStatus status);
    static void saveUIBK(rw::core::Ptr<GraspTask> task, const std::string& name );
    static void saveRWTask(rw::core::Ptr<GraspTask> task, const std::string& name );
    static void saveText(rw::core::Ptr<GraspTask> task, const std::string& name );
    static rw::core::Ptr<GraspTask> load(const std::string& name);
    static rw::core::Ptr<GraspTask> load(std::istringstream& inputStream);
    rw::core::Ptr<GraspTask> clone();
};

%template (GraspTaskPtr) rw::core::Ptr<GraspTask>;
OWNEDPTR(GraspTask);

class GraspResult {
public:
	enum TestStatus {
        UnInitialized = 0,
        Success, // 1
        CollisionInitially, // 2
        ObjectMissed, // 3
        ObjectDropped, // 4
        ObjectSlipped, // 5
        TimeOut, // 6
        SimulationFailure, // 7
        InvKinFailure, // 8
        PoseEstimateFailure, // 9
        CollisionFiltered, // 10
        CollisionObjectInitially, // 11
        CollisionEnvironmentInitially, // 12
        CollisionDuringExecution, // 13
        Interference, // 14
        WrenchInsufficient, // 15
        SizeOfStatusArray
     };
};
%template (GraspResultPtr) rw::core::Ptr<GraspResult>;
OWNEDPTR(GraspResult);

/* @} */

//TODO add other grasptask related classes
