%module sdurw_assembly

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rwlibs::task::Task;
%}

%include <std_string.i>
%include <std_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw.i>
%import <rwlibs/swig/sdurw_task.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_task.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_task.*;
%}

class AssemblyControlResponse
{
public:
	AssemblyControlResponse();
	virtual ~AssemblyControlResponse();
	
	/*
	typedef enum Type {
		POSITION,    //!< Position control
		VELOCITY,    //!< Velocity control
		HYBRID_FT_POS//!< Hybrid position and force/torque control
	} Type;

	Type type;
	*/
	
	rw::math::Transform3D<double>  femaleTmaleTarget;
	rw::core::Ptr<Trajectory<rw::math::Transform3D<double> > > worldTendTrajectory;
	rw::math::VelocityScrew6D<double>  femaleTmaleVelocityTarget;
	rw::math::Rotation3D<double>  offset;
	//VectorND<6,bool> selection;
	rw::math::Wrench6D<double> force_torque;
	bool done;
	bool success;
};

%template (AssemblyControlResponsePtr) rw::core::Ptr<AssemblyControlResponse>;
OWNEDPTR(AssemblyControlResponse);

/**
 * @brief Interface for assembly control strategies.
 */
class AssemblyControlStrategy
{
public:
	AssemblyControlStrategy();
	virtual ~AssemblyControlStrategy();
	
	/*
	class ControlState {
	public:
		//! @brief smart pointer type to this class
	    typedef rw::core::Ptr<ControlState> Ptr;

		//! @brief Constructor.
		ControlState() {};

		//! @brief Destructor.
		virtual ~ControlState() {};
	};
	virtual rw::core::Ptr<ControlState> createState() const;
	*/
	
	//virtual rw::core::Ptr<AssemblyControlResponse> update(rw::core::Ptr<AssemblyParameterization> parameters, rw::core::Ptr<AssemblyState> real, rw::core::Ptr<AssemblyState> assumed, rw::core::Ptr<ControlState> controlState, rw::kinematics::State &state, FTSensor* ftSensor, double time) const = 0;
	virtual rw::math::Transform3D<double>  getApproach(rw::core::Ptr<AssemblyParameterization> parameters) = 0;
	virtual std::string getID() = 0;
	virtual std::string getDescription() = 0;
	virtual rw::core::Ptr<AssemblyParameterization> createParameterization(const rw::core::Ptr<rw::core::PropertyMap> map) = 0;
};

%template (AssemblyControlStrategyPtr) rw::core::Ptr<AssemblyControlStrategy>;

class AssemblyParameterization
{
public:
	AssemblyParameterization();
	AssemblyParameterization(rw::core::Ptr<rw::core::PropertyMap> pmap);
	virtual ~AssemblyParameterization();
	virtual rw::core::Ptr<rw::core::PropertyMap> toPropertyMap() const;
	virtual rw::core::Ptr<AssemblyParameterization> clone() const;
};

%template (AssemblyParameterizationPtr) rw::core::Ptr<AssemblyParameterization>;
OWNEDPTR(AssemblyParameterization);

class AssemblyRegistry
{
public:
	AssemblyRegistry();
	virtual ~AssemblyRegistry();
	void addStrategy(const std::string id, rw::core::Ptr<AssemblyControlStrategy> strategy);
	std::vector<std::string> getStrategies() const;
	bool hasStrategy(const std::string& id) const;
	rw::core::Ptr<AssemblyControlStrategy> getStrategy(const std::string &id) const;
};

%template (AssemblyRegistryPtr) rw::core::Ptr<AssemblyRegistry>;
OWNEDPTR(AssemblyRegistry);

class AssemblyResult
{
public:
	AssemblyResult();
	AssemblyResult(rw::core::Ptr<Task<rw::math::Transform3D<double> > > task);
	virtual ~AssemblyResult();
	rw::core::Ptr<AssemblyResult> clone() const;
	rw::core::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
	static void saveRWResult(rw::core::Ptr<AssemblyResult> result, const std::string& name);
	static void saveRWResult(std::vector<rw::core::Ptr<AssemblyResult> > results, const std::string& name);
	static std::vector<rw::core::Ptr<AssemblyResult> > load(const std::string& name);
	static std::vector<rw::core::Ptr<AssemblyResult> > load(std::istringstream& inputStream);
	
	bool success;
	//Error error;
	rw::math::Transform3D<double>  femaleTmaleEnd;

	std::string taskID;
	std::string resultID;
    
    rw::trajectory::Path<Timed<AssemblyState> > realState;
	rw::trajectory::Path<Timed<AssemblyState> > assumedState;
	rw::math::Transform3D<double>  approach;
	std::string errorMessage;
};

%template (AssemblyResultPtr) rw::core::Ptr<AssemblyResult>;
%template (AssemblyResultPtrVector) std::vector<rw::core::Ptr<AssemblyResult> >;
OWNEDPTR(AssemblyResult);

class AssemblyState
{
public:
	AssemblyState();
	//AssemblyState(rw::core::Ptr<Target<rw::math::Transform3D<double> > > target);
	virtual ~AssemblyState();
	//static rw::core::Ptr<Target<rw::math::Transform3D<double> > > toCartesianTarget(const AssemblyState &state);

	std::string phase;
	rw::math::Transform3D<double>  femaleOffset;
	rw::math::Transform3D<double>  maleOffset;
	rw::math::Transform3D<double>  femaleTmale;
	rw::math::Wrench6D<double> ftSensorMale;
	rw::math::Wrench6D<double> ftSensorFemale;
	bool contact;
	rw::trajectory::Path<rw::math::Transform3D<double> > maleflexT;
	rw::trajectory::Path<rw::math::Transform3D<double> > femaleflexT;
	rw::trajectory::Path<rw::math::Transform3D<double> > contacts;
	rw::math::Vector3D<double> maxContactForce;
};

%template (AssemblyStatePtr) rw::core::Ptr<AssemblyState>;
%template (TimedAssemblyState) Timed<AssemblyState>;
%template (TimedAssemblyStateVector) std::vector<Timed<AssemblyState> >;
%template (PathTimedAssemblyState) rw::trajectory::Path<Timed<AssemblyState> >;
OWNEDPTR(AssemblyState);

class AssemblyTask
{
public:
	AssemblyTask();
	AssemblyTask(rw::core::Ptr<Task<rw::math::Transform3D<double> > > task, rw::core::Ptr<AssemblyRegistry> registry = NULL);
	virtual ~AssemblyTask();
	rw::core::Ptr<Task<rw::math::Transform3D<double> > > toCartesianTask();
	static void saveRWTask(rw::core::Ptr<AssemblyTask> task, const std::string& name);
	static void saveRWTask(std::vector<rw::core::Ptr<AssemblyTask> > tasks, const std::string& name);
	static std::vector<rw::core::Ptr<AssemblyTask> > load(const std::string& name, rw::core::Ptr<AssemblyRegistry> registry = NULL);
	static std::vector<rw::core::Ptr<AssemblyTask> > load(std::istringstream& inputStream, rw::core::Ptr<AssemblyRegistry> registry = NULL);
	rw::core::Ptr<AssemblyTask> clone() const;

	std::string maleID;
    std::string femaleID;
    rw::math::Transform3D<double>  femaleTmaleTarget;
    rw::core::Ptr<AssemblyControlStrategy> strategy;
    rw::core::Ptr<AssemblyParameterization> parameters;
    
    std::string maleTCP;
    std::string femaleTCP;
    
    std::string taskID;
    std::string workcellName;
    std::string generator;
    std::string date;
    std::string author;
    
    std::string malePoseController;
    std::string femalePoseController;
    std::string maleFTSensor;
    std::string femaleFTSensor;
    std::vector<std::string> maleFlexFrames;
    std::vector<std::string> femaleFlexFrames;
    std::vector<std::string> bodyContactSensors;
};

%template (AssemblyTaskPtr) rw::core::Ptr<AssemblyTask>;
%template (AssemblyTaskPtrVector) std::vector<rw::core::Ptr<AssemblyTask> >;
OWNEDPTR(AssemblyTask);