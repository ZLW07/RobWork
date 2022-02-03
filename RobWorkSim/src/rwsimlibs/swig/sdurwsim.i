%module sdurwsim

%{
#include <RobWorkSimConfig.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwsimlibs/swig/ScriptTypes.hpp>
#include <rw/models.hpp>

#if defined (SWIGLUA)
    #include <rwsimlibs/swig/lua/Lua.hpp>
#endif
#if defined (SWIGJAVA)
	#include <rwsimlibs/swig/java/ThreadSimulatorStepCallbackEnv.hpp>
    typedef rwsimlibs::swig::ThreadSimulatorStepCallbackEnv ThreadSimulatorStepCallbackEnv;
    typedef rwsimlibs::swig::ThreadSimulatorStepCallbackEnv::cThreadSimulatorStepCallback cThreadSimulatorStepCallback;
#endif

#include <rw/core/Ptr.hpp>
using rw::trajectory::Trajectory;
using namespace rwlibs::swig;
using namespace rwsim::swig;

#ifndef WIN32
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif
#if defined(SWIGJAVA)

#include <assert.h>
struct callback_data {
  JNIEnv *env;
  jobject obj;
};

void java_ThreadSimulatorStepCallback(ThreadSimulator* sim, rw::kinematics::State &state, void *ptr) {
  struct callback_data *data = (struct callback_data*) ptr;
  JNIEnv* env = data->env;
  bool fromThread = false;
  
  JavaVM* jvm;
  env->GetJavaVM(&jvm);
  JNIEnv* newEnv;
  int getEnvStat = jvm->GetEnv((void **)&newEnv, JNI_VERSION_1_6);
  if (getEnvStat == JNI_EDETACHED) {
    fromThread = true;
    if (jvm->AttachCurrentThread((void **) &newEnv, NULL) != 0) {
      std::cout << "Failed to attach" << std::endl;
      return;
    } else {
      env = newEnv;
    }
  //} else if (getEnvStat == JNI_OK) {
  } else if (getEnvStat == JNI_EVERSION) {
    std::cout << "GetEnv: version not supported" << std::endl;
    return;
  }
  
  const jclass callbackInterfaceClass = env->FindClass("dk/robwork/ThreadSimulatorStepCallbackHandler");
  assert(callbackInterfaceClass);
  const jmethodID meth = env->GetMethodID(callbackInterfaceClass, "callback", "(Ldk/robwork/ThreadSimulator;Ldk/robwork/State;)V");
  assert(meth);
  
  jclass threadSimClass = env->FindClass("dk/robwork/ThreadSimulator");
  jmethodID threadSimConstructor = env->GetMethodID(threadSimClass, "<init>", "(JZ)V");
  jobject jsim = env->NewObject(threadSimClass, threadSimConstructor, (jlong) sim, (jboolean)false);
  
  jclass stateClass = env->FindClass("dk/robwork/State");
  jmethodID stateConstructor = env->GetMethodID(stateClass, "<init>", "(JZ)V");
  jobject jstate = env->NewObject(stateClass, stateConstructor, (jlong) &state, (jboolean)false);
  
  env->CallVoidMethod(data->obj, meth, jsim, jstate);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
  }
  
  if (fromThread) {
    jvm->DetachCurrentThread();
  }
}
#endif
%}

#include <RobWorkSimConfig.hpp>

%include <std_string.i>
%include <std_vector.i>
%include <shared_ptr.i>
%include <exception.i>

#if defined(SWIGPYTHON) && RW_USE_NUMPY
%include <rwlibs/swig/ext_i/eigen.i>
#endif 

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw_sensor.i>
%import <rwlibs/swig/sdurw.i>
%import <rwlibs/swig/sdurw_assembly.i>
%import <rwlibs/swig/sdurw_control.i>
%import <rwlibs/swig/sdurw_simulation.i>
%import <rwlibs/swig/sdurw_proximity.i>



%pragma(java) jniclassclassmodifiers="class"

%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_assembly.*;
import org.robwork.sdurw_control.*;
import org.robwork.sdurw_simulation.*;
import org.robwork.sdurw_task.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_sensor.*;
%}
%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_assembly.*;
import org.robwork.sdurw_simulation.*;
import org.robwork.sdurw_task.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_models.*;
import org.robwork.sdurw_proximity.*;
import org.robwork.sdurw_sensor.*;
%}

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%include <stl.i>

/********************************************
 * Boost
 ********************************************/

#if defined(SWIGJAVA)
%typemap(jstype) cThreadSimulatorStepCallback fct "ThreadSimulatorStepCallbackHandler";
%typemap(jtype) cThreadSimulatorStepCallback fct "ThreadSimulatorStepCallbackHandler";
%typemap(jni) cThreadSimulatorStepCallback fct "jobject";
%typemap(javain) cThreadSimulatorStepCallback fct "$javainput";

%typemap(in,numinputs=1) (cThreadSimulatorStepCallback fct, void *userdata) {
  struct callback_data *data = (struct callback_data*) malloc(sizeof *data);
  data->env = jenv;
  data->obj = JCALL1(NewGlobalRef, jenv, $input);
  JCALL1(DeleteLocalRef, jenv, $input);
  $1 = java_ThreadSimulatorStepCallback;
  $2 = data;
}
#endif

%nodefaultctor ThreadSimulatorStepCallback;
class ThreadSimulatorStepCallback {
public:
	// Make pass by value possible in setStepCallback
	ThreadSimulatorStepCallback(const ThreadSimulatorStepCallback &cb);
};

#if defined(SWIGJAVA)
%nodefaultctor ThreadSimulatorStepCallbackEnv;
class ThreadSimulatorStepCallbackEnv/*: public ThreadSimulatorStepCallback*/ {
public:
	ThreadSimulatorStepCallbackEnv(const ThreadSimulatorStepCallbackEnv &cb);
	ThreadSimulatorStepCallbackEnv(cThreadSimulatorStepCallback fct, void *userdata);
	void set(cThreadSimulatorStepCallback fct, void *userdata);
	%rename(dispatch) operator();
	void operator()(ThreadSimulator* sim, rw::kinematics::State& state);
};
#endif

/********************************************
 * CONTACTS
 ********************************************/

%nodefaultctor ContactDetector;
class ContactDetector {
public:
	/*struct StrategyTableRow {
		std::size_t priority;
		ProximitySetup rules;
		ContactStrategy::Ptr strategy;
		//FrameMap<std::map<std::string, rw::core::Ptr<ContactModel> > > models;
	};*/
	//typedef std::list<StrategyTableRow> StrategyTable;

	ContactDetector(rw::core::Ptr<rw::models::WorkCell> workcell);
	//ContactDetector(rw::core::Ptr<rw::models::WorkCell> workcell, rw::core::Ptr<ProximityFilterStrategy> filter);
	virtual ~ContactDetector();
	//void setProximityFilterStrategy(rw::proximity::ProximityFilterStrategy::Ptr filter);
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state);
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state, ContactDetectorData &data);
	virtual std::vector<Contact> findContacts(const rw::kinematics::State& state, ContactDetectorData &data, ContactDetectorTracking& tracking);
	virtual std::vector<Contact> updateContacts(const rw::kinematics::State& state, ContactDetectorData &data, ContactDetectorTracking& tracking);
	//virtual rw::core::Ptr<ProximityFilterStrategy> getProximityFilterStrategy() const;
	virtual double getTimer() const;
	virtual void setTimer(double value = 0);
	//virtual StrategyTable getContactStategies() const;
	//virtual StrategyTable getContactStrategies(const std::string& frameA, rw::core::Ptr<const rw::geometry::Geometry> geometryA, const std::string& frameB, rw::core::Ptr<const rw::geometry::Geometry> geometryB) const;
	virtual void addContactStrategy(rw::core::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(ProximitySetupRule rule, rw::core::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(ProximitySetup rules, rw::core::Ptr<ContactStrategy> strategy, std::size_t priority = 0);
	//virtual void addContactStrategy(StrategyTableRow &strategy, std::size_t priority = 0);
	virtual void removeContactStrategy(std::size_t priority = 0);
	virtual void clearStrategies();
	//virtual void setContactStrategies(StrategyTable strategies);
	virtual void setDefaultStrategies();
	virtual void setDefaultStrategies(const rw::core::PropertyMap& map);
	virtual void printStrategyTable() const;
};

%template (ContactDetectorPtr) rw::core::Ptr<ContactDetector>;
OWNEDPTR(ContactDetector);

class ContactDetectorData {
public:
	ContactDetectorData();
	ContactDetectorData(const ContactDetectorData& data);
	virtual ~ContactDetectorData();
	//ContactDetectorData& operator=(const ContactDetectorData& data);
	void clear();
	//ContactStrategyData& getStrategyData(const ContactModel* modelA, const ContactModel* modelB);
};

%template (ContactDetectorDataPtr) rw::core::Ptr<ContactDetectorData>;
OWNEDPTR(ContactDetectorData);

class ContactDetectorTracking {
public:
	ContactDetectorTracking();
	ContactDetectorTracking(const ContactDetectorTracking& data);
	virtual ~ContactDetectorTracking();
	//ContactDetectorTracking& operator=(const ContactDetectorTracking& data);
	void clear();
	void remove(std::size_t index);

	std::size_t getSize() const;
};

%template (ContactDetectorTrackingPtr) rw::core::Ptr<ContactDetectorTracking>;
OWNEDPTR(ContactDetectorTracking);

class ContactStrategy //: public rw::proximity::ProximityStrategy
{
public:
	ContactStrategy();
	virtual ~ContactStrategy() {};
	virtual bool match(rw::core::Ptr<rw::geometry::GeometryData> geoA, rw::core::Ptr<rw::geometry::GeometryData> geoB) = 0;

	virtual std::string getName() = 0;
	
	virtual void clear() = 0;
	
	// ContactStrategy:
	virtual rw::core::PropertyMap& getPropertyMap();
#if !defined(SWIGJAVA)
	virtual const rw::core::PropertyMap& getPropertyMap() const;
#endif
	virtual void setPropertyMap(const rw::core::PropertyMap& map);
};

%template (ContactStrategyPtr) rw::core::Ptr<ContactStrategy>;
OWNEDPTR(ContactStrategy);

class Contact {
public:
	Contact();
	virtual ~Contact();

	const rw::kinematics::Frame* getFrameA() const;
	const rw::kinematics::Frame* getFrameB() const;
	rw::math::Transform3D<double> aTb() const;
	rw::math::Vector3D<double> getPointA() const;
	rw::math::Vector3D<double> getPointB() const;
	rw::math::Vector3D<double> getNormal() const;
	double getDepth() const;
	void setFrameA(const rw::kinematics::Frame* Aframe);
	void setFrameB(const rw::kinematics::Frame* Bframe);
	void setTransform(rw::math::Transform3D<double> aTb);
	void setPointA(rw::math::Vector3D<double> pointA);
	void setPointB(rw::math::Vector3D<double> pointB);
	void setPoints(rw::math::Vector3D<double> pointA, rw::math::Vector3D<double> pointB);
	void setNormal(rw::math::Vector3D<double> normal);
	void setDepth();
	void setDepth(double depth);
	//tostring
};

%template (ContactVector) std::vector<Contact>;


/********************************************
 * CONTROL
 ********************************************/
 
%nodefaultctor PoseController;
class PoseController //: public SimulatedController
{
public:

    double getSampleTime();

    void setSampleTime(double stime);

    //void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

    //void reset(const rw::kinematics::State& state);

    //Controller* getController(){ return this; };

    std::string getControllerName();

    //rw::core::Ptr<rw::models::Device> getControlledDevice(){ return _device; }

    void setEnabled(bool enabled);

    bool isEnabled();

    void setTarget(const rw::math::Transform3D<double>& target);

    void setTarget(const rw::math::Transform3D<double>& target, const rw::math::VelocityScrew6D<double>& vals);

};

struct PDParam {
	PDParam();
	PDParam(double p, double d);
	double P;
	double D;
};

%template (PDParamVector) std::vector<PDParam>;

%nodefaultctor PDController;
class PDController
{
public:
	PDController(
	        const std::string& name,
	        rw::core::Ptr<DynamicDevice> rdev,
			JointController::ControlMode cmode,
			const std::vector<PDParam>& pdparams,
			double dt
			);

	PDController(
	        const std::string& name,
	        rw::core::Ptr<DynamicDevice> rdev,
			JointController::ControlMode cmode,
			const PDParam& pdparam,
			double dt
			);

	virtual ~PDController();

	std::vector<PDParam> getParameters();
	void setParameters(const std::vector<PDParam>& params);
	double getSampleTime();
	void setSampleTime(double stime);

	// From SimulatedController
	void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);
	void reset(const rw::kinematics::State& state);
	Controller* getController();
	std::string getControllerName();
    void setEnabled(bool enabled);
    bool isEnabled();

	// From JointController
	unsigned int getControlModes();
	void setControlMode(JointController::ControlMode mode);
	void setTargetPos(const rw::math::Q& target);
	void setTargetVel(const rw::math::Q& vals);
	void setTargetAcc(const rw::math::Q& vals);
	rw::math::Q getQ();
	rw::math::Q getQd();
};

%template (PDControllerPtr) rw::core::Ptr<PDController>;
OWNEDPTR(PDController);

%nodefaultctor SerialDeviceController;
class SerialDeviceController //: public rwlibs::simulation::SimulatedController 
{
public:

	/**
	 *
	 * @param name [in] controller name
	 * @param ddev [in]
	 */
	// SerialDeviceController(const std::string& name, dynamics::DynamicDevice::Ptr ddev);

	//! destructor
	virtual ~SerialDeviceController();

	//! @brief move robot in a linear Cartesian path
	bool moveLin(const rw::math::Transform3D<double>& target, float speed=100, float blend=0);

	//! @brief move robot from point to point
	bool movePTP(const rw::math::Q& target, float speed=100, float blend=0);

	//! @brief move robot from point to point but using a pose as target (require invkin)
	virtual bool movePTP_T(const rw::math::Transform3D<double>& target, float speed=100, float blend=0);

	//! @brief move robot in a servoing fasion
	virtual bool moveVelQ(const rw::math::Q& target_joint_velocity);

	virtual bool moveVelT(const rw::math::VelocityScrew6D<double>& vel);

	//! move robot with a hybrid position/force control

    /*%extend {

    	bool moveLinFC(const rw::math::Transform3D<double>& target,
    							  const rw::math::Wrench6D<double>& wtarget,
    							  rw::math::Q selection,
    							  std::string refframe,
    							  rw::math::Rotation3D<double> offset,
    							  float speed=100,
    							  float blend=0)
    	{
    		float arr[6];
    		for(int i=0;i<6;i++)
    			arr[i] = (float)selection[i];
    		return $self->SerialDeviceController::moveLinFC(target,wtarget,arr,refframe,offset,speed,blend);
    	}

        
    }*/
	
	//! hard stop the robot,
	bool stop();

	//! pause the robot, should be able to continue trajectory
	bool pause();

	//! enable safe mode, so that robot stops when collisions are detected
	bool setSafeModeEnabled(bool enable);

	rw::math::Q getQ();
	rw::math::Q getQd();

	bool isMoving();

	// simulated controller stuff

	//void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);


    std::string getControllerName();

    void reset(const rw::kinematics::State& state);

    //rwlibs::control::Controller* getController();

    void setEnabled(bool enabled);

    bool isEnabled();

};

%template (SerialDeviceControllerPtr) rw::core::Ptr<SerialDeviceController>;
OWNEDPTR(SerialDeviceController);

%nodefaultctor BodyController;
class BodyController
{
};

%template (BodyControllerPtr) rw::core::Ptr<BodyController>;

/********************************************
 * DRAWABLE
 ********************************************/

/********************************************
 * DYNAMICS
 ********************************************/

struct BodyInfo {
public:
    BodyInfo();

    std::string material;
    std::string objectType;
    double mass;
    rw::math::Vector3D<double> masscenter;
    rw::math::InertiaMatrix<double> inertia;
    std::string integratorType;
    //std::vector<rw::kinematics::Frame*> frames;

#if !defined(SWIGPYTHON)
    void print() const;
    void print(std::ostream& ostr) const;
#endif
};

class Body
{
public:
    //typedef rw::core::Ptr<Body> Ptr;
    rw::kinematics::Frame* getBodyFrame() const;

    //const std::vector<rw::geometry::Geometry::Ptr>& getGeometry();
    const std::vector<rw::kinematics::Frame*>& getFrames();

#if !defined(SWIGJAVA)
    const BodyInfo& getInfo() const;
#endif
    BodyInfo& getInfo();

    const std::string& getName() const;
    const std::string& getMaterialID() const;
    const rw::math::InertiaMatrix<double>& getInertia() const;

    void setMass(double m);
    void setMass(double m, const rw::math::InertiaMatrix<double>& inertia);
    void setMass(double m, const rw::math::InertiaMatrix<double>& inertia, const rw::math::Vector3D<double>& com);

    //! interface functions
    virtual rw::math::Vector3D<double> getPointVelW(const rw::math::Vector3D<double>& p, const rw::kinematics::State& state) const = 0;
    virtual void reset(rw::kinematics::State &state) = 0;
    virtual double calcEnergy(const rw::kinematics::State& state) = 0;
    virtual void setForce(const rw::math::Vector3D<double>& f, rw::kinematics::State& state) = 0;
    virtual rw::math::Vector3D<double> getForce(const rw::kinematics::State& state) const = 0;
    virtual void addForce(const rw::math::Vector3D<double>& force, rw::kinematics::State& state) = 0;
    virtual void setTorque(const rw::math::Vector3D<double>& t, rw::kinematics::State& state) = 0;
    virtual void addTorque(const rw::math::Vector3D<double>& t, rw::kinematics::State& state) = 0;
    virtual rw::math::Vector3D<double> getTorque(const rw::kinematics::State& state) const = 0;

    virtual rw::kinematics::Frame* getParentFrame(const rw::kinematics::State& state) const;
    virtual void setForceW(const rw::math::Vector3D<double>& f, rw::kinematics::State& state);
    virtual rw::math::Vector3D<double> getForceW(const rw::kinematics::State& state) const;
    virtual void addForceW(const rw::math::Vector3D<double>& force, rw::kinematics::State& state);
    void addForceToPos(const rw::math::Vector3D<double>& force,
                               const rw::math::Vector3D<double>& pos,
                               rw::kinematics::State& state);
    virtual void addForceWToPosW(const rw::math::Vector3D<double>& force,
                                 const rw::math::Vector3D<double>& pos,
                                 rw::kinematics::State& state);
    virtual void setTorqueW(const rw::math::Vector3D<double>& t, rw::kinematics::State& state);
    virtual void addTorqueW(const rw::math::Vector3D<double>& t, rw::kinematics::State& state);
    virtual rw::math::Vector3D<double> getTorqueW(rw::kinematics::State& state);
    virtual rw::math::Transform3D<double> getTransformW(const rw::kinematics::State& state);

    rw::math::Transform3D<double> pTbf(const rw::kinematics::State& state);

    rw::math::Transform3D<double> pTcom(const rw::kinematics::State& state);

    rw::math::Transform3D<double> wTbf(const rw::kinematics::State& state);
    // world
    rw::math::Transform3D<double> wTcom(const rw::kinematics::State& state);

    %extend {

        rw::math::Transform3D<double> place(rw::core::Ptr<rw::proximity::CollisionDetector> coldect, const rw::kinematics::State& state, const rw::math::Vector3D<double>& dir){
            return rwsim::dynamics::BodyUtil::placeBody($self, coldect, state, dir);
        }

        rw::math::Transform3D<double> place(rw::core::Ptr<rw::proximity::CollisionDetector> coldect, const rw::kinematics::State& state){
            return rwsim::dynamics::BodyUtil::placeBody($self, coldect, state, -rw::math::Vector3D<double>::z());
        }
        
    };
};

%template (BodyPtr) rw::core::Ptr<Body>;
%template (BodyPtrVector) std::vector<rw::core::Ptr<Body> >;
OWNEDPTR(Body);


class FixedBody: public Body
{
};

%template (FixedBodyPtr) rw::core::Ptr<FixedBody>;
%template (FixedBodyPtrVector) std::vector<rw::core::Ptr<FixedBody> >;
OWNEDPTR(FixedBody);

class KinematicBody: public Body
{
};

%template (KinematicBodyPtr) rw::core::Ptr<KinematicBody>;
%template (KinematicBodyPtrVector) std::vector<rw::core::Ptr<KinematicBody> >;
OWNEDPTR(KinematicBody);

class RigidBody : public Body
{
public:
    RigidBody(
        const BodyInfo& info,
        rw::kinematics::MovableFrame* frame,
        rw::core::Ptr<rw::geometry::Geometry> geom
        );

    RigidBody(
        const BodyInfo& info,
        rw::kinematics::MovableFrame* frame,
        const std::vector<rw::core::Ptr<rw::geometry::Geometry> >& geoms
        );

    //rw::math::InertiaMatrix<double> getEffectiveMassW(const rw::math::Vector3D<double>& wPc);
    rw::kinematics::Frame* getParent(rw::kinematics::State& state) const;
    rw::math::Transform3D<double> getPTBody(const rw::kinematics::State& state) const;
    void setPTBody(const rw::math::Transform3D<double>& pTb, rw::kinematics::State& state);
    rw::math::Transform3D<double> getWTBody(const rw::kinematics::State& state) const;

    rw::math::Transform3D<double> getWTParent(const rw::kinematics::State& state) const;
    rw::math::Vector3D<double> getLinVel(const rw::kinematics::State& state) const;

    /**
     * @brief return the linear velocity described in world frame
     */
    rw::math::Vector3D<double> getLinVelW(const rw::kinematics::State& state) const;
    void setLinVel(const rw::math::Vector3D<double> &lvel, rw::kinematics::State& state);
    void setLinVelW(const rw::math::Vector3D<double> &lvel, rw::kinematics::State& state);
    rw::math::Vector3D<double> getAngVel(const rw::kinematics::State& state) const ;
    rw::math::Vector3D<double> getAngVelW(rw::kinematics::State& state);
    void setAngVel(const rw::math::Vector3D<double> &avel, rw::kinematics::State& state);
    void setAngVelW(const rw::math::Vector3D<double> &avel, rw::kinematics::State& state);
    rw::math::Vector3D<double> getPointVel(const rw::math::Vector3D<double>& p, const rw::kinematics::State& state);
    double getMass() const;
    const rw::math::InertiaMatrix<double>& getBodyInertia() const;
    const rw::math::InertiaMatrix<double>& getBodyInertiaInv() const;
    rw::math::InertiaMatrix<double> calcInertiaTensorInv(const rw::kinematics::State& state) const;
    //rw::math::InertiaMatrix<double> calcInertiaTensorInvW(const rw::kinematics::State& state) const;
    rw::math::InertiaMatrix<double> calcInertiaTensor(const rw::kinematics::State& state) const;
    rw::kinematics::MovableFrame* getMovableFrame();
    rw::math::InertiaMatrix<double> calcEffectiveMass(const rw::math::Vector3D<double>& wPc, const rw::kinematics::State& state) const;
    rw::math::InertiaMatrix<double> calcEffectiveMassW(const rw::math::Vector3D<double>& wPc, const rw::kinematics::State& state) const;
    //rw::math::InertiaMatrix<double> calcEffectiveInertia(const rw::kinematics::State& state) const;
    //rw::math::InertiaMatrix<double> calcEffectiveInertiaInv(const rw::kinematics::State& state) const;
};

%template (RigidBodyPtr) rw::core::Ptr<RigidBody>;
%template (RigidBodyPtrVector) std::vector<rw::core::Ptr<RigidBody> >;
OWNEDPTR(RigidBody);

%nodefaultctor Constraint;
class Constraint {
public:
	typedef enum {
		Fixed,
		Prismatic,
		Revolute,
		Universal,
		Spherical,
		Piston,
		PrismaticRotoid,
		PrismaticUniversal,
		Free
	} ConstraintType;

	struct SpringParams {
	public:
		SpringParams();
		bool enabled;
		Eigen::Matrix<double,-1,-1> compliance;
		Eigen::Matrix<double,-1,-1> damping;
	};

	Constraint(const std::string& name, const ConstraintType &type, Body* b1, Body* b2);
	virtual ~Constraint();
	ConstraintType getType() const;
	Body* getBody1() const;
	Body* getBody2() const;
	size_t getDOF() const;
	size_t getDOFLinear() const;
	size_t getDOFAngular() const;
	rw::math::Transform3D<double> getTransform() const;
	void setTransform(const rw::math::Transform3D<double> &parentTconstraint);
	SpringParams getSpringParams() const;
	void setSpringParams(const SpringParams &params);
	//static bool toConstraintType(const std::string &string, Constraint::ConstraintType &type);
};

%template (ConstraintPtr) rw::core::Ptr<Constraint>;
%template (ConstraintPtrVector) std::vector<rw::core::Ptr<Constraint> >;
OWNEDPTR(Constraint);
%nodefaultctor DynamicDevice;
class DynamicDevice {
public:
    virtual rw::math::Q getQ(const rw::kinematics::State& state);

    virtual void setQ(const rw::math::Q &q, rw::kinematics::State& state);

    rw::core::Ptr<rw::models::Device> getKinematicModel();
    rw::core::Ptr<Body> getBase();

    virtual rw::math::Q getJointVelocities(const rw::kinematics::State& state);
    virtual void setJointVelocities(const rw::math::Q &vel, rw::kinematics::State& state);

    //deprecated
    virtual rw::math::Q getVelocity(const rw::kinematics::State& state);
    virtual void setVelocity(const rw::math::Q& vel, rw::kinematics::State& state);

    virtual std::vector<rw::core::Ptr<Body> > getLinks();

};

%template (DynamicDevicePtr) rw::core::Ptr<DynamicDevice>;
%template (DynamicDevicePtrVector) std::vector<rw::core::Ptr<DynamicDevice> >;
OWNEDPTR(DynamicDevice);

%nodefaultctor RigidDevice;
class RigidDevice : public DynamicDevice {
    public:
        void setMotorForceLimits(const rw::math::Q& force);

        rw::math::Q getMotorForceLimits();

        rw::math::Q getJointVelocities(const rw::kinematics::State& state);
        double getJointVelocity(int i, const rw::kinematics::State& state);

        void setJointVelocities(const rw::math::Q& q, rw::kinematics::State& state);
        void setJointVelocity(double vel, int i, rw::kinematics::State& state);

        typedef enum{Force, Velocity} MotorControlMode;

        //std::vector<MotorControlMode> getMotorModes(const rw::kinematics::State& state);
        MotorControlMode getMotorMode(int i, const rw::kinematics::State& state);

        rw::math::Q getMotorTargets(const rw::kinematics::State& state);
        double getMotorTarget(int i, const rw::kinematics::State& state);

        void setMotorTargets(const rw::math::Q& q, rw::kinematics::State& state);
        void setMotorForceTargets(const rw::math::Q& force, rw::kinematics::State& state);
        void setMotorVelocityTargets(const rw::math::Q& vel, rw::kinematics::State& state);

        void setMotorTarget(double q, int i, rw::kinematics::State& state);
        void setMotorForceTarget(double force, int i, rw::kinematics::State& state);
        void setMotorVelocityTarget(double vel, int i, rw::kinematics::State& state);

        rw::core::Ptr<rw::models::JointDevice> getJointDevice();
        std::vector<rw::core::Ptr<Body> > getLinks();

        //virtual void registerStateData(rw::kinematics::StateStructure::Ptr statestructure);

    public: ///// DEPRECATED FUNCTIONS
        //rw::math::Q getForceLimit() { return getMotorForceLimits(); }
        // void setVelocity(rw::math::Q& vel, rw::kinematics::State& state){ setJointVelocities(vel, state);}
    };

%template (RigidDevicePtr) rw::core::Ptr<RigidDevice>;
%template (RigidDevicePtrVector) std::vector<rw::core::Ptr<RigidDevice> >;
OWNEDPTR(RigidDevice);

%nodefaultctor SuctionCup;
class SuctionCup : public DynamicDevice {
public:

    rw::core::Ptr<Body> getBaseBody();

    rw::core::Ptr<Body> getEndBody();

    //void addToWorkCell(rw::core::Ptr<DynamicWorkCell> dwc);

    double getRadius();

    double getHeight();

    rw::math::Q getSpringParamsOpen();

    rw::math::Q getSpringParamsClosed();

    rw::math::Q getJointVelocities(const rw::kinematics::State& state);

    void setJointVelocities(const rw::math::Q &vel, rw::kinematics::State& state);

    void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state);

    rw::math::Transform3D<double> getOffset();

    std::vector<rw::core::Ptr<Body> > getLinks();

    bool isClosed(const rw::kinematics::State& state);
    void setClosed(bool closed, rw::kinematics::State& state);

    rw::core::Ptr<Body> getContactBody(const rw::kinematics::State& state);
    void setContactBody(rw::core::Ptr<Body> b, rw::kinematics::State& state);

    double getPressure(const rw::kinematics::State& state);
    void setPressure(double pressure, rw::kinematics::State& state);

};

%template (SuctionCupPtr) rw::core::Ptr<SuctionCup>;
OWNEDPTR(SuctionCup);

class DynamicWorkCell
{
public:
    //const BodyList& getBodies();

    /**
     * @brief Constructor
     */
    
    DynamicWorkCell(rw::core::Ptr<rw::models::WorkCell> workcell,
                    const std::vector<rw::core::Ptr<Body> >& bodies,
                    const std::vector<rw::core::Ptr<Body> >& allbodies,
                    const std::vector<rw::core::Ptr<Constraint> >& constraints,
                    const std::vector<rw::core::Ptr<DynamicDevice> >& devices,
                    const std::vector<rw::core::Ptr<SimulatedController> >& controllers);
	
    rw::core::Ptr<Body> findBody(const std::string& name) const;

    //template<class T> T* findBody(const std::string& name) const;
    //template<class T> std::vector<T*> findBodies() const;

    //const DeviceList& getDynamicDevices(){ return _devices; };
    //DynamicDevice* findDevice(const std::string& name);

    //const ControllerList& getControllers();
    //const SensorList& getSensors();
    //void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);
    //const std::vector<Constraint>& getConstraints();
    void addController(rw::core::Ptr<SimulatedController> manipulator);
    rw::core::Ptr<SimulatedController> findController(const std::string& name);
    rw::core::Ptr<DynamicDevice> findDevice(const std::string& name) const;
    rw::core::Ptr<SimulatedSensor> findSensor(const std::string& name);

    //ContactDataMap& getContactData();
    //MaterialDataMap& getMaterialData();

    const std::vector<rw::core::Ptr<Body> >& getBodies();
    void addBody(rw::core::Ptr<Body> body);
    rw::core::Ptr<Body> getBody(rw::kinematics::Frame *f);
    
    void addConstraint(rw::core::Ptr<Constraint> constraint);
    const std::vector<rw::core::Ptr<Constraint> >& getConstraints() const;
	rw::core::Ptr<Constraint> findConstraint(const std::string& name) const;

    rw::core::Ptr<rw::models::WorkCell> getWorkcell();

    double getCollisionMargin();
    void setCollisionMargin(double margin);

    //WorkCellDimension getWorldDimension();

    bool inDevice(rw::core::Ptr<Body> body);
    void setGravity(const rw::math::Vector3D<double>& grav);
    const rw::math::Vector3D<double>& getGravity();
    rw::core::PropertyMap& getEngineSettings();

    %extend {
        rw::core::Ptr<RigidBody> findRigidBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<RigidBody>(name); }
        rw::core::Ptr<KinematicBody> findKinematicBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<KinematicBody>(name); }
        rw::core::Ptr<FixedBody> findFixedBody(const std::string& name)
        { return $self->DynamicWorkCell::findBody<FixedBody>(name); }

        rw::core::Ptr<RigidDevice> findRigidDevice(const std::string& name)
        { return $self->DynamicWorkCell::findDevice<RigidDevice>(name); }
        rw::core::Ptr<SuctionCup> findSuctionCup(const std::string& name)
        { return $self->DynamicWorkCell::findDevice<SuctionCup>(name); }

        rw::core::Ptr<SimulatedFTSensor> findFTSensor(const std::string& name)
        { 
        	rw::core::Ptr<SimulatedFTSensor> sensor = $self->DynamicWorkCell::findSensor<SimulatedFTSensor>(name);
        	if(sensor==NULL)
        		RW_THROW("No such sensor!");
        	return sensor; 
        
        }

        rw::core::Ptr<SerialDeviceController> findSerialDeviceController(const std::string& name)
        { return $self->DynamicWorkCell::findController<SerialDeviceController>(name); }
        
        rw::core::Ptr<PDController> findPDController(const std::string& name)
        { return $self->DynamicWorkCell::findController<PDController>(name); }
        
        void setGravity(double x, double y, double z){
            $self->DynamicWorkCell::setGravity( rw::math::Vector3D<double>(x,y,z) );
        }
        
		rw::core::Ptr<Body> getBody(const std::string& name) const{
			rw::core::Ptr<Body> body = $self->findBody(name);
			if(body==NULL)
				RW_THROW("Could not find body: \"" << name << "\"" );
			return body;
		}


    };

};
%template (DynamicWorkCellPtr) rw::core::Ptr<DynamicWorkCell>;
OWNEDPTR(DynamicWorkCell);


/********************************************
 * LOADERS
 ********************************************/

class DynamicWorkCellLoader
{
public:
    static rw::core::Ptr<DynamicWorkCell> load(const std::string& filename);
};

/********************************************
 * RWPHYSICS
 ********************************************/

/********************************************
 * SENSOR
 ********************************************/

class SimulatedFTSensor //: public SimulatedTactileSensor 
{
public:
    SimulatedFTSensor(const std::string& name,
                      rw::core::Ptr<Body> body,
                      rw::core::Ptr<Body> body1,
                      rw::kinematics::Frame* frame=NULL);

	virtual ~SimulatedFTSensor();

	void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);
	void reset(const rw::kinematics::State& state);

	void addForceW(const rw::math::Vector3D<double>& point,
				   const rw::math::Vector3D<double>& force,
				   const rw::math::Vector3D<double>& cnormal,
				   rw::kinematics::State& state,
				   rw::core::Ptr<Body> body = NULL);

	void addForce(const rw::math::Vector3D<double>& point,
				  const rw::math::Vector3D<double>& force,
				  const rw::math::Vector3D<double>& cnormal,
				  rw::kinematics::State& state,
				  rw::core::Ptr<Body> body=NULL);

    void addWrenchToCOM(
                  const rw::math::Vector3D<double>& force,
                  const rw::math::Vector3D<double>& torque,
                  rw::kinematics::State& state,
                  rw::core::Ptr<Body> body=NULL);

    void addWrenchWToCOM(
                  const rw::math::Vector3D<double>& force,
                  const rw::math::Vector3D<double>& torque,
                  rw::kinematics::State& state,
                  rw::core::Ptr<Body> body=NULL);

    rw::math::Transform3D<double> getTransform();

    rw::math::Vector3D<double> getForce( rw::kinematics::State& state );
    
	rw::math::Vector3D<double> getTorque( rw::kinematics::State& state );

	//double getMaxTorque();

	//double getMaxForce();

	rw::kinematics::Frame* getSensorFrame();

	void acquire();

	//rw::core::Ptr<FTSensor> getSensor();

	rw::core::Ptr<Body> getBody1();
	rw::core::Ptr<Body> getBody2();
};

%template (SimulatedFTSensorPtr) rw::core::Ptr<SimulatedFTSensor>;
OWNEDPTR(SimulatedFTSensor);

/********************************************
 * SIMULATOR
 ********************************************/

%nodefaultctor PhysicsEngine;
class PhysicsEngine
{
public:
	virtual ~PhysicsEngine(){};
	virtual void load(rw::core::Ptr<DynamicWorkCell> dwc) = 0;
	virtual bool setContactDetector(rw::core::Ptr<ContactDetector> detector) = 0;
	virtual void step(double dt, rw::kinematics::State &state) = 0;
	virtual void resetScene(rw::kinematics::State& state) = 0;
	virtual void initPhysics(rw::kinematics::State& state) = 0;
	virtual void exitPhysics() = 0;
	virtual double getTime() = 0;
	virtual void setEnabled(rw::core::Ptr<Body> body, bool enabled) = 0;
	virtual void setDynamicsEnabled(rw::core::Ptr<Body> body, bool enabled) = 0;
	//virtual drawable::SimulatorDebugRender::Ptr createDebugRender() = 0;
	virtual rw::core::PropertyMap& getPropertyMap() = 0;
	virtual void emitPropertyChanged() = 0;
	virtual void addController(rw::core::Ptr<SimulatedController> controller) = 0;
	virtual void removeController(rw::core::Ptr<SimulatedController> controller) = 0;
	virtual void addBody(rw::core::Ptr<Body> body, rw::kinematics::State &state) = 0;
	virtual void addDevice(rw::core::Ptr<DynamicDevice> dev, rw::kinematics::State &state) = 0;
	virtual void addSensor(rw::core::Ptr<SimulatedSensor> sensor, rw::kinematics::State &state) = 0;
	virtual void removeSensor(rw::core::Ptr<SimulatedSensor> sensor) = 0;
	virtual void attach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2) = 0;
	virtual void detach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2) = 0;
	virtual std::vector<rw::core::Ptr<SimulatedSensor> > getSensors() = 0;
};

%template (PhysicsEnginePtr) rw::core::Ptr<PhysicsEngine>;
OWNEDPTR(PhysicsEngine);

%nodefaultctor PhysicsEngineFactory;
class PhysicsEngineFactory
{
public:
	static std::vector<std::string> getEngineIDs();
	static bool hasEngineID(const std::string& engineID);
	static rw::core::Ptr<PhysicsEngine> makePhysicsEngine(const std::string& engineID, rw::core::Ptr<DynamicWorkCell> dwc);
    static rw::core::Ptr<PhysicsEngine> makePhysicsEngine(rw::core::Ptr<DynamicWorkCell> dwc);
};

class DynamicSimulator: public Simulator
{
public:
    DynamicSimulator(rw::core::Ptr<DynamicWorkCell> dworkcell, rw::core::Ptr<PhysicsEngine> pengine);
    DynamicSimulator(rw::core::Ptr<DynamicWorkCell> dworkcell);
    virtual ~DynamicSimulator();

	%extend {
		static rw::core::Ptr<DynamicSimulator> make(rw::core::Ptr<DynamicWorkCell> dworkcell, rw::core::Ptr<PhysicsEngine> pengine){
			return rw::core::ownedPtr( new rwsim::simulator::DynamicSimulator(dworkcell,pengine) );
		 }
    }
    void exitPhysics();
	double getTime();
	rw::kinematics::State& getState();
	void setEnabled(rw::core::Ptr<Body> body, bool enabled);

	//drawable::SimulatorDebugRender::Ptr createDebugRender();
	rw::core::PropertyMap& getPropertyMap();
	
	void addController(rw::core::Ptr<SimulatedController> controller);
	void removeController(rw::core::Ptr<SimulatedController> controller);

	void addBody(rw::core::Ptr<Body> body, rw::kinematics::State &state);
	void addDevice(rw::core::Ptr<DynamicDevice> dev, rw::kinematics::State &state);
	void addSensor(rw::core::Ptr<SimulatedSensor> sensor, rw::kinematics::State &state);
	void removeSensor(rw::core::Ptr<SimulatedSensor> sensor);
	std::vector<rw::core::Ptr<SimulatedSensor> > getSensors();

	 // Simulator interface
     void step(double dt);
     void reset(rw::kinematics::State& state);
	 void init(rw::kinematics::State& state);
	 void setEnabled(rw::kinematics::Frame* f, bool enabled);
	 void setDynamicsEnabled(rw::core::Ptr<Body> body, bool enabled);
	 // interfaces for manipulating/controlling bodies
	 void setTarget(rw::core::Ptr<Body> body, const rw::math::Transform3D<double>& t3d, rw::kinematics::State& state); 
	 void setTarget(rw::core::Ptr<Body> body, rw::core::Ptr<Trajectory<rw::math::Transform3D<double> > > traj);

	 void disableBodyControl( rw::core::Ptr<Body> body );
	 void disableBodyControl( );

	 rw::core::Ptr<BodyController> getBodyController();

	 void attach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);

	 void detach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);
};

%template (DynamicSimulatorPtr) rw::core::Ptr<DynamicSimulator>;
%extend rw::core::Ptr<DynamicSimulator> { 
		rw::core::Ptr<Simulator> toSimulator(){
			std::cout << "Casting" << std::endl;
		 	rw::core::Ptr<Simulator> ssim = *$self;
		 	std::cout << "Cast result: " << ssim.isNull() << std::endl;
		 	return ssim;
		 }
		 
		 rw::core::Ptr<Simulator> toSimulator1(){
			std::cout << "Casting" << std::endl;
		 	rw::core::Ptr<rwlibs::simulation::Simulator> ssim = *$self;
		 	std::cout << "Cast result: " << ssim.isNull() << std::endl;
		 	return ssim;
		 }
		 rw::core::Ptr<Simulator> toSimulator2(){
			std::cout << "Casting" << std::endl;
		 	rw::core::Ptr<Simulator> ssim = *$self;
		 	std::cout << "Cast result: " << ssim.isNull() << std::endl;
		 	return ssim;
		 }
		 
}

class ThreadSimulator {
	public:
		ThreadSimulator(rw::core::Ptr<DynamicSimulator> simulator, const rw::kinematics::State &state);
		virtual ~ThreadSimulator();
		//void setPeriodMs(long period);
		void setRealTimeScale(double scale);
		void setTimeStep(double dt);
		void start();
		void stop();
		void postStop();
		void step();
		rw::kinematics::State getState();
		void setState(const rw::kinematics::State& state);
		void reset(const rw::kinematics::State& state);
		bool isRunning();
		double getTime();
		rw::core::Ptr<DynamicSimulator> getSimulator();

#if defined(SWIGJAVA)
%extend {
		void setStepCallBack(ThreadSimulatorStepCallbackEnv cb) {
			(*$self).setStepCallBack((ThreadSimulatorStepCallback)cb);
		}
}
#endif
		void setStepCallBack(ThreadSimulatorStepCallback cb);
		
		bool isInError();
		void setInError(bool inError);
	};

%template (ThreadSimulatorPtr) rw::core::Ptr<ThreadSimulator>;
%template (ThreadSimulatorPtrVector) std::vector<rw::core::Ptr<ThreadSimulator> >;
OWNEDPTR(ThreadSimulator);

%nodefaultctor GraspTaskSimulator;
class GraspTaskSimulator
{
public:
	GraspTaskSimulator(rw::core::Ptr<DynamicWorkCell> dwc, int nrThreads=1);
	virtual ~GraspTaskSimulator();
	void load(const std::string& filename);
	void load(rw::core::Ptr<GraspTask> graspTasks);
	rw::core::Ptr<GraspTask> getTasks();
	rw::core::Ptr<GraspTask> getResult();
	size_t getNrTargets();
	rw::core::Ptr<ThreadSimulator> getSimulator();
	std::vector<rw::core::Ptr<ThreadSimulator> > getSimulators();
	void init(rw::core::Ptr<DynamicWorkCell> dwc, const rw::kinematics::State& initState);
	void startSimulation(const rw::kinematics::State& initState);
	void pauseSimulation();
	void resumeSimulation();
	bool isRunning();
	bool isFinished();
	int getStat(GraspResult::TestStatus status);
	std::vector<int> getStat();
	std::string getStatDescription();
	int getNrTargetsDone();
	void setAlwaysResting(bool alwaysResting);
	void setStepDelay(int delay);
	void setWallTimeLimit(double limit);
	void setSimTimeLimit(double limit);
};

%template (GraspTaskSimulatorPtr) rw::core::Ptr<GraspTaskSimulator>;
OWNEDPTR(GraspTaskSimulator);
%nodefaultctor AssemblySimulator;
class AssemblySimulator
{
public:
	AssemblySimulator(rw::core::Ptr<DynamicWorkCell> dwc, const std::string &engineID, rw::core::Ptr<ContactDetector> contactDetector = NULL);
	virtual ~AssemblySimulator();
	void start(rw::core::Ptr<rw::common::ThreadTask> task = NULL);
	void stopFinishCurrent();
	void stopCancelCurrent();
	bool isRunning();
	void setTasks(std::vector<rw::core::Ptr<AssemblyTask> > tasks);
	std::vector<rw::core::Ptr<AssemblyResult> > getResults();
	void setStoreExecutionData(bool enable);
	bool storeExecutionData();
	double getMaxSimTime() const;
	void setMaxSimTime(double maxTime);
};

%template (AssemblySimulatorPtr) rw::core::Ptr<AssemblySimulator>;
OWNEDPTR(AssemblySimulator);

/********************************************
 * UTIL
 ********************************************/

/********************************************
 * RWSIMLIBS BULLET
 ********************************************/

/********************************************
 * RWSIMLIBS GUI
 ********************************************/

/********************************************
 * RWSIMLIBS ODE
 ********************************************/

#if defined(RWSIM_HAVE_ODE)
%nodefaultctor ODESimulator;
class ODESimulator: public PhysicsEngine
{
public:
		typedef enum{WorldStep, WorldQuickStep, WorldFast1} StepMethod;
		//typedef enum{Simple, HashTable, QuadTree} SpaceType;
		
		ODESimulator(rw::core::Ptr<DynamicWorkCell> dwc, rw::core::Ptr<ContactDetector> detector = rw::core::Ptr<ContactDetector>());

		virtual ~ODESimulator();
		
		// PhysicsEngine interface
		void load(rw::core::Ptr<DynamicWorkCell> dwc);
		bool setContactDetector(rw::core::Ptr<ContactDetector> detector);
		void step(double dt, rw::kinematics::State& state);
		void resetScene(rw::kinematics::State& state);
		void initPhysics(rw::kinematics::State& state);
		void exitPhysics();
		double getTime();
		void setEnabled(rw::core::Ptr<Body> body, bool enabled);
		void setDynamicsEnabled(rw::core::Ptr<Body> body, bool enabled);
		//drawable::SimulatorDebugRender::Ptr createDebugRender();
		virtual rw::core::PropertyMap& getPropertyMap();
		void emitPropertyChanged();
		void addController(rw::core::Ptr<SimulatedController> controller);
		void removeController(rw::core::Ptr<SimulatedController> controller);
		void addBody(rw::core::Ptr<Body> body, rw::kinematics::State &state);
		void addDevice(rw::core::Ptr<DynamicDevice> dev, rw::kinematics::State &state);
		void addSensor(rw::core::Ptr<SimulatedSensor> sensor, rw::kinematics::State &state);
		void removeSensor(rw::core::Ptr<SimulatedSensor> sensor);
		void attach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);
		void detach(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);
		std::vector<rw::core::Ptr<SimulatedSensor> > getSensors();
		
		// ODESimulator specific
		void setStepMethod(StepMethod method);
		//void DWCChangedListener(dynamics::DynamicWorkCell::DWCEventType type, boost::any data);
		bool isInitialized();
		//const rw::kinematics::FramePairMap<std::vector<dynamics::ContactManifold> >&getContactManifoldMap();
		//std::vector<ODEBody*>& getODEBodies(){ return _odeBodies;}
		rw::core::Ptr<DynamicWorkCell> getDynamicWorkCell();
		void disableCollision(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);
		void enableCollision(rw::core::Ptr<Body> b1, rw::core::Ptr<Body> b2);
		rw::math::Vector3D<double> getGravity();
		//dWorldID getODEWorldId();
		//void addODEJoint(ODEJoint* odejoint);
		//ODEJoint* getODEJoint(rw::models::Joint* joint);
        //void addODEBody(ODEBody* odebody);
        //void addODEBody(dBodyID body);
        //void addODEJoint(dJointID joint);
		//ODEBody* getODEBody(rw::kinematics::Frame* frame);
		//dBodyID getODEBodyId(rw::kinematics::Frame* frame);
		//dBodyID getODEBodyId(rwsim::dynamics::Body* body);
        //std::vector<ODEDevice*> getODEDevices() { return _odeDevices;}
        void addEmulatedContact(const rw::math::Vector3D<double>& pos, const rw::math::Vector3D<double>& force, const rw::math::Vector3D<double>& normal, Body* b);
        void setContactLoggingEnabled(bool enable);
        //std::map<std::pair<std::string,std::string>,std::vector<dynamics::ContactPoint> > getContactingBodies(){ return _contactingBodiesTmp; }
        //std::map<std::pair<std::string,std::string>,std::vector<dynamics::ContactPoint> > _contactingBodies, _contactingBodiesTmp;
		//void handleCollisionBetween(dGeomID o0, dGeomID o1);
		//const std::vector<ODEUtil::TriGeomData*>& getTriMeshs();
		//std::vector<dynamics::ContactPoint> getContacts();
		int getContactCnt();
		/*struct ODEStateStuff{
			ODEStateStuff():body(NULL){}
			dBodyID body;
			dReal pos[4];
			dReal rot[4];
			dReal lvel[4];
			dReal avel[4];
			dReal force[4];
			dReal torque[4];

			ODEJoint *joint;
			dReal desvel; //desired vel
			dReal fmax;
		};*/

        double getMaxSeperatingDistance();
        //dSpaceID getODESpace(){ return _spaceId; };
        //void addContacts(std::vector<dContact>& contacts, size_t nr_con, ODEBody* dataB1, ODEBody* dataB2);
        //std::vector<ODETactileSensor*> getODESensors(dBodyID odebody){ return _odeBodyToSensor[odebody]; }
        //dynamics::MaterialDataMap& getMaterialMap(){ return _materialMap; }
        //dynamics::ContactDataMap& getContactMap(){ return _contactMap; }
};

%template (ODESimulatorPtr) rw::core::Ptr<ODESimulator>;
OWNEDPTR(ODESimulator);
#endif

/********************************************
 * RWSIMLIBS PLUGINS
 ********************************************/

/********************************************
 * RWSIMLIBS SWIG
 ********************************************/

/********************************************
 * RWSIMLIBS TOOLS
 ********************************************/
 
 
 
/********************************************
 * General utility functions
 ********************************************/

rw::core::Ptr<DynamicWorkCell> getDynamicWorkCell();
void setDynamicWorkCell(rw::core::Ptr<DynamicWorkCell> dwc);

rw::core::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
void addSimulatorInstance(rw::core::Ptr<ThreadSimulator> sim, const std::string& id);
rw::core::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);
rw::core::Ptr<ThreadSimulator> getSimulatorInstance();
void removeSimulatorInstance(const std::string& id);
std::vector<std::string> getSimulatorInstances();

#ifndef WIN32
	#pragma GCC diagnostic pop
#endif
