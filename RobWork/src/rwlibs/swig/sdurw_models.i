%module sdurw_models

%include <stl.i>
%include <std_vector.i>
%include <rwlibs/swig/swig_macros.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>

%import <rwlibs/swig/ext_i/std.i>

%{
	#include <rw/kinematics/FixedFrame.hpp>
	#include <rw/sensor/CameraModel.hpp>
	#include <rw/sensor/TactileArrayModel.hpp>
	#include <rw/sensor/FTSensorModel.hpp>
	#include <rw/sensor/Scanner25DModel.hpp>
	#include <rw/sensor/Scanner2DModel.hpp>
	#include <rw/sensor/RGBDCameraModel.hpp>
	#include <rw/sensor/StereoCameraModel.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
%}

%{
	#include <rw/proximity/CollisionSetup.hpp>
	#include <rw/proximity/CollisionSetup.cpp>
	#if SWIG_VERSION < 0x040000
		#include <rw/geometry/IndexedTriangle.hpp>
		template<class T >
		using IndexedTriangle = rw::geometry::IndexedTriangle<T>;
	#endif 
%}

#if defined(SWIGJAVA)
	%ignore rw::models::Object::getBase() const;
#endif 
%{
	#include<rw/models/Object.hpp>
%}
%include <rw/models/Object.hpp>
NAMED_OWNEDPTR(Object,rw::models::Object);
%template (ObjectPtrVector) std::vector< rw::core::Ptr< rw::models::Object > >;

#if defined(SWIGJAVA)
	%ignore rw::models::Device::getBase() const;
	%ignore rw::models::Device::getEnd() const;
	%ignore rw::models::Device::getPropertyMap() const;
#endif 
%{
	#include<rw/models/Device.hpp>
%}
%include <rw/models/Device.hpp>
NAMED_OWNEDPTR(Device,rw::models::Device);
%template (DevicePtrVector) std::vector<rw::core::Ptr<rw::models::Device> >;

%{
	#include<rw/models/Joint.hpp>
%}
%include <rw/models/Joint.hpp>
NAMED_OWNEDPTR(Joint,rw::models::Joint);
%template(VectorJoint_p) std::vector<rw::models::Joint*>;

%{
	#include<rw/models/JointDevice.hpp>
%}
%include <rw/models/JointDevice.hpp>
NAMED_OWNEDPTR(JointDevice,rw::models::JointDevice);
%template (JointDevicePtrVector) std::vector< rw::core::Ptr< rw::models::JointDevice>>;


%nodefaultctor JacobianCalculator;
%{
	#include<rw/models/JacobianCalculator.hpp>
%}
%include <rw/models/JacobianCalculator.hpp>
NAMED_OWNEDPTR(JacobianCalculator,rw::models::JacobianCalculator);

%extend rw::models::JacobianCalculator{
	/**
	 * @brief Returns the Jacobian associated to \b state
	 *
	 * @param state [in] State for which to calculate the Jacobian
	 * @return Jacobian for \b state
	 */
	virtual rw::math::Jacobian getJacobian(const rw::kinematics::State& state) const {
		return $self->get(state);
	}
};

%{
	#include<rw/models/CompositeDevice.hpp>
%}
%include <rw/models/CompositeDevice.hpp>
NAMED_OWNEDPTR(CompositeDevice,rw::models::CompositeDevice);
%extend rw::core::Ptr<rw::models::CompositeDevice>{
	rw::core::Ptr<rw::models::Device> asDevicePtr() { return *$self; }
	rw::core::Ptr<rw::models::Device const> asDeviceCPtr() { return *$self; }
	rw::core::Ptr<rw::models::JointDevice> asJointDevicePtr() { return *$self; }
	rw::core::Ptr<rw::models::JointDevice const> asJointDeviceCPtr() { return *$self; }
}
#if defined(SWIGPYTHON)
#ifndef SWIG_POINTER_NO_NULL
#define SWIG_POINTER_NO_NULL 0 
#endif
%typecheck(SWIG_TYPECHECK_SWIGOBJECT) rw::core::Ptr<rw::models::Device const>{
	int res = SWIG_ConvertPtr($input, 0, $descriptor(rw::core::Ptr<rw::models::Device const> *),SWIG_POINTER_NO_NULL | 0);
	if (SWIG_IsOK(res)) {
		$1 = 1;
	}
	else {
		int res = SWIG_ConvertPtr($input, 0, $descriptor(rw::core::Ptr<rw::models::CompositeDevice const> *),SWIG_POINTER_NO_NULL | 0);
		if (SWIG_IsOK(res)) {
			$1 = 1;
		}
		else {
			$1 = 0;
		}
	}
}
%typemap(in) rw::core::Ptr<rw::models::Device const> (void * argp){
	int res = SWIG_ConvertPtr($input, &argp, $descriptor(rw::core::Ptr<rw::models::Device const> *),SWIG_POINTER_NO_NULL | 0);
	if (SWIG_IsOK(res)) {
		rw::core::Ptr<rw::models::Device const> * tmp_var = reinterpret_cast< rw::core::Ptr<rw::models::Device const> * > (argp);
		$1 = *tmp_var;
	}
	else {
		int res = SWIG_ConvertPtr($input, &argp, $descriptor(rw::core::Ptr<rw::models::CompositeDevice const> *),SWIG_POINTER_NO_NULL | 0);
		if (SWIG_IsOK(res)) {
			rw::core::Ptr<rw::models::CompositeDevice const> * tmp_var = reinterpret_cast< rw::core::Ptr<rw::models::CompositeDevice const> *> (argp);
			$1 = tmp_var->cast<rw::models::Device const>();
		}
		else {
			SWIG_exception_fail(SWIG_ArgError(res), "could not convert to type rw::core::Ptr<rw::models::Device const>");
		}
	}
}
#endif 

%{
	#include<rw/models/CompositeJointDevice.hpp>
%}
%include <rw/models/CompositeJointDevice.hpp>
NAMED_OWNEDPTR(CompositeJointDevice,rw::models::CompositeJointDevice);

%{
	#include<rw/models/ControllerModel.hpp>
%}
%include <rw/models/ControllerModel.hpp>
NAMED_OWNEDPTR(ControllerModel,rw::models::ControllerModel);
%template(ControllerModelPtrVector) std::vector<rw::core::Ptr<rw::models::ControllerModel> >;

%ignore rw::models::DeformableObject::getNode(int,rw::kinematics::State const &) const;
%{
	#include<rw/models/DeformableObject.hpp>
%}
%include <rw/models/DeformableObject.hpp>
NAMED_OWNEDPTR(DeformableObject,rw::models::DeformableObject);
%template (DeformableObjectPtrVector) std::vector<rw::core::Ptr<rw::models::DeformableObject>>;

%{
	#include<rw/models/DependentJoint.hpp>
%}
%include <rw/models/DependentJoint.hpp>
NAMED_OWNEDPTR(DependentJoint,rw::models::DependentJoint);

%{
	#include<rw/models/DependentPrismaticJoint.hpp>
%}
%include <rw/models/DependentPrismaticJoint.hpp>
NAMED_OWNEDPTR(DependentPrismaticJoint,rw::models::DependentPrismaticJoint);

%{
	#include<rw/models/DependentRevoluteJoint.hpp>
%}
%include <rw/models/DependentRevoluteJoint.hpp>
NAMED_OWNEDPTR(DependentRevoluteJoint,rw::models::DependentRevoluteJoint);

%{
	#include<rw/models/DeviceJacobianCalculator.hpp>
%}
%include <rw/models/DeviceJacobianCalculator.hpp>
NAMED_OWNEDPTR(DeviceJacobianCalculator,rw::models::DeviceJacobianCalculator);

%nodefaultctor DHParameterSet;
%{
	#include<rw/models/DHParameterSet.hpp>
%}
%include <rw/models/DHParameterSet.hpp>
NAMED_OWNEDPTR(DHParameterSet,rw::models::DHParameterSet);
%template (DHParameterSetVector) std::vector<rw::models::DHParameterSet>;

%{
	#include<rw/models/JacobianUtil.hpp>
%}
%include <rw/models/JacobianUtil.hpp>
NAMED_OWNEDPTR(JacobianUtil,rw::models::JacobianUtil);


%{
	#include<rw/models/JointDeviceJacobianCalculator.hpp>
%}
%include <rw/models/JointDeviceJacobianCalculator.hpp>
NAMED_OWNEDPTR(JointDeviceJacobianCalculator,rw::models::JointDeviceJacobianCalculator);

%{
	#include<rw/models/MobileDevice.hpp>
%}
%include <rw/models/MobileDevice.hpp>
NAMED_OWNEDPTR(MobileDevice,rw::models::MobileDevice);

%{
	#include<rw/models/Models.hpp>
%}
%include <rw/models/Models.hpp>
NAMED_OWNEDPTR(Models,rw::models::Models);

%{
	#include<rw/models/ParallelDevice.hpp>
%}
%include <rw/models/ParallelDevice.hpp>
NAMED_OWNEDPTR(ParallelDevice,rw::models::ParallelDevice);
%template (VectorParallelDevicePtr) std::vector<rw::core::Ptr<rw::models::ParallelDevice>>;

%{
	#include<rw/models/ParallelLeg.hpp>
%}
%include <rw/models/ParallelLeg.hpp>
NAMED_OWNEDPTR(ParallelLeg,rw::models::ParallelLeg);
    %template(VectorParallelLegPtr) std::vector<rw::core::Ptr<rw::models::ParallelLeg>>;
    %template(VectorParallelLeg_p) std::vector<rw::models::ParallelLeg*>;
    %template(VectorVectorParallelLeg_p) std::vector<std::vector<rw::models::ParallelLeg*>>;



%{
	#include<rw/models/PrismaticJoint.hpp>
%}
%include <rw/models/PrismaticJoint.hpp>
NAMED_OWNEDPTR(PrismaticJoint,rw::models::PrismaticJoint);

%{
	#include<rw/models/PrismaticSphericalJoint.hpp>
%}
%include <rw/models/PrismaticSphericalJoint.hpp>
NAMED_OWNEDPTR(PrismaticSphericalJoint,rw::models::PrismaticSphericalJoint);

%{
	#include<rw/models/PrismaticUniversalJoint.hpp>
%}
%include <rw/models/PrismaticUniversalJoint.hpp>
NAMED_OWNEDPTR(PrismaticUniversalJoint,rw::models::PrismaticUniversalJoint);

%{
	#include<rw/models/RevoluteJoint.hpp>
%}
%include <rw/models/RevoluteJoint.hpp>
NAMED_OWNEDPTR(RevoluteJoint,rw::models::RevoluteJoint);

%{
	#include<rw/models/RigidBodyInfo.hpp>
%}
%include <rw/models/RigidBodyInfo.hpp>
NAMED_OWNEDPTR(RigidBodyInfo,rw::models::RigidBodyInfo);

%{
	#include<rw/models/RigidObject.hpp>
%}
%include <rw/models/RigidObject.hpp>
NAMED_OWNEDPTR(RigidObject,rw::models::RigidObject);
%template (VectorRigidObjectPtr) std::vector<rw::core::Ptr < rw::models::RigidObject > >;

%{
	#include<rw/models/SE3Device.hpp>
%}
%include <rw/models/SE3Device.hpp>
NAMED_OWNEDPTR(SE3Device,rw::models::SE3Device);

%{
	#include<rw/models/SerialDevice.hpp>
%}
%include <rw/models/SerialDevice.hpp>
NAMED_OWNEDPTR(SerialDevice,rw::models::SerialDevice);
%template (VectorSerialDevicePtr) std::vector<rw::core::Ptr<rw::models::SerialDevice>>;

%{
	#include<rw/models/SphericalJoint.hpp>
%}
%include <rw/models/SphericalJoint.hpp>
NAMED_OWNEDPTR(SphericalJoint,rw::models::SphericalJoint);

%{
	#include<rw/models/TreeDevice.hpp>
%}
%include <rw/models/TreeDevice.hpp>
NAMED_OWNEDPTR(TreeDevice,rw::models::TreeDevice);
%template (VectorTreeDevicePtr) std::vector<rw::core::Ptr<rw::models::TreeDevice>>;

%{
	#include<rw/models/UniversalJoint.hpp>
%}
%include <rw/models/UniversalJoint.hpp>
NAMED_OWNEDPTR(UniversalJoint,rw::models::UniversalJoint);

%{
	#include<rw/models/VirtualJoint.hpp>
%}
%include <rw/models/VirtualJoint.hpp>
NAMED_OWNEDPTR(VirtualJoint,rw::models::VirtualJoint);

%{
	#include<rw/models/WorkCell.hpp>
%}
%include <rw/models/WorkCell.hpp>
NAMED_OWNEDPTR(WorkCell,rw::models::WorkCell);
//%template (WorkCellChangedEvent) rw::core::Event< rw::models::WorkCell::WorkCellChangedListener, int >;

%extend rw::models::WorkCell {
	/**
	 * @brief Returns MovableFrame with the specified name.
	 *
	 * If multiple frames has the same name, the first frame encountered
	 * will be returned. If no frame is found, the method returns NULL.
	 *
	 * @param name [in] name of Frame.
	 *
	 * @return The MovableFrame with name \b name or NULL if no such frame.
	 */
	rw::kinematics::MovableFrame* findMovableFrame(const std::string& name)
	{ 
		return $self->WorkCell::findFrame<rw::kinematics::MovableFrame>(name); 
	}

	/**
	 * @brief Returns FixedFrame with the specified name.
	 *
	 * If multiple frames has the same name, the first frame encountered
	 * will be returned. If no frame is found, the method returns NULL.
	 *
	 * @param name [in] name of Frame.
	 *
	 * @return The FixedFrame with name \b name or NULL if no such frame.
	 */
	rw::kinematics::FixedFrame* findFixedFrame(const std::string& name)
	{ 
		return $self->WorkCell::findFrame<rw::kinematics::FixedFrame>(name); 
	}

	/**
	 * @brief Returns all \b MovableFrames.
	 * @return all frames of type \b MovableFrames in the workcell
	 */
	std::vector<rw::kinematics::MovableFrame*> findMovableFrames() const
	{ 
		return $self->WorkCell::findFrames<rw::kinematics::MovableFrame>(); 
	}

	/**
	 * @brief Returns all \b FixedFrame.
	 * @return all frames of type \b FixedFrame in the workcell
	 */
	std::vector<rw::kinematics::FixedFrame*> findFixedFrames() const
	{ 
		return $self->WorkCell::findFrames<rw::kinematics::FixedFrame>(); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::JointDevice> findJointDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::JointDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::SerialDevice> findSerialDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::SerialDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::TreeDevice> findTreeDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::TreeDevice>(name); 
	}

	/**
	 * @brief The device named \b name of the workcell.
	 *
	 * NULL is returned if there is no such device.
	 *
	 * @param name [in] The device name
	 *
	 * @return The device named \b name or NULL if no such device.
	 */
	rw::core::Ptr<rw::models::ParallelDevice> findParallelDevice(const std::string& name)
	{ 
		return $self->WorkCell::findDevice<rw::models::ParallelDevice>(name); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b JointDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::JointDevice> > findJointDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::JointDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b SerialDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::SerialDevice> > findSerialDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::SerialDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b TreeDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::TreeDevice> > findTreeDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::TreeDevice>(); 
	}

	/**
	 * @brief Returns a vector with pointers to the Device(s) with a
	 * specific type \b ParallelDevice in the WorkCell
	 *
	 * @return vector with pointers to Device(s) of type T.
	 */
	std::vector < rw::core::Ptr<rw::models::ParallelDevice> > findParallelDevices()
	{ 
		return $self->WorkCell::findDevices<rw::models::ParallelDevice>(); 
	}

};
