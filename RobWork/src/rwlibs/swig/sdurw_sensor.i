%module sdurw_sensor

%include <stl.i>
%include <std_vector.i>
%include <rwlibs/swig/swig_macros.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>

%import <rwlibs/swig/ext_i/std.i>

%{
    #include <rw/geometry/IndexedTriMesh.hpp>
    #include <rw/kinematics/FixedFrame.hpp>
    #include <rw/kinematics/MovableFrame.hpp>
    #include <boost/mpl/equal_to.hpp>
    #include <boost/mpl/int.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
%}

%ignore rw::sensor::SensorModel::getPropertyMap() const;
%{
	#include<rw/sensor/SensorModel.hpp>
%}
%include <rw/sensor/SensorModel.hpp>
NAMED_OWNEDPTR(SensorModel,rw::sensor::SensorModel);
%template (VectorSensorModelPtr) std::vector<rw::core::Ptr<rw::sensor::SensorModel>>;

%ignore rw::sensor::Sensor::getPropertyMap() const;
%{
	#include<rw/sensor/Sensor.hpp>
%}
%include <rw/sensor/Sensor.hpp>
NAMED_OWNEDPTR(Sensor,rw::sensor::Sensor);

%ignore rw::sensor::Image::getImageData() const ;
%{
	#include<rw/sensor/Image.hpp>
%}
%include <rw/sensor/Image.hpp>
NAMED_OWNEDPTR(Image,rw::sensor::Image);


%{
	#include<rw/sensor/Camera.hpp>
%}
%include <rw/sensor/Camera.hpp>
NAMED_OWNEDPTR(Camera,rw::sensor::Camera);


%{
	#include<rw/sensor/CameraFirewire.hpp>
%}
%include <rw/sensor/CameraFirewire.hpp>
NAMED_OWNEDPTR(CameraFirewire,rw::sensor::CameraFirewire);

%{
	#include<rw/sensor/CameraListener.hpp>
%}
%include <rw/sensor/CameraListener.hpp>
NAMED_OWNEDPTR(CameraListener,rw::sensor::CameraListener);

%{
	#include<rw/sensor/CameraModel.hpp>
%}
%include <rw/sensor/CameraModel.hpp>
NAMED_OWNEDPTR(CameraModel,rw::sensor::CameraModel);

%{
	#include<rw/sensor/Contact2D.hpp>
%}
%include <rw/sensor/Contact2D.hpp>
NAMED_OWNEDPTR(Contact2D,rw::sensor::Contact2D);

%{
	#include<rw/sensor/Contact3D.hpp>
%}
%include <rw/sensor/Contact3D.hpp>
NAMED_OWNEDPTR(Contact3D,rw::sensor::Contact3D);
%template(VectorContact3D) std::vector<rw::sensor::Contact3D>;

%{
	#include<rw/sensor/FTSensor.hpp>
%}
%include <rw/sensor/FTSensor.hpp>
NAMED_OWNEDPTR(FTSensor,rw::sensor::FTSensor);

%{
	#include<rw/sensor/FTSensorModel.hpp>
%}
%include <rw/sensor/FTSensorModel.hpp>
NAMED_OWNEDPTR(FTSensorModel,rw::sensor::FTSensorModel);

%{
	#include<rw/sensor/ImageUtil.hpp>
%}
%include <rw/sensor/ImageUtil.hpp>
NAMED_OWNEDPTR(ImageUtil,rw::sensor::ImageUtil);

%{
	#include<rw/sensor/RGBDCameraModel.hpp>
%}
%include <rw/sensor/RGBDCameraModel.hpp>
NAMED_OWNEDPTR(RGBDCameraModel,rw::sensor::RGBDCameraModel);

%{
	#include<rw/sensor/Scanner.hpp>
%}
%include <rw/sensor/Scanner.hpp>
NAMED_OWNEDPTR(Scanner,rw::sensor::Scanner);

%{
	#include<rw/sensor/Scanner1D.hpp>
%}
%include <rw/sensor/Scanner1D.hpp>
NAMED_OWNEDPTR(Scanner1D,rw::sensor::Scanner1D);

%{
	#include<rw/sensor/Scanner25D.hpp>
%}
%include <rw/sensor/Scanner25D.hpp>
NAMED_OWNEDPTR(Scanner25D,rw::sensor::Scanner25D);

%{
	#include<rw/sensor/Scanner25DModel.hpp>
%}
%include <rw/sensor/Scanner25DModel.hpp>
NAMED_OWNEDPTR(Scanner25DModel,rw::sensor::Scanner25DModel);

%{
	#include<rw/sensor/Scanner2D.hpp>
%}
%include <rw/sensor/Scanner2D.hpp>
NAMED_OWNEDPTR(Scanner2D,rw::sensor::Scanner2D);

%{
	#include<rw/sensor/Scanner2DModel.hpp>
%}
%include <rw/sensor/Scanner2DModel.hpp>
NAMED_OWNEDPTR(Scanner2DModel,rw::sensor::Scanner2DModel);

%{
	#include<rw/sensor/StereoCameraModel.hpp>
%}
%include <rw/sensor/StereoCameraModel.hpp>
NAMED_OWNEDPTR(StereoCameraModel,rw::sensor::StereoCameraModel);



%ignore rw::sensor::TactileArrayModel::getTexelData(rw::kinematics::State const &) const;
%{
	#include<rw/sensor/TactileArrayModel.hpp>
%}
%include <rw/sensor/TactileArrayModel.hpp>
NAMED_OWNEDPTR(TactileArrayModel,rw::sensor::TactileArrayModel);
%template(TactileVertexMatrix) boost::multi_array< rw::math::Vector3D<double>, 2 >;

%{
	#include<rw/sensor/TactileArray.hpp>
%}
%include <rw/sensor/TactileArray.hpp>
NAMED_OWNEDPTR(TactileArray,rw::sensor::TactileArray);

%{
	#include<rw/sensor/TactileArrayUtil.hpp>
%}
%include <rw/sensor/TactileArrayUtil.hpp>
NAMED_OWNEDPTR(TactileArrayUtil,rw::sensor::TactileArrayUtil);
