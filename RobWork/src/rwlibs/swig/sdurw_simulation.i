%module sdurw_simulation

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/models.hpp>
#include <rw/core/Ptr.hpp>

using namespace rwlibs::swig;
%}

%include <std_vector.i>
%include <exception.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_kinematics.i>
%import <rwlibs/swig/sdurw_geometry.i>
%import <rwlibs/swig/sdurw_sensor.i>
%import <rwlibs/swig/sdurw_models.i>
%import <rwlibs/swig/sdurw.i>
%import <rwlibs/swig/sdurw_control.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw.*;
import org.robwork.sdurw_control.*;
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_kinematics.*;
import org.robwork.sdurw_geometry.*;
import org.robwork.sdurw_sensor.*;
import org.robwork.sdurw_models.*;
%}

#if (defined(SWIGPYTHON) || defined(SWIGLUA))
%feature("flatnested");
#endif

%nodefaultctor SimulatedSensor;
/**
 * @brief simulated sensor interface
 */
class SimulatedSensor {
public:
    //! @brief destructor
    virtual ~SimulatedSensor();

    /**
     * @brief get name of this simulated sensor
     */
    const std::string& getName() const;

    /**
     * @brief get frame that this sensor is attached to.
     *
     * @return frame
     */
    rw::kinematics::Frame* getFrame() const;

    /**
     * @brief steps the the SimulatedSensor with time \b dt and saves any state
     * changes in \b state.
     *
     * @param info [in] update information related to the time step.
     * @param state [out] changes of the SimulatedSensor is saved in state.
     */
    virtual void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state) = 0;

    /**
     * @brief Resets the state of the SimulatedSensor to that of \b state
     *
     * @param state [in] the state that the sensor is reset too.
     */
    virtual void reset(const rw::kinematics::State& state) = 0;

    /**
     * @brief get the sensor model of this simulated sensor. 
     */
    rw::core::Ptr<rw::sensor::SensorModel> getSensorModel();

    /**
     * @brief get a handle to controlling an instance of the simulated sensor in a specific simulator
     *
     * @param sim [in] the simulator in which the handle is active
     */
    rw::core::Ptr<rw::sensor::Sensor> getSensorHandle(rw::core::Ptr<Simulator> sim);
};

%template (SimulatedSensorPtr) rw::core::Ptr<SimulatedSensor>;
%template (SimulatedSensorPtrVector) std::vector<rw::core::Ptr<SimulatedSensor> >;

%nodefaultctor SimulatedController;
/**
 * @brief interface of a simulated controller
 */
class SimulatedController {
public:
    /**
     *  @brief get the name of this controller
     *
     *  @return name of this controller
     */
    virtual std::string getControllerName() = 0;

    /**
     * @brief updates/steps the controller with time step \b dt. It will update
     * the state \b state accordingly
     *
     * @param info [in] update information related to the time step.
     * @param state [in/out] the current state
     */
    virtual void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state) = 0;

    /**
     * @brief reset the controller to the applied state
     *
     * @param state [in] the state to reset to
     */
    virtual void reset(const rw::kinematics::State& state) = 0;

    /**
     * @brief get the controller handle eg. statefull handle, associated with this simulated controller
     *
     * @return controller handle
     */
    virtual rw::core::Ptr<Controller> getControllerHandle(rw::core::Ptr<Simulator> sim) = 0;

    /**
     * @brief disable or enable this controller
     *
     * @param enabled
     */
    virtual void setEnabled(bool enabled) = 0;

    /**
     * @brief true if this controller is enabled
     *
     * @return true if this controller is enabled
     */
    virtual bool isEnabled() const = 0;
};

%template (SimulatedControllerPtr) rw::core::Ptr<SimulatedController>;
%template (SimulatedControllerPtrVector) std::vector<rw::core::Ptr<SimulatedController> >;

%nodefaultctor Simulator;
class Simulator {
public:
   struct UpdateInfo {
	   UpdateInfo();
	   UpdateInfo(double dt_step);

	   double dt;
	   double dt_prev;
	   double time;
	   bool rollback;
   };
   
   virtual ~Simulator();
   virtual void step(double dt) = 0;
   virtual void reset(rw::kinematics::State& state) = 0;
   virtual void init(rw::kinematics::State& state) = 0;
   virtual double getTime() = 0;
   virtual void setEnabled(rw::kinematics::Frame* frame, bool enabled) = 0;
   virtual rw::kinematics::State& getState() = 0;
   virtual rw::core::PropertyMap& getPropertyMap() = 0;

};

%template (SimulatorPtr) rw::core::Ptr<Simulator>;

/**
 * @brief The FrameGrabber abstract interface, can be used to grab images from a
 * specialized source.
 */
class FrameGrabber
{
public:
    /**
     * @brief constructor
     *
     * @param width [in] width of the image that this FrameGrabber uses.
     * @param height [in] height of the image that this FrameGrabber uses.
     * @param encoding [in] color encoding of the image that this FrameGrabber uses.
     */
    FrameGrabber(int width, int height, rw::sensor::Image::ColorCode encoding);

    /**
     * @brief destructor
     */
    virtual ~FrameGrabber();

    /**
     * @brief returns the width of the image
     *
     * @return the width of the image
     */
    int getWidth();

    /**
     * @brief returns the height of the image
     *
     * @return the height of the image
     */
    int getHeight();
	
    /**
     * @brief resizes the image that this frameGrabber use. The colorcode will
     * default to the one that FrameGrabber was initialized with.
     *
     * @param width [in] width of image
     * @param height [in] height of image
     */
    virtual void resize(int width, int height);

    /**
     * @brief resizes the image that this frameGrabber use.
     *
     * @param width [in] width of image.
     * @param height [in] height of image.
     * @param colorCode [in] Color encoding of the image.
     */
    virtual void resize(int width, int height, rw::sensor::Image::ColorCode colorCode);

    /**
     * @brief returns the image
     *
     * @return the image
     */
    virtual rw::sensor::Image& getImage();

    /**
     * @brief this function grabs a image from the specialized source and
     * copies it to the FrameGrabber image.
     */
    virtual void grab(rw::kinematics::Frame *frame, const rw::kinematics::State& state) = 0;
};

%template (FrameGrabberPtr) rw::core::Ptr<FrameGrabber>;

/**
 * @brief The FrameGrabber25D abstract interface, can be used to grab images from a
 * specialized source.
 */
class FrameGrabber25D
{
public:
    /**
     * @brief constructor
     *
     * @param width [in] width of the image that this FrameGrabber25D uses.
     * @param height [in] height of the image that this FrameGrabber25D uses.
     */
    FrameGrabber25D(size_t width, size_t height);

    /**
     * @brief destructor
     */
    virtual ~FrameGrabber25D();

    /**
     * @brief returns the width of the image
     *
     * @return the height of the image
     */
    size_t getWidth() const;

    /**
     * @brief returns the height of the image
     *
     * @return the height of the image
     */
    size_t getHeight() const;


    /**
     * @brief Returns the field of view measured around the y-axis.
     *
     * @return Field of view measured around y-axis in radians
     */
    virtual double getFieldOfViewY() = 0;

    /**
     * @brief resizes the image that this frameGrabber use. The colorcode will
     * default to the one that FrameGrabber25D was initialized with.
     *
     * @param width [in] width of image
     * @param height [in] height of image
     */
    void resize(size_t width, size_t height);

    /**
     * @brief returns the image
     *
     * @return the image
     */
    virtual rw::geometry::PointCloud& getImage();

    virtual void grab(rw::kinematics::Frame *frame, const rw::kinematics::State& state) = 0;

    /**
     * @brief maximum depth that this framegrabber can handle
     *
     * @return maximum depth in meter
     */
    virtual double getMaxDepth() = 0;

    /**
     * @brief minimum depth that this framegrabber can handle
     *
     * @return minimum depth in meter
     */
    virtual double getMinDepth() = 0;
};

%template (FrameGrabber25DPtr) rw::core::Ptr<FrameGrabber25D>;

/**
 * @brief An implementation of the FrameGrabber interface. The GLFrameGrabber
 * grabs images from a OpenGL scene using a simple pinhole camera model.
 *
 * a framethe opengl rendering to
 * take pictures of the scene.

 * The most basic parameter of a camera is its Field of view. This
 * can be used as an initial camera model. Field of view can be
 * calculated from the focal length and the size of the CCD
 * typically (1/2, 1/3, 1/4) inch.
 * If a more realistic camera model is
 * required the perspective transform of a specific camera can be added
 */
class GLFrameGrabber : public FrameGrabber
{
public:
    /**
     * @brief constructor
     *
     * @param width [in] width of image
     * @param height [in] height of image
     * @param fov [in] the vertical field of view angle in degree
     * @param near [in] the minimum depth of camera.
     * @param far [in] the maximum depth of camera.
     */
    GLFrameGrabber(int width, int height,
                   double fov,
                   double near=0.1, double far=10.0);

    /**
     * @brief destructor
     */
    virtual ~GLFrameGrabber();

    void resize(int width, int height);

    void resize(int width, int height, rw::sensor::Image::ColorCode colorCode);

    /**
     * @brief initialize the grabber with a scene viewer. This registers the grabber
     * as a camera in the scene and enables rendering.
     *
     * @param drawer [in] the scene viewer
     * @return true if initialization succeeded, false otherwise (depends on the capabilities of the SceneViewer).
     */
    bool init(rw::core::Ptr<SceneViewer> drawer);

    //! @copydoc FrameGrabber::grab
    void grab(rw::kinematics::Frame* frame, const rw::kinematics::State& state);
};

%template (GLFrameGrabberPtr) rw::core::Ptr<GLFrameGrabber>;
OWNEDPTR(GLFrameGrabber)

%extend rw::core::Ptr<GLFrameGrabber> {
    rw::core::Ptr<FrameGrabber> asFrameGrabberPtr() { return *$self; }
}

/**
 * @brief An implementation of the FrameGrabber interface. The GLFrameGrabber25D
 * grabs images from a OpenGL scene using a simple pinhole camera model.
 *
 * a framethe opengl rendering to
 * take pictures of the scene.
 *
 * The most basic parameter of a camera is its Field of view. This
 * can be used as an initial camera model. Field of view can be
 * calculated from the focal length and the size of the CCD
 * typically (1/2, 1/3, 1/4) inch.
 * If a more realistic camera model is
 * required the perspective transform of a specific camera can be added
 */
class GLFrameGrabber25D : public FrameGrabber25D
{
public:
    /**
     * @brief constructor
     *
     * @param width [in] width of image
     * @param height [in] height of image
     * @param fov [in] the vertical field of view angle in degree
     * @param mindepth [in] the minimum depth of camera.
     * @param maxdepth [in] the maximum depth of camera.
     */
    GLFrameGrabber25D(int width, int height, double fov, double mindepth=0.1, double maxdepth=10.0);

    /**
     * @brief destructor
     */
    virtual ~GLFrameGrabber25D();

    /**
     * @brief initialize the grabber with a scene viewer. This registers the grabber
     * as a camera in the scene and enables rendering.
     *
     * @param drawer [in] the scene viewer
     * @return true if initialization succeeded, false otherwise (depends on the capabilities of the SceneViewer).
     */
    bool init(rw::core::Ptr<SceneViewer> drawer);

    /**
     * @brief set the maximum depth that is percieved by this frame grabber.
     * If min and max depth are too far apart the resolution of the depth
     * perception will become bad. Hence keep the range realistic.
     *
     * @param depth [in] max depth
     */
    void setMaxDepth(double depth);

    /**
     * @brief set the minimum depth that is percieved by this frame grabber.
     * If min and max depth are too far apart the resolution of the depth
     * perception will become bad. Hence keep the range realistic.
     *
     * @param depth [in] min depth
     */
    void setMinDepth(double depth);

    //! @copydoc FrameGrabber::grab
    void grab(rw::kinematics::Frame* frame, const rw::kinematics::State& state);

    //! @copydoc FrameGrabber25D::getMaxDepth()
    double getMaxDepth();

    //! @copydoc FrameGrabber25D::getMinDepth()
    double getMinDepth();

    /**
     * @copydoc FrameGrabber25D::getFieldOfViewY()
     */
    virtual double getFieldOfViewY();
};

%template (GLFrameGrabber25DPtr) rw::core::Ptr<GLFrameGrabber25D>;
OWNEDPTR(GLFrameGrabber25D)

%extend rw::core::Ptr<GLFrameGrabber25D> {
    rw::core::Ptr<FrameGrabber25D> asFrameGrabber25DPtr() { return *$self; }
}

/**
 * @brief The SimulatedCamera class makes it posible to use virtual camera
 * sensors by using different framegrapper implementations.
 *
 * The SimulatedCamera implements the camera interface though the setting of
 * framerate has no meaning to the virtual camera since no timing is done in
 * this implementation.
 */
class SimulatedCamera : public SimulatedSensor
{
public:
    /**
     * @brief creates a simulated pinhole camera.
     *
     * @param name [in] name of sensor
     * @param fov [in] field of view for the camera.
     * @param frame [in] frame to which the camera is attached
     * @param frameGrabber [in] the frameGrabber from which this Camera should grab images
     */
    SimulatedCamera(const std::string& name, double fov, rw::kinematics::Frame* frame, rw::core::Ptr<FrameGrabber> frameGrabber);

    /**
     * @brief constructor
     *
     * @param model [in] the model and info of the camera
     * @param frameGrabber [in] the frameGrabber from which this Camera should grab
     * images.
     */
    SimulatedCamera(rw::core::Ptr<rw::sensor::CameraModel> model, rw::core::Ptr<FrameGrabber> frameGrabber);

    /**
     * @brief destructor
     */
    virtual ~SimulatedCamera();

    /**
     * @copydoc rw::sensor::Camera::initialize
     */
    bool initialize();

    /**
     * @copydoc rw::sensor::Camera::start
     */
    bool start();

    /**
     * @copydoc rw::sensor::Camera::stop
     */
    void stop();

    /**
     * @copydoc rw::sensor::Camera::acquire
     */
    void acquire();

    /**
     * @copydoc rw::sensor::Camera::isImageReady
     */
    bool isImageReady();

    /**
     * @copydoc rw::sensor::Camera::getImage
     */
    const rw::sensor::Image* getImage();

    /**
     * @copydoc rw::sensor::Camera::getFrameRate
     */
    double getFrameRate();

    /**
     * @copydoc rw::sensor::Camera::setFrameRate
     */
    void setFrameRate(double framerate);

    /**
     * @copydoc rw::sensor::Camera::getWidth
     */
    virtual unsigned int getWidth() const;

    /**
     * @copydoc rw::sensor::Camera::getHeight
     */
    virtual unsigned int getHeight() const;

    /**
     * @copydoc SimulatedSensor::update
     */
    void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

    /**
     * @copydoc SimulatedSensor::reset
     */
    void reset(const rw::kinematics::State& state);

    rw::core::Ptr<rw::sensor::Sensor> getSensor();

    /**
     * @brief Get the camera sensor.
     *
     * @return the sensor.
     */
    rw::core::Ptr<rw::sensor::Camera> getCameraSensor();
};

%template (SimulatedCameraPtr) rw::core::Ptr<SimulatedCamera>;
OWNEDPTR(SimulatedCamera)

/**
 * @brief Simulated scanner in 2D.
 */
class SimulatedScanner2D : public SimulatedSensor
{
public:
    /**
     * @brief constructor
     *
     * @param name [in] name of this simulated scanner
     * @param frame [in] the sensor frame.
     * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
     */
    SimulatedScanner2D(const std::string& name, rw::kinematics::Frame* frame,
    		rw::core::Ptr<FrameGrabber25D> framegrabber);

    /**
     * @brief constructor
     *
     * @param name [in] name of this simulated scanner
     * @param desc [in] description of this scanner
     * @param frame [in] the sensor frame.
     * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
     */
    SimulatedScanner2D(const std::string& name,
            const std::string& desc,
            rw::kinematics::Frame* frame,
			rw::core::Ptr<FrameGrabber25D> framegrabber);

    /**
     * @brief destructor
     */
    virtual ~SimulatedScanner2D();

    /**
     * @brief set the framerate in frames per sec.
     *
     * @param rate [in] frames per sec
     */
    void setFrameRate(double rate);

    ///////////// below is inheritet functions form Scanner25D and Sensor

    //! @copydoc rw::sensor::Scanner2D::open
    void open();

    //! @copydoc rw::sensor::Scanner2D::isOpen
    bool isOpen();

    //! @copydoc rw::sensor::Scanner2D::close
    void close();

    //! @copydoc rw::sensor::Scanner2D::acquire
    void acquire();

    //! @copydoc rw::sensor::Scanner2D::isScanReady
    bool isScanReady();

    //! @copydoc rw::sensor::Scanner2D::getFrameRate
    double getFrameRate();

    //! @copydoc rwlibs::simulation::SimulatedKinect::getScan
    const rw::geometry::PointCloud& getScan() const;

    //! @copydoc SimulatedSensor::update
    void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

    //! @copydoc SimulatedSensor::reset
    void reset(const rw::kinematics::State& state);

    /**
     * @brief returns a handle to what represents a statefull interface.
     * The handle will be locked to the simulator
     *
     * @return Scnner2D handle.
     */
    rw::core::Ptr<rw::sensor::Scanner2D> getScanner2DSensor(Simulator* instance);

    //! @copydoc rw::sensor::Scanner2DModel::getAngularRange
    virtual double getAngularRange();

    virtual size_t getMeasurementCount() const;
};

%template (SimulatedScanner2DPtr) rw::core::Ptr<SimulatedScanner2D>;
OWNEDPTR(SimulatedScanner2D)

/**
 * @brief a simulated range scanner for 2.5D images, that is basically
 * pointclouds without color information.
 */
class SimulatedScanner25D : public SimulatedSensor
{
public:
    /**
     * @brief constructor
     *
     * @param name [in] name of this simulated scanner
     * @param frame [in] the frame the scanner is attached to.
     * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
     */
	SimulatedScanner25D(const std::string& name,
	                    rw::kinematics::Frame *frame,
						rw::core::Ptr<FrameGrabber25D> framegrabber);

    /**
     * @brief constructor
     *
     * @param name [in] name of this simulated scanner
     * @param desc [in] description of this scanner
     * @param frame [in] the frame the scanner is attached to.
     * @param framegrabber [in] the framegrabber used for grabbing 2.5D images
     */
	SimulatedScanner25D(const std::string& name,
                        const std::string& desc,
                        rw::kinematics::Frame *frame,
						rw::core::Ptr<FrameGrabber25D> framegrabber);

	/**
	 * @brief destructor
	 */
	virtual ~SimulatedScanner25D();

	/**
	 * @brief set the framerate in frames per sec.
	 *
	 * @param rate [in] frames per sec
	 */
	void setFrameRate(double rate);

	///////////// below is inheritet functions form Scanner25D and Sensor

	//! @copydoc rw::sensor::Scanner25D::open
    void open();

    //! @copydoc rw::sensor::Scanner25D::isOpen
    bool isOpen();

    //! @copydoc rw::sensor::Scanner25D::close
    void close();

    //! @copydoc rw::sensor::Scanner25D::acquire
    void acquire();

    //! @copydoc rw::sensor::Scanner25D::isScanReady
    bool isScanReady();

    //! @copydoc rw::sensor::Scanner25D::getFrameRate
    double getFrameRate();

    //! @copydoc rw::geometry::PointCloud::getData
	const rw::geometry::PointCloud& getScan();

	//! @copydoc SimulatedSensor::update
    void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state);

	//! @copydoc SimulatedSensor::reset
	void reset(const rw::kinematics::State& state);

    /**
	 * @brief get a handle to controlling an instance of the simulated sensor in a specific simulator
	 *
     * @param instance [in] the simulator in which the handle is active
     */
    rw::core::Ptr<rw::sensor::Sensor> getSensorHandle(rw::core::Ptr<Simulator> instance);

	//! get instance of scanner
    rw::core::Ptr<rw::sensor::Scanner25D> getScanner25DSensor(rw::core::Ptr<Simulator> instance);

private:
    rw::core::Ptr<FrameGrabber25D> _framegrabber;
    double _frameRate, _dtsum;
    bool _isAcquired,_isOpenned;
};

%template (SimulatedScanner25DPtr) rw::core::Ptr<SimulatedScanner25D>;
OWNEDPTR(SimulatedScanner25D)
