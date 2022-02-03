/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_MODELS_WORKCELL_HPP
#define RW_MODELS_WORKCELL_HPP

/**
 * @file WorkCell.hpp
 */
#if !defined(SWIG)
#include <rw/core/Event.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/core/macros.hpp>
#include <rw/kinematics/State.hpp>

#include <boost/function.hpp>
#include <iosfwd>
#include <string>
#include <vector>
#endif

// Forward declarations
namespace rw { namespace core {
    class PropertyMap;
}}    // namespace rw::core
namespace rw { namespace graphics {
    class SceneDescriptor;
}}    // namespace rw::graphics
namespace rw { namespace kinematics {
    class StateStructure;
    class Frame;
}}    // namespace rw::kinematics
namespace rw { namespace proximity {
    class CollisionSetup;
}}    // namespace rw::proximity
namespace rw { namespace sensor {
    class SensorModel;
}}    // namespace rw::sensor

namespace rw { namespace models {

    // Forward declarations
    class ControllerModel;
    class Device;
    class Object;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief WorkCell keeps track of devices, obstacles and objects in the
     * scene.
     *
     * WorkCell is a pretty dumb container to which you can add your devices and
     * the frames you your GUI to show as objects or camera views.
     *
     * WorkCell is responsible for keeping track of everything including all
     * devices, object and obstacles in the environment. WorkCell contains the
     * World Frame, which represents the root and the only frame without a
     * parent.
     */
    class WorkCell
    {
      public:
        //! @brief Smart pointer type to a WorkCell object
        typedef rw::core::Ptr< WorkCell > Ptr;
        //! @brief Smart pointer type to a constant WorkCell object
        typedef rw::core::Ptr< const WorkCell > CPtr;

        //! @brief WorkCell Event Types.
        typedef enum {
            //! StateData was added to state structure.
            STATE_DATA_ADDED,
            //! StateData was removed from state structure.
            STATE_DATA_REMOVED,
            /**
             * WorkCell changed (such as addition or removal of Device,
             * Frame, Object, SensorModel, or ControllerModel).
             */
            WORKCELL_CHANGED
        } WorkCellEventType;

        /**
         * @brief Constructs an empty WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was
         * loaded from.
         */
        WorkCell (const std::string& name);

        /**
         * @brief Constructs a WorkCell
         *
         * @param tree [in] The (initial) tree structure of the WorkCell
         *
         * @param name [in] The name of the workcell. A good name for the
         * workcell would be the (eventual) file that the workcell was
         * loaded from.
         *
         * @param filename [in] The filename from which the workcell is
         * loaded.
         */
        WorkCell (rw::core::Ptr< rw::kinematics::StateStructure > tree,
                  const std::string& name = "", const std::string& filename = "");

        /**
         * Destroys a work cell including the devices that have been added.
         *
         * Management of the frames is done by a tree of which the work cell
         * knows nothing. Therefore if this kinematics tree is still in
         * existence (which it probably is), then the frames that used to be
         * accessible via this work cell will still be valid.
         */
        ~WorkCell ();

        /**
         * @brief The name of the workcell or the empty string if no name
         * was provided.
         * @return the name of the workcell
         */
        std::string getName () const { return _name; }

        /**
         * @brief Returns pointer to the world frame
         *
         * @return Pointer to the world frame
         */
        rw::kinematics::Frame* getWorldFrame () const;

#if !defined(SWIGJAVA)
/**
 * @brief Adds \b frame with \b parent as parent.
 *
 * If parent == NULL, then \b world is used as parent
 *
 * @param frame [in] Frame to add
 * @param parent [in] Parent frame - uses World is parent == NULL
 * @deprecated Since January 2018.
 * Please use the addFrame method using smart pointers instead.
 */
#if !defined(SWIG)
        DEPRECATED ("Use Frame::Ptr insted of Frame*")
#endif
        void addFrame (rw::kinematics::Frame* frame, rw::kinematics::Frame* parent = NULL);
#endif
        /**
         * @brief Adds \b frame with \b parent as parent.
         *
         * If parent == NULL, then \b world is used as parent
         *
         * @param frame [in] Frame to add
         * @param parent [in] Parent frame - uses World is parent == NULL
         */
        void addFrame (rw::core::Ptr< rw::kinematics::Frame > frame,
                       rw::core::Ptr< rw::kinematics::Frame > parent = NULL);
#if !defined(SWIGJAVA)
/**
 * @brief Adds dynamically attachable frame (DAF) \b frame with
 * \b parent as parent.
 *
 * If parent == NULL, then \b world is used as parent
 *
 * @param frame [in] Frame to add
 * @param parent [in] Parent frame - uses World is parent == NULL
 * @deprecated Since January 2018.
 * Please use the addDAF method using smart pointers instead.
 */
#if !defined(SWIG)
        DEPRECATED ("Use Frame::Ptr insted of Frame*")
#endif
        void addDAF (rw::kinematics::Frame* frame, rw::kinematics::Frame* parent = NULL);
#endif
        /**
         * @brief Adds dynamically attachable frame (DAF) \b frame with
         * \b parent as parent.
         *
         * If parent == NULL, then \b world is used as parent
         *
         * @param frame [in] Frame to add
         * @param parent [in] Parent frame - uses World is parent == NULL
         */
        void addDAF (rw::core::Ptr< rw::kinematics::Frame > frame,
                     rw::core::Ptr< rw::kinematics::Frame > parent = NULL);

        /**
         * @brief Removes \b frame from work cell
         *
         * @param frame [in] Frame to remove
         * @deprecated Since January 2018.
         * Please use remove(rw::core::Ptr<rw::kinematics::Frame>)
         * instead.
         */
        void remove (rw::kinematics::Frame* frame);

        /**
         * @brief Removes \b frame from work cell
         *
         * @param frame [in] Frame to remove
         */
        void remove (rw::core::Ptr< rw::kinematics::Frame > frame);

        /**
         * @brief Removes \b object from workcell
         *
         * @param object [in] Object to remove
         */
        void removeObject (Object* object);

        /**
         * @brief Adds a Device to the WorkCell.
         *
         * Ownership of \b device is taken.
         *
         * @param device [in] pointer to device.
         */
        void addDevice (rw::core::Ptr< rw::models::Device > device);

        /**
         * @brief Returns a reference to a vector with pointers to the
         * Device(s) in the WorkCell
         *
         * @return const vector with pointers to rw::models::Device(s).
         */
        const std::vector< rw::core::Ptr< rw::models::Device > >& getDevices () const;

        /**
         * @brief Returns frame with the specified name.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         *
         * @param name [in] name of Frame.
         *
         * @return The frame with name \b name or NULL if no such frame.
         */
        rw::kinematics::Frame* findFrame (const std::string& name) const;

        /**
         * @brief Returns frame with the specified name and type \b T.
         *
         * If multiple frames has the same name, the first frame encountered
         * will be returned. If no frame is found, the method returns NULL.
         * if a frame is found and it is nt of type \b T then NULL is
         * returned.
         *
         * @param name [in] name of Frame.
         *
         * @return The frame with name \b name or NULL if no such frame or
         * the frame is not of type \b T.
         */
        template< class T > T* findFrame (const std::string& name) const
        {
            rw::kinematics::Frame* frame = findFrame (name);
            if (frame == NULL)
                return NULL;
            return dynamic_cast< T* > (frame);
        }

        /**
         * @brief Returns all frames of a specific type \b T.
         * @return all frames of type \b T in the workcell
         */
        template< class T > std::vector< T* > findFrames () const
        {
            using rw::kinematics::Frame;
            const std::vector< Frame* > frames = getFrames ();
            std::vector< T* > result;
            for (Frame* f : frames) {
                T* res = dynamic_cast< T* > (f);
                if (res != NULL)
                    result.push_back (res);
            }
            return result;
        }

        /**
         * @brief Returns all frames in workcell
         * @return List of all frames
         */
        std::vector< rw::kinematics::Frame* > getFrames () const;

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device.
         *
         * @param name [in] The device name
         *
         * @return The device named \b name or NULL if no such device.
         */
        rw::core::Ptr< rw::models::Device > findDevice (const std::string& name) const;

        /**
         * @brief The device named \b name of the workcell.
         *
         * NULL is returned if there is no such device or if the device is
         * not of type \b T.
         *
         * @param name [in] The workcell name
         * @return The device named \b name or NULL if no such device is
         * found or if the device is not of type \b T.
         */
        template< class T > rw::core::Ptr< T > findDevice (const std::string& name) const
        {
            rw::core::Ptr< rw::models::Device > dev = findDevice (name);
            if (dev == NULL)
                return NULL;
            return dev.cast< T > ();
        }

        /**
         * @brief Returns a vector with pointers to the Device(s) with a
         * specific type \b T in the WorkCell
         *
         * @return vector with pointers to Device(s) of type T.
         */
        template< class T > std::vector< rw::core::Ptr< T > > findDevices () const
        {
            std::vector< rw::core::Ptr< T > > result;
            for (rw::core::Ptr< rw::models::Device > dev : _devices) {
                rw::core::Ptr< T > res = dev.cast< T > ();
                if (res != NULL)
                    result.push_back (res);
            }
            return result;
        }

        /**
         * @brief Returns a default State
         *
         * @return default State
         */
        rw::kinematics::State getDefaultState () const;

        /**
         * @brief Returns sensor with the specified name.
         *
         * If multiple sensors has the same name, the first sensor
         * encountered will be returned. If no sensor is found, the method
         * returns NULL.
         *
         * @param name [in] name of sensor.
         *
         * @return The sensor with name \b name or NULL if no such sensor.
         */
        rw::core::Ptr< rw::sensor::SensorModel > findSensor (const std::string& name) const;

        /**
         * @brief Returns sensor with the specified name and type \b T.
         *
         * If multiple sensors has the same name, the first sensor encountered
         * will be returned. If no sensor is found, the method returns NULL.
         * if a sensor is found and it is nt of type \b T then NULL is returned.
         *
         * @param name [in] name of sensor.
         *
         * @return The sensor with name \b name or NULL if no such sensor or
         * the sensor is not of type \b T.
         */
        template< class T > rw::core::Ptr< T > findSensor (const std::string& name) const
        {
            rw::core::Ptr< rw::sensor::SensorModel > sensor = findSensor (name);
            if (sensor == NULL)
                return NULL;
            return sensor.cast< T > ();
        }

        /**
         * @brief Returns all frames of a specific type \b T.
         * @return all frames of type \b T in the workcell
         */
        template< class T > std::vector< rw::core::Ptr< T > > findSensors () const
        {
            const std::vector< rw::core::Ptr< rw::sensor::SensorModel > > sensors = _sensors;
            std::vector< rw::core::Ptr< T > > result;
            for (rw::core::Ptr< rw::sensor::SensorModel > f : sensors) {
                rw::core::Ptr< T > res = f.cast< T > ();
                if (res != NULL)
                    result.push_back (res);
            }
            return result;
        }

        /**
         * @brief Returns all frames in workcell
         * @return List of all frames
         */
        std::vector< rw::core::Ptr< rw::sensor::SensorModel > > getSensors () const;

        /**
         * @brief Returns controller with the specified name.
         *
         * If multiple controlelrs has the same name, the first controller
         * encountered will be returned. If no controller is found, the
         * method returns NULL.
         *
         * @param name [in] name of controller.
         *
         * @return The controller with name \b name or NULL if no such
         * controller.
         */
        rw::core::Ptr< rw::models::ControllerModel > findController (const std::string& name) const;

        /**
         * @brief Returns controller with the specified name and type \b T.
         *
         * @param name [in] name of controller.
         *
         * @return The sensor with name \b name or NULL if no such sensor
         * or the sensor is not of type \b T.
         */
        template< class T > rw::core::Ptr< T > findController (const std::string& name) const
        {
            rw::core::Ptr< ControllerModel > sensor = findController (name);
            if (sensor == NULL)
                return NULL;
            return sensor.cast< T > ();
        }

        /**
         * @brief Returns all controllers of a specific type \b T.
         */
        template< class T > std::vector< rw::core::Ptr< T > > findControllers () const
        {
            const std::vector< rw::core::Ptr< ControllerModel > > sensors = _controllers;
            std::vector< rw::core::Ptr< T > > result;
            for (rw::core::Ptr< ControllerModel > f : sensors) {
                rw::core::Ptr< T > res = f.cast< T > ();
                if (res != NULL)
                    result.push_back (res);
            }
            return result;
        }

        /**
         * @brief Returns all controllers in workcell
         * @return List of all controllers
         */
        std::vector< rw::core::Ptr< ControllerModel > > getControllers () const;

        /**
         * @brief Returns all object in the work cell
         *
         * @return All object in work cell
         */
        std::vector< rw::core::Ptr< Object > > getObjects () const { return _objects; }

        /**
         * @brief The object named \b name of the workcell.
         *
         * NULL is returned if there is no such object.
         *
         * @param name [in] The object name
         *
         * @return The object named \b name or NULL if no such object.
         */
        rw::core::Ptr< Object > findObject (const std::string& name) const;

        //! @brief Add device to workcell
        void add (rw::core::Ptr< rw::models::Device > device);
        //! @brief Add object to workcell
        void add (rw::core::Ptr< Object > object);
        //! @brief Add sensormodel to workcell
        void add (rw::core::Ptr< rw::sensor::SensorModel > sensor);
        //! @brief Add controllermodel to workcell
        void add (rw::core::Ptr< ControllerModel > controller);

        //! @brief Remove object from workcell
        void remove (rw::core::Ptr< Object > object);
        //! @brief Remove device from workcell
        void remove (rw::core::Ptr< rw::models::Device > device);
        //! @brief Remove sensormodel from workcell
        void remove (rw::core::Ptr< rw::sensor::SensorModel > sensor);
        //! @brief Remove controllermodel from workcell
        void remove (rw::core::Ptr< ControllerModel > controller);

        /**
         * @brief gets the complete state structure of the workcell.
         * @return the state structure of the workcell.
         */
        rw::core::Ptr< rw::kinematics::StateStructure > getStateStructure () { return _tree; }

#if !defined(SWIG)
        /**
         * @brief Definition of work cell changed listener
         */
        typedef boost::function< void (int) > WorkCellChangedListener;

        /**
         * @brief Definition of even for work cell changed
         */
        typedef rw::core::Event< WorkCellChangedListener, int > WorkCellChangedEvent;

        /**
         * @brief Returns the work cell changed event
         * @return
         */
        WorkCellChangedEvent& workCellChangedEvent () { return _workCellChangedEvent; }
#endif

        /**
         * @brief Properties of this workcell
         */
        rw::core::PropertyMap& getPropertyMap ();

        /**
         * @brief Properties of this workcell
         * @return the property map including the properties of this workcell
         */
        const rw::core::PropertyMap& getPropertyMap () const;

        /**
         * @brief Returns collision setup associated to work cell
         * @return Collision setup
         */
        rw::proximity::CollisionSetup getCollisionSetup ();

        /**
         * @brief Get the scene descriptor.
         * @return the scene descriptor.
         */
        rw::core::Ptr< rw::graphics::SceneDescriptor > getSceneDescriptor ()
        {
            return _sceneDescriptor;
        }

        /**
         * @brief Set the scene descriptor.
         * @param scene [in] the scene descriptor.
         */
        void setSceneDescriptor (rw::core::Ptr< rw::graphics::SceneDescriptor > scene)
        {
            _sceneDescriptor = scene;
        }

        /**
         * @brief Returns the full path and filename of the workcell.
         *
         * If the workcell is loaded from file, then this method returns the
         * full filename. Otherwise it returns an empty string.
         */
        std::string getFilename () const;

        /**
         * @brief Returns the path of where the work cell is located
         *
         * If the workcell is not loaded from file, it returns an empty
         * string
         */
        std::string getFilePath () const;

        /**
         * @brief Returns the filename of the calibration associated to the
         * work cell.
         *
         * Returns an empty string in case no calibration is associated.
         *
         * To load the file use the getFilePath()+getCalibrationFilename()
         * to get the absolute location
         */
        const std::string& getCalibrationFilename () const;

        /**
         * @brief Sets the filename of the calibration file
         *
         * @param calibrationFilename [in] Filename of calibration file with
         * path relative to the work cell path.
         */
        void setCalibrationFilename (const std::string& calibrationFilename);

      private:
        void stateDataAddedListener (const rw::kinematics::StateData* data);
        void stateDataRemovedListener (const rw::kinematics::StateData* data);

      private:
        rw::core::Ptr< rw::kinematics::StateStructure > _tree;
        std::vector< rw::core::Ptr< rw::models::Device > > _devices;
        std::vector< rw::core::Ptr< Object > > _objects;
        std::string _name;
        std::string _filename;
        std::string _calibrationFilename;

        rw::core::PropertyMap* _map;
        WorkCellChangedEvent _workCellChangedEvent;
        std::vector< rw::core::Ptr< rw::sensor::SensorModel > > _sensors;
        std::vector< rw::core::Ptr< ControllerModel > > _controllers;
        rw::core::Ptr< rw::graphics::SceneDescriptor > _sceneDescriptor;

      private:
        WorkCell ();
        WorkCell (const WorkCell&);
        WorkCell& operator= (const WorkCell&);
    };

    /**
     * @brief Streaming operator.
     * @param out [out] output stream to write to.
     * @param workcell [in] the workcell to stream to \b out .
     * @return the output stream \b out .
     */
    std::ostream& operator<< (std::ostream& out, const WorkCell& workcell);

    /*@}*/
}}    // namespace rw::models

#endif    // end include guard
