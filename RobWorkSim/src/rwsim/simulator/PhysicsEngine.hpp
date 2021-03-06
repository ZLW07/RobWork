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

#ifndef RWSIM_SIMULATOR_PHYSICSENGINE_HPP_
#define RWSIM_SIMULATOR_PHYSICSENGINE_HPP_

#include <rw/core/ExtensionPoint.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>

// Forward declarations
namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics
namespace rwlibs { namespace simulation {
    class SimulatedController;
}}    // namespace rwlibs::simulation
namespace rwsim { namespace contacts {
    class ContactDetector;
}}    // namespace rwsim::contacts
namespace rwsim { namespace dynamics {
    class Body;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class DynamicDevice;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class DynamicWorkCell;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace log {
    class SimulatorLogScope;
}}    // namespace rwsim::log

namespace rwsim { namespace simulator {
    //! @addtogroup rwsim_simulator
    //! @{

    /**
     * @brief A general physics engine interface for simulating dynamics
     * of objects and robot devices.
     *
     * The general step looks like this:;
     *
     * Foreach controller=_controllers
     *  controller->update(dt,state);
     *
     * collision detection (contact detection)
     * constraint solving
     * update device/body positions (integration)
     *
     * Foreach sensor=_sensors
     *  sensor->update(dt,state)
     *
     */
    class PhysicsEngine
    {
      public:
        //! smart pointer type of PhysicsEngine
        typedef rw::core::Ptr< PhysicsEngine > Ptr;

        /**
         * @brief destructor
         */
        virtual ~PhysicsEngine (){};

        /**
         * @brief adds dynamic workcell
         */
        virtual void load (rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc) = 0;

        /**
         * @brief Change the contact detector used by the engine.
         * @param detector [in] the contact detector to use (NULL for default contact detection)
         * @return true if engine supports using a ContactDetector and the detector was set
         * successfully.
         */
        virtual bool
        setContactDetector (rw::core::Ptr< rwsim::contacts::ContactDetector > detector) = 0;

        /**
         * @brief Performs a step and updates the state
         */
        virtual void step (double dt, rw::kinematics::State& state) = 0;

        /**
         * @brief reset velocity and acceleration of all bodies to 0. And sets the position of all
         * bodies to that described in state
         */
        virtual void resetScene (rw::kinematics::State& state) = 0;

        /**
         * @brief initialize simulator physics with state
         */
        virtual void initPhysics (rw::kinematics::State& state) = 0;

        /**
         * @brief cleans up the allocated storage fo bullet physics
         */
        virtual void exitPhysics () = 0;

        /**
         * @brief gets the the current simulated time
         */
        virtual double getTime () = 0;

        /**
         * Enables or disables a body. A body is automatically enabled if contacts or new
         * interactions with the Body arrise
         * @param body [in] the body to enable/disable
         * @param enabled [in]
         */
        virtual void setEnabled (rw::core::Ptr< rwsim::dynamics::Body > body, bool enabled) = 0;

        /**
         * @brief disables the dynamics of a body.
         * @param body
         * @param enabled
         */
        virtual void setDynamicsEnabled (rw::core::Ptr< rwsim::dynamics::Body > body,
                                         bool enabled) = 0;

        /**
         * @brief create a debug render for the specific implementation
         * @return NULL if no render is available else a valid render
         */
        virtual drawable::SimulatorDebugRender::Ptr createDebugRender () = 0;

        /**
         * @brief properties of the physics engine
         */
        virtual rw::core::PropertyMap& getPropertyMap () = 0;

        /**
         * @brief should be called when properties have been changed and one wants the physics
         * engine to reflect the new properties.
         */
        virtual void emitPropertyChanged () = 0;

        /**
         * @brief add a simulated controller to this simulator
         */
        virtual void
        addController (rw::core::Ptr< rwlibs::simulation::SimulatedController > controller) = 0;

        /**
         * @brief removes a simulated controller from this simulator
         * @param controller
         */
        virtual void
        removeController (rw::core::Ptr< rwlibs::simulation::SimulatedController > controller) = 0;

        /**
         * @brief add a body to the physics engine
         * @param body [in] body to add
         * @param state [in] current state
         */
        virtual void addBody (rw::core::Ptr< rwsim::dynamics::Body > body,
                              rw::kinematics::State& state) = 0;

        /**
         * @brief add a dynamic device to the physics engine
         * @param device [in] device to add
         * @param state [in] current state
         */
        virtual void addDevice (rw::core::Ptr< rwsim::dynamics::DynamicDevice > device,
                                rw::kinematics::State& state) = 0;

        /**
         * @brief add a simulated sensor to this simulator
         */
        virtual void addSensor (rwlibs::simulation::SimulatedSensor::Ptr sensor,
                                rw::kinematics::State& state) = 0;

        /**
         * @brief add a simulated sensor to this simulator
         */
        virtual void removeSensor (rwlibs::simulation::SimulatedSensor::Ptr sensor) = 0;

        /**
         * @brief creates a 6dof dynamic constraint between the two bodies \b b1 and \b b2
         * @param b1
         * @param b2
         */
        virtual void attach (rw::core::Ptr< rwsim::dynamics::Body > b1,
                             rw::core::Ptr< rwsim::dynamics::Body > b2) = 0;

        /**
         * @brief removes the 6dof constraint between bodies \b b1 and \b b2 if there is any
         * @param b1
         * @param b2
         */
        virtual void detach (rw::core::Ptr< rwsim::dynamics::Body > b1,
                             rw::core::Ptr< rwsim::dynamics::Body > b2) = 0;

        // this should be a flexible version of the above
        // virtual void addConstraint( );

        /**
         * @brief get the list of simulated sensors
         * @return
         */
        virtual std::vector< rwlibs::simulation::SimulatedSensor::Ptr > getSensors () = 0;

        /**
         * @brief Store internal info during simulation.
         *
         * This can be used for debugging, statistics, visualization of internal subresults,
         * timing, benchmark and similar.
         *
         * @param log [in] a pointer to the log structure to store to.
         */
        virtual void setSimulatorLog (rw::core::Ptr< rwsim::log::SimulatorLogScope > log) {}

        //! @brief Each engine implements a dispatcher that creates instances of the engine.
        class Dispatcher
        {
          public:
            //! @brief Constructor.
            Dispatcher (){};

            //! @brief Destructor.
            virtual ~Dispatcher (){};

            /**
             * @brief Create a physics engine.
             * @return the physics engine.
             */
            virtual PhysicsEngine::Ptr makePhysicsEngine () const = 0;
        };

        /**
         * @addtogroup extensionpoints
         * @extensionpoint{rwsim::simulator::PhysicsEngine::Factory,rwsim::simulator::PhysicsEngine::Dispatcher,rwsim.simulator.PhysicsEngine}
         */

        /**
         * @brief a factory for PhysicsEngine. This factory defines an extension point for
         * PhysicsEngines.
         *
         * Required properties on an extension is:
         *  - name: engineID value:string desc:identifies the engine to the user
         */
        class Factory : public rw::core::ExtensionPoint< Dispatcher >
        {
          private:
            //! constructor
            Factory ();

          public:
            /**
             * @brief test if the factory has a specific physics engine
             * @return true if engine with \b engineID is available
             */
            static bool hasEngineID (const std::string& engineID);

            /**
             * @brief get ids of all engines that are available
             * @return list of string IDs
             */
            static std::vector< std::string > getEngineIDs ();

            /**
             * @brief Create a physics engine using a dynamic workcell
             * @param dwc [in] the dynamic workcell
             * @return physics engine
             */
            static PhysicsEngine::Ptr
            makePhysicsEngine (rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc);

            /**
             * @brief construct a physics engine with \b engineID and
             * @param engineID [in] ID of engine
             * @param dwc [in] dynamic workcell
             * @return physics engine
             */
            static PhysicsEngine::Ptr
            makePhysicsEngine (const std::string& engineID,
                               rw::core::Ptr< rwsim::dynamics::DynamicWorkCell > dwc);

            /**
             * @brief construct a physics engine with \b engineID
             * @param engineID [in] ID of engine
             * @return physics engine
             */
            static PhysicsEngine::Ptr makePhysicsEngine (const std::string& engineID);
        };
    };

    //! @}
}}     // namespace rwsim::simulator
#endif /*Simulator_HPP_*/
