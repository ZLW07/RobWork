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

#ifndef RWSIM_SIMULATOR_ODESUCTIONCUPDEVICE_HPP_
#define RWSIM_SIMULATOR_ODESUCTIONCUPDEVICE_HPP_

#include <rw/geometry/TriMesh.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rwsimlibs/ode/ODEBody.hpp>
#include <rwsimlibs/ode/ODEDevice.hpp>

#include <ode/ode.h>

namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics
namespace rw { namespace proximity {
    class ProximityModel;
}}    // namespace rw::proximity
namespace rwlibs { namespace proximitystrategies {
    class ProximityStrategyPQP;
}}    // namespace rwlibs::proximitystrategies
namespace rwsim { namespace dynamics {
    class Body;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class SuctionCup;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace sensor {
    class BodyContactSensor;
}}    // namespace rwsim::sensor

namespace rwsim { namespace simulator {
    class ODESimulator;

    /**
     * @brief interface for classes (ODEDevices) that control a set of ode bodies
     * that map to a RWSim dynamic device type.
     */
    class ODESuctionCupDevice : public ODEDevice
    {
      public:
        /**
         * @brief Constructor.
         * @param base [in] the base body of the device.
         * @param dev [in] the RobWork SuctionCup device.
         * @param odesim [in] the simulator.
         * @param state [in] the state.
         */
        ODESuctionCupDevice (ODEBody* base, rwsim::dynamics::SuctionCup* dev, ODESimulator* odesim,
                             rw::kinematics::State& state);

        /**
         * @brief Initialize the device.
         * @param base [in] the base body of the device.
         * @param dev [in] the RobWork SuctionCup device.
         * @param odesim [in] the simulator.
         * @param state [in] the state.
         */
        void init (ODEBody* base, rwsim::dynamics::SuctionCup* dev, ODESimulator* odesim,
                   rw::kinematics::State& state);

        //! @brief Destructor.
        virtual ~ODESuctionCupDevice ();

        /**
         * @brief resets the ODE device to the state values of the RWSim device.
         * @param state
         */
        virtual void reset (rw::kinematics::State& state);

        /**
         * @brief the update call is made prior to the simulation step. In this
         * method states of the ODE bodies and joints (forces, velocities, eg)
         * can be updated from the state of the RWSim device.
         * @param dt
         * @param state [out] ODEDevice state values are copied to \b state
         */
        virtual void update (const rwlibs::simulation::Simulator::UpdateInfo& dt,
                             rw::kinematics::State& state);

        /**
         * @brief The post update is called after a simulation step has
         * been performed. Here the modified states (force,velocity,position)
         * of the ODE device is written back to the \b state object.
         * @param state
         */
        virtual void postUpdate (rw::kinematics::State& state);

        // static ODESuctionCupDevice* makeSuctionCup(rwsim::dynamics::SuctionCup* scup,
        // ODESimulator *sim, rw::kinematics::State &state);

        /**
         * @brief Get spiked mesh.
         * @return the mesh.
         */
        rw::geometry::TriMesh::Ptr getSpikedMesh () { return _spikedCupMesh; }

        /**
         * @brief Get the end body.
         * @return the end body.
         */
        ODEBody* getEndBody () { return _odeEnd; }

        //! @copydoc ODEDevice::getBodies
        std::vector< ODEBody* > getBodies () { return _ode_bodies; };

      private:
        void updateNoRollBack (const rwlibs::simulation::Simulator::UpdateInfo& dt,
                               rw::kinematics::State& state);

      private:
        // rwsim::dynamics::SuctionCup::Ptr _dev;

        // rwsim::sensor::SuctionCupSensor::Ptr _sensor;
        // std::vector<Spring> _springs;
        // std::vector<std::pair<rwsim::dynamics::Body*,rwsim::dynamics::Body*> > _bodyPairs;
        // std::vector<rw::math::Transform3D<> > _bodyTransforms;
        std::string _name;
        rw::core::Ptr< rwsim::dynamics::SuctionCup > _dev;
        rw::core::Ptr< rwsim::sensor::BodyContactSensor > _sensor;
        rw::core::Ptr< rwsim::dynamics::Body > _tcp;
        double _v;

        bool _isInContact;
        rw::core::Ptr< rwsim::dynamics::Body > _object;

        rw::geometry::TriMesh::Ptr _spikedCupMesh;
        rw::geometry::Geometry::Ptr _spikedCup;
        rw::core::Ptr< rw::proximity::ProximityModel > _spikedCupModel;

        rw::core::Ptr< rwlibs::proximitystrategies::ProximityStrategyPQP > _narrowStrategy;
        rw::proximity::ProximityStrategyData _pdata;
        ODESimulator* _odesim;
        dWorldID _worldId;
        dBodyID _bTmp1, _bTmp2;
        dJointGroupID _contactGroupId;
        dJointID _slider, _hinge1, _hinge2, _fjoint;
        dJointFeedback _feedback;
        ODEBody *_odeBase, *_odeEnd;
        double _lastX, _lastAng;
        std::vector< dContact > _contacts;
        std::vector< ODEBody* > _ode_bodies;
    };
}}     // namespace rwsim::simulator
#endif /* ODEDEVICE_HPP_ */
