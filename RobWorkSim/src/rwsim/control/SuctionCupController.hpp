/*
 * SuctionCupController.hpp
 *
 *  Created on: 15/01/2011
 *      Author: jimali
 */

#ifndef SUCTIONCUPCONTROLLER_HPP_
#define SUCTIONCUPCONTROLLER_HPP_

#include <rw/math/Transform3D.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim { namespace dynamics {
    class Body;
}}    // namespace rwsim::dynamics
namespace rwsim { namespace dynamics {
    class SuctionCup;
}}    // namespace rwsim::dynamics

namespace rwsim { namespace control {

    /**
     * @brief controlls the forces and the internal states of the bodies that
     * makes a suction cup. The controller depends on the contacts of the elastic
     * suctioncup geometry with other bodies and the class gets this feedback from
     * a SuctionCupSensor
     *
     */

    class SuctionCupController : public rwlibs::simulation::SimulatedController
    {
      public:
        struct Spring
        {
            Spring (double l, double e, double damp) :
                length (l), elasticity (e), dampeningFactor (damp)
            {}
            double length, elasticity, dampeningFactor;
        };

      public:
        typedef rw::core::Ptr< SuctionCupController > Ptr;

        SuctionCupController (const std::string& name,
                              rw::core::Ptr< rwsim::dynamics::SuctionCup > dev);

        virtual ~SuctionCupController ();

        std::string getControllerName () { return _name; }

        //! @copydoc rwlibs::simulation::SimulatedController::update
        void update (const rwlibs::simulation::Simulator::UpdateInfo& info,
                     rw::kinematics::State& state);

        //! @copydoc rwlibs::simulation::SimulatedController::reset
        void reset (const rw::kinematics::State& state);

        rwlibs::control::Controller* getController ();

        void setEnabled (bool enabled) { _enabled = enabled; }

        bool isEnabled () const { return _enabled; }

      private:
        rw::core::Ptr< rwsim::dynamics::SuctionCup > _dev;

        // rwsim::sensor::SuctionCupSensor::Ptr _sensor;
        std::vector< Spring > _springs;
        std::vector< std::pair< rwsim::dynamics::Body*, rwsim::dynamics::Body* > > _bodyPairs;
        std::vector< rw::math::Transform3D<> > _bodyTransforms;
        bool _enabled;

        std::string _name;
    };

}}     // namespace rwsim::control
#endif /* SUCTIONCUPCONTROLLER_HPP_ */
