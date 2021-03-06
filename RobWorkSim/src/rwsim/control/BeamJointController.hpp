#ifndef RWSIM_CONTROL_BeamJointController_HPP_
#define RWSIM_CONTROL_BeamJointController_HPP_

//! @file BeamJointController.hpp

#include <rw/core/Ptr.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim { namespace dynamics {
    class RigidDevice;
}}    // namespace rwsim::dynamics

namespace rwsim { namespace control {
    //! @addtogroup rwsim_control
    //! @{

    /**
     * @brief The beamjoint controller controls a joint device composed of coupled beam joints.
     * The coupling of beamjoints is somewhat special since its not an configuration space
     * coupling (position) but rather a coupling of the forces acting on the joints.
     *
     * This controller continuesly updates the torques acting on the joints by considering the
     * current deflection of the joint.
     *
     */
    class BeamJointController : public rwlibs::control::JointController,
                                public rwlibs::simulation::SimulatedController
    {
      public:
        /**
         * @brief constructor
         * @param name
         * @param rdev [in] device that is to be controlled
         * @cond
         * @param state [in] target state
         * @param pdparams [in] list of pd parameters. must be same length as number of joints.
         * @endcond
         * @param cmode [in] the control mode used
         * @param dt [in] the sampletime (time between samples in seconds) used in the control
         * loop, this should be larger than the expected update sample time.
         */
        BeamJointController (const std::string& name, dynamics::RigidDevice* rdev,
                             ControlMode cmode, double dt);

        /**
         * @brief destructor
         */
        virtual ~BeamJointController (){};

        /**
         * @brief the time between samples
         * @return the sample time in seconds
         */
        double getSampleTime ();

        /**
         * @brief set the time between samples in seconds
         * @param stime [in] sample time
         */
        void setSampleTime (double stime);

        //! @copydoc rwlibs::simulation::SimulatedController::update
        void update (const rwlibs::simulation::Simulator::UpdateInfo& info,
                     rw::kinematics::State& state);

        //! @copydoc rwlibs::simulation::SimulatedController::reset
        void reset (const rw::kinematics::State& state);

        //! @copydoc rwlibs::simulation::SimulatedController::getControllerName
        Controller* getController () { return this; };

        std::string getControllerName () { return getName (); };

        ////// inherited from JointController

        /**
         * @copydoc rwlibs::control::JointController::getControlModes
         *
         * This controller supports both position and velocity control.
         */
        unsigned int getControlModes () { return _mode; }

        //! @copydoc rwlibs::control::JointController::setControlMode
        void setControlMode (ControlMode mode);

        //! @copydoc rwlibs::control::JointController::setTargetPos
        void setTargetPos (const rw::math::Q& target);

        //! @copydoc rwlibs::control::JointController::setTargetVel
        void setTargetVel (const rw::math::Q& vals);

        //! @copydoc rwlibs::control::JointController::setTargetAcc
        void setTargetAcc (const rw::math::Q& vals);

        //! @copydoc rwlibs::control::JointController::getQ
        rw::math::Q getQ () { return _currentQ; }

        //! @copydoc rwlibs::control::JointController::getQd
        rw::math::Q getQd () { return _currentVel; }

        void setEnabled (bool enabled) { _enabled = enabled; };

        bool isEnabled () const { return _enabled; };

        rwlibs::control::Controller::Ptr
        getControllerHandle (rwlibs::simulation::Simulator::Ptr sim)
        {
            return NULL;
        }

      private:
        BeamJointController ();

      private:
        // std::vector<rw::models::BeamJoint*> _beamJoints;

        dynamics::RigidDevice* _ddev;
        rw::math::Q _maxVel;
        rw::math::Q _lastError, _target, _currentQ, _currentVel;
        rw::math::Q _targetVel;
        ControlMode _mode;
        double _stime, _accTime;    // sample time
        rw::math::Q _P, _D;
        bool _enabled;
    };

    typedef rw::core::Ptr< BeamJointController > BeamJointControllerPtr;
    //! @}
}}    // namespace rwsim::control

#endif /*BeamJointController_HPP_*/
