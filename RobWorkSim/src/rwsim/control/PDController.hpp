#ifndef RWSIM_CONTROL_PDCONTROLLER_HPP_
#define RWSIM_CONTROL_PDCONTROLLER_HPP_

//! @file PDController.hpp

#include <rw/core/Ptr.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>

namespace rwsim { namespace dynamics {
    class DynamicDevice;
}}    // namespace rwsim::dynamics

namespace rwsim { namespace control {
    //! @addtogroup rwsim_control
    //! @{

    //! @brief struct for holding PD parameters
    struct PDParam
    {
        PDParam () : P (10), D (0.003) {}
        PDParam (double p, double d) : P (p), D (d){};
        double P;    //! the proportional parameter
        double D;    //! the derivative parameter
    };

    /**
     * @brief a JointController that use a PD loop on each joint
     * to control the velocity such that the position target is
     * reached.
     */
    class PDController : public rwlibs::control::JointController,
                         public rwlibs::simulation::SimulatedController
    {
      public:
        typedef rw::core::Ptr< PDController > Ptr;
        /**
         * @brief constructor
         * @param name
         * @param rdev [in] device that is to be controlled
         * @cond
         * @param state [in] target state
         * @endcond
         * @param cmode [in] the control mode used
         * @param pdparams [in] list of pd parameters. must be same length as number of joints.
         * @param dt [in] the sampletime (time between samples in seconds) used in the control
         * loop, this should be larger than the expected update sample time.
         */
        PDController (const std::string& name, rw::core::Ptr< rwsim::dynamics::DynamicDevice > rdev,
                      ControlMode cmode, const std::vector< PDParam >& pdparams, double dt);

        /**
         * @brief constructor
         * @param name
         * @param rdev [in] device that is to be controlled
         * @param cmode [in] the control mode used
         * @param pdparam [in] pd parameter - used for all joints
         * @param dt [in] the sampletime (time between samples in seconds) used in the control
         * loop, this should be larger than the expected update sample time.
         */
        PDController (const std::string& name, rw::core::Ptr< rwsim::dynamics::DynamicDevice > rdev,
                      ControlMode cmode, const PDParam& pdparam, double dt);

        /**
         * @brief destructor
         */
        virtual ~PDController (){};

        /**
         * @brief the PD parameters
         * @return list of PD parameters
         */
        std::vector< PDParam > getParameters ();

        /**
         * @brief set the PD parameters
         * @param params [in] list of parameters. must be same length as DOF
         * of controlling device
         */
        void setParameters (const std::vector< PDParam >& params);

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
            return this;
        }

      private:
        PDController ();

      private:
        rw::core::Ptr< rwsim::dynamics::DynamicDevice > _ddev;
        rw::math::Q _maxVel;
        rw::math::Q _lastError, _currentError, _target, _currentQ, _currentVel;
        rw::math::Q _targetVel;
        std::vector< PDParam > _pdparams;
        ControlMode _mode;
        double _stime, _accTime;    // sample time
        rw::math::Q _P, _D;
        bool _enabled;
    };

    typedef rw::core::Ptr< PDController > PDControllerPtr;
    //! @}
}}    // namespace rwsim::control

#endif /*PDController_HPP_*/
