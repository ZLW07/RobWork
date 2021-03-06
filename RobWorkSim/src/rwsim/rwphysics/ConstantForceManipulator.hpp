#ifndef RWSIM_SIMULATOR_CONSTANTFORCEMANIPULATOR_HPP_
#define RWSIM_SIMULATOR_CONSTANTFORCEMANIPULATOR_HPP_

#include "BodyController.hpp"

#include <rw/math/Vector3D.hpp>

#include <vector>

namespace rwsim { namespace simulator {
    class RWBody;

    /**
     * @brief class for adding a constant force to any body it controls
     */
    class ConstantForceManipulator : public BodyController
    {
      public:
        /**
         * @brief constructor
         * @param force
         * @param bodies
         */
        ConstantForceManipulator (const rw::math::Vector3D<>& force,
                                  std::vector< RWBody* >& bodies);

        /**
         * @brief destructor
         */
        virtual ~ConstantForceManipulator (){};

        /**
         * @copydoc BodyController::addForces
         */
        void addForces (rw::kinematics::State& state, double h);

        /**
         * @copydoc BodyController::reset
         */
        void reset (rw::kinematics::State& state) {}

        /**
         * @brief returns the list of bodies that are controlled
         * @return
         */
        std::vector< RWBody* >& getBodies () { return _bodies; }

        /**
         * @brief sets the force
         */
        void setForce (const rw::math::Vector3D<>& f) { _force = f; }

        /**
         * @brief gets the force
         */
        const rw::math::Vector3D<>& getForce () { return _force; }

      private:
        rw::math::Vector3D<> _force;
        std::vector< RWBody* > _bodies;
    };
}}     // namespace rwsim::simulator
#endif /*CONSTANTFORCEMANIPULATOR_HPP_*/
