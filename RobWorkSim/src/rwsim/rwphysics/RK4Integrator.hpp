/*
 * RK4Integrator.hpp
 *
 *  Created on: 22-10-2008
 *      Author: jimali
 */

#ifndef RK4INTEGRATOR_HPP_
#define RK4INTEGRATOR_HPP_

#include "BodyIntegrator.hpp"

namespace rwsim { namespace dynamics {
    class RigidBody;
}}    // namespace rwsim::dynamics

namespace rwsim { namespace simulator {

    class RK4Integrator : public BodyIntegrator
    {
      public:
        RK4Integrator (dynamics::RigidBody* body);

        virtual ~RK4Integrator (){};

        void updatePosition (double h, rw::kinematics::State& state);

        void updateVelocity (double h, rw::kinematics::State& state);

      private:
        dynamics::RigidBody* _body;
    };

}}    // namespace rwsim::simulator

#endif /* RK4INTEGRATOR_HPP_ */
