/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RWSIMLIBS_ODE_ODELOGUTIL_HPP_
#define RWSIMLIBS_ODE_ODELOGUTIL_HPP_

#include <rwsim/log/SimulatorLogUtil.hpp>

namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics
namespace rwsim { namespace log {
    class SimulatorLogScope;
}}    // namespace rwsim::log

/**
 * @file ODELogUtil.hpp
 *
 * \copydoc rwsim::simulator::ODELogUtil
 */

namespace rwsim { namespace simulator {

    // Forward declarations
    class ODEBody;

    //! @addtogroup rwsim_simulator

    //! @{
    /**
     * @brief Utility to write to SimulatorLog.
     */
    class ODELogUtil : public rwsim::log::SimulatorLogUtil
    {
      public:
        //! @brief Constructor.
        ODELogUtil ();

        //! @brief Destructor.
        virtual ~ODELogUtil ();

        //! @copydoc addPositions(const std::string&, const
        //! std::map<std::string,rw::math::Transform3D<> >&, const char*, int)
        void addPositions (const std::string& description,
                           const std::map< std::string, rw::math::Transform3D<> >& positions,
                           const char* file = "", int line = -1);

        /**
         * @brief Add positions of bodies to log.
         * @param description [in] description of log entry.
         * @param bodies [in] vector of bodies to add position for.
         * @param state [in] the state with the positions.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        void addPositions (const std::string& description, const std::vector< ODEBody* >& bodies,
                           const rw::kinematics::State& state, const char* file = "",
                           int line = -1);

        //! @copydoc addVelocities(const std::string&, const
        //! std::map<std::string,rw::math::VelocityScrew6D<> >&, const char*, int)
        void addVelocities (const std::string& description,
                            const std::map< std::string, rw::math::VelocityScrew6D<> >& velocities,
                            const char* file = "", int line = -1);

        /**
         * @brief Add velocities of bodies to log.
         * @param description [in] description of log entry.
         * @param bodies [in] vector of bodies to add velocities for.
         * @param state [in] the state with the velocities.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        void addVelocities (const std::string& description, const std::vector< ODEBody* >& bodies,
                            const rw::kinematics::State& state, const char* file = "",
                            int line = -1);
    };
    //! @}

}}    // namespace rwsim::simulator

#endif /* RWSIMLIBS_ODE_ODELOGUTIL_HPP_ */
