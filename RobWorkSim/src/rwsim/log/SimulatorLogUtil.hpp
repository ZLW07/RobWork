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

#ifndef RWSIM_LOG_SIMULATORLOGUTIL_HPP_
#define RWSIM_LOG_SIMULATORLOGUTIL_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/DistanceMultiStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

#include <map>
#include <sstream>
#include <vector>

namespace rwsim { namespace log {
    class SimulatorLogScope;
}}    // namespace rwsim::log

/**
 * @file SimulatorLogUtil.hpp
 *
 * \copydoc rwsim::log::SimulatorLogUtil
 */

namespace rwsim { namespace log {

    //! @addtogroup rwsim_log

    //! @{
    /**
     * @brief Utility to make it more convenient to write to a SimulatorLogScope.
     */
    class SimulatorLogUtil
    {
      public:
        //! @brief Constructor.
        SimulatorLogUtil ();

        //! @brief Destructor.
        virtual ~SimulatorLogUtil ();

        /**
         * @brief Set the simulator log to write to.
         * @param log [in] the base log scope.
         */
        virtual void setSimulatorLog (rw::core::Ptr< rwsim::log::SimulatorLogScope > log);

        /**
         * @brief Check if there is a log to write to.
         * @return true if logging is enabled.
         */
        virtual bool doLog () const;

        /**
         * @brief Begin a new discrete simulation step.
         * @param time [in] initial simulation time.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void beginStep (double time, const char* file = "", int line = -1);

        /**
         * @brief End a discrete simulation step.
         * @param time [in] simulation time at end of step.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void endStep (double time, int line = -1);

        /**
         * @brief Add a grouping section.
         * @param name [in] name of the section.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void beginSection (const std::string& name, const char* file = "", int line = -1);

        /**
         * @brief End current grouping section.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void endSection (int line = -1);

        /**
         * @brief Make a subscope.
         * @param name [in] name of scope.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         * @return a new log scope.
         */
        virtual rwsim::log::SimulatorLogScope* makeScope (const std::string& name,
                                                          const char* file = "", int line = -1);

        /**
         * @brief Add numeric values.
         * @param description [in] description of log entry.
         * @param values [in] list of values.
         * @param labels [in] list of labels with same length as \b values.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void addValues (const std::string& description, const std::vector< double >& values,
                                const std::vector< std::string >& labels, const char* file = "",
                                int line = -1);

        /**
         * @brief Log a message.
         * @param description [in] description of log entry.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         * @return a stream to write longer message to.
         */
        virtual std::ostream& log (const std::string& description, const char* file = "",
                                   int line = -1);

        /**
         * @brief Log a message.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         * @return a stream to write longer message to.
         */
        virtual std::ostream& log (const char* file = "", int line = -1);

        /**
         * @brief Add positions for bodies to log.
         * @param description [in] description of log entry.
         * @param positions [in] map of body name to its transform.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void
        addPositions (const std::string& description,
                      const std::map< std::string, rw::math::Transform3D<> >& positions,
                      const char* file = "", int line = -1);

        /**
         * @brief Add velocities for bodies to log.
         * @param description [in] description of log entry.
         * @param velocities [in] map of body name to its velocity.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void
        addVelocities (const std::string& description,
                       const std::map< std::string, rw::math::VelocityScrew6D<> >& velocities,
                       const char* file = "", int line = -1);

        /**
         * @brief Add results from a collision strategy.
         * @param description [in] description of log entry.
         * @param results [in] the results of a collision detection.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void
        addCollisionResults (const std::string& description,
                             const std::vector< rw::proximity::CollisionStrategy::Result >& results,
                             const char* file = "", int line = -1);

        /**
         * @brief Add results from a distance strategy.
         * @param description [in] description of log entry.
         * @param results [in] the results of a distance detection.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void
        addDistanceResults (const std::string& description,
                            const std::vector< rw::proximity::DistanceStrategy::Result >& results,
                            const char* file = "", int line = -1);

        /**
         * @brief Add results from a multi-distance strategy.
         * @param description [in] description of log entry.
         * @param results [in] the results of a multi-distance detection.
         * @param file [in] filename where logging is happening.
         * @param line [in] the line number of the file where logging is happening.
         */
        virtual void addDistanceMultiResults (
            const std::string& description,
            const std::vector< rw::proximity::DistanceMultiStrategy::Result >& results,
            const char* file = "", int line = -1);

      protected:
        //! @brief Current log scope.
        rwsim::log::SimulatorLogScope* _scope;

      private:
        rw::core::Ptr< rwsim::log::SimulatorLogScope > _log;
        std::stringstream _dummyStream;
    };
    //! @}

}}    // namespace rwsim::log

#endif /* RWSIM_LOG_SIMULATORLOGUTIL_HPP_ */
