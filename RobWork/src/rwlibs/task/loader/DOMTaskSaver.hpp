/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_TASK_DOMTASKSAVER_HPP
#define RWLIBS_TASK_DOMTASKSAVER_HPP

#include "TaskSaver.hpp"

#include <rw/core/Ptr.hpp>
#include <rwlibs/task/Entity.hpp>
#include <rwlibs/task/Task.hpp>

#include <string>

namespace rw { namespace core {
    class DOMElem;
}}    // namespace rw::core

namespace rwlibs { namespace task {
    //! @addtogroup task

    //! @{
    /**
     * @brief Saver for the RobWork task format, using the DOMParser.
     */
    class DOMTaskSaver : public TaskSaver
    {
      public:
        //! @brief Constructor.
        DOMTaskSaver () {}

        //! @brief Destructor.
        virtual ~DOMTaskSaver () {}

        //! @copydoc TaskSaver::save(rwlibs::task::QTask::Ptr, std::ostream&)
        bool save (rwlibs::task::QTask::Ptr task, std::ostream& outstream);

        //! @copydoc TaskSaver::save(rwlibs::task::CartesianTask::Ptr, std::ostream&)
        bool save (rwlibs::task::CartesianTask::Ptr task, std::ostream& outstream);

        //! @copydoc TaskSaver::save(rwlibs::task::QTask::Ptr, const std::string&)
        bool save (rwlibs::task::QTask::Ptr task, const std::string& filename);

        //! @copydoc TaskSaver::save(rwlibs::task::CartesianTask::Ptr, const std::string&)
        bool save (rwlibs::task::CartesianTask::Ptr task, const std::string& filename);

        /**
         * @brief Utility class which initializes local static variables.
         *
         * If the DOMTaskSaver is used outside main (as a part of global
         * initialization/destruction), the Initializer should be used explicitly to control the
         * static initialization/destruction order.
         *
         * Notice that the Initializer is automatically defined as a global variable, hence it
         * should not be necessary to specify the initializer explicitly if DOMTaskSaver is to be
         * used in local static initialization/destruction.
         */
        class Initializer
        {
          public:
            //! @brief Initializes when constructed.
            Initializer ();
        };

      private:
        void writeTask (TaskBase::Ptr task, rw::core::Ptr< rw::core::DOMElem > parent);

        template< class T >
        void saveImpl (typename Task< T >::Ptr task, rw::core::Ptr< rw::core::DOMElem > parrent);

        void writeEntityInfo (Entity::Ptr entity, rw::core::Ptr< rw::core::DOMElem > parent);

        template< class T >
        void writeTargets (typename Task< T >::Ptr task, rw::core::Ptr< rw::core::DOMElem > parent);

        template< class T >
        void writeMotion (typename Motion< T >::Ptr motion,
                          rw::core::Ptr< rw::core::DOMElem > element);

        template< class T >
        void writeEntities (typename Task< T >::Ptr task,
                            rw::core::Ptr< rw::core::DOMElem > parent);

        void writeAction (Action::Ptr action, rw::core::Ptr< rw::core::DOMElem > parent);

        template< class T >
        void writeTaskImpl (typename Task< T >::Ptr task,
                            rw::core::Ptr< rw::core::DOMElem > element);

        std::map< rwlibs::task::TargetBase::Ptr, std::string > _targetMap;

      private:
        static const Initializer initializer;
    };
    //! @}
}}    // namespace rwlibs::task

#endif    // end include guard
