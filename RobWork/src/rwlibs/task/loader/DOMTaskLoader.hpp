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

#ifndef RWLIBS_TASK_DOMTASKLOADER_HPP
#define RWLIBS_TASK_DOMTASKLOADER_HPP

#include "TaskLoader.hpp"

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
     * @brief Loader for the RobWork task format, using the DOMParser.
     */
    class DOMTaskLoader : public TaskLoader
    {
      public:
        //! @brief Constructor.
        DOMTaskLoader () {}

        //! @brief Destructor.
        ~DOMTaskLoader () {}

        //! @copydoc TaskLoader::load(const std::string&, const std::string&)
        void load (const std::string& filename, const std::string& schemaFileName = "");

        //! @copydoc TaskLoader::load(std::istream&, const std::string&)
        void load (std::istream& instream, const std::string& schemaFileName = "");

        //! @copydoc TaskLoader::getQTask
        rwlibs::task::QTask::Ptr getQTask ();

        //! @copydoc TaskLoader::getCartesianTask
        rwlibs::task::CartesianTask::Ptr getCartesianTask ();

        //! @copydoc TaskLoader::getTask
        rwlibs::task::TaskBase::Ptr getTask ();

        //! @copydoc TaskLoader::clone
        rwlibs::task::TaskLoader::Ptr clone () const;

      private:
        rwlibs::task::TaskBase::Ptr readTask (rw::core::Ptr< rw::core::DOMElem > element);

        void readEntityData (rw::core::Ptr< rw::core::DOMElem > element,
                             rw::core::Ptr< rwlibs::task::Entity > entity);

        rwlibs::task::Action::Ptr readAction (rw::core::Ptr< rw::core::DOMElem > element);

        template< class T >
        typename rwlibs::task::Motion< T >::Ptr
        readMotion (rw::core::Ptr< rw::core::DOMElem > element);

        template< class T >
        typename rwlibs::task::Target< T >::Ptr
        readTarget (rw::core::Ptr< rw::core::DOMElem > element);

        template< class T >
        void readTargets (rw::core::Ptr< rw::core::DOMElem > element,
                          typename rwlibs::task::Task< T >::Ptr task);

        template< class T >
        void readEntities (rw::core::Ptr< rw::core::DOMElem > element,
                           typename rwlibs::task::Task< T >::Ptr task);

        void readAugmentations (rw::core::Ptr< rw::core::DOMElem > element,
                                rwlibs::task::TaskBase::Ptr task);

        template< class T >
        typename rwlibs::task::Task< T >::Ptr
        readTemplateTask (rw::core::Ptr< rw::core::DOMElem > element);

      private:
        typedef std::map< std::string, rwlibs::task::TargetBase::Ptr > TargetMap;
        TargetMap _targetMap;

        rwlibs::task::QTask::Ptr _qTask;
        rwlibs::task::CartesianTask::Ptr _cartTask;
        rwlibs::task::TaskBase::Ptr _task;
    };
    //! @}
}}    // namespace rwlibs::task

#endif    // end include guard
