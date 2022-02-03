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

#ifndef RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_ANALYTICGEOMETRYPLUGIN_HPP_
#define RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_ANALYTICGEOMETRYPLUGIN_HPP_

/**
 * @file AnalyticGeometryPlugin.hpp
 *
 * \copydoc rwlibs::geometry::AnalyticGeometryPlugin
 */

#include <rw/core/Extension.hpp>
#include <rw/core/Plugin.hpp>
#include <rw/core/Ptr.hpp>

namespace rwlibs { namespace geometry {

    //! @addtogroup rwlibs_geometry

    //! @{
    /**
     * @brief Plugin adding new analytic geometry types to the
     * rw.loaders.GeometryFactory extension point.
     *
     * The following types are added with the use of this plugin:
     *  - QuadraticTestObjectA - rwlibs::geometry::QuadraticTestObjects::objectA()
     *  - QuadraticTestObjectB - rwlibs::geometry::QuadraticTestObjects::objectB()
     */
    class AnalyticGeometryPlugin : public rw::core::Plugin
    {
      public:
        //! @brief Construct new plugin
        AnalyticGeometryPlugin ();

        //! @brief Destructor
        virtual ~AnalyticGeometryPlugin ();

        //! @copydoc rw::core::Plugin::getExtensionDescriptors
        std::vector< rw::core::Extension::Descriptor > getExtensionDescriptors ();

        //! @copydoc rw::core::Plugin::makeExtension
        rw::core::Ptr< rw::core::Extension > makeExtension (const std::string& id);
    };
    //! @}

}}    // namespace rwlibs::geometry

#endif /* RWLIBS_GEOMETRY_ANALYTIC_GEOMETRY_ANALYTICGEOMETRYPLUGIN_HPP_ */
