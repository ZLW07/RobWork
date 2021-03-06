/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_CONTACTS_RENDERCONTACTS_HPP_
#define RWSIM_CONTACTS_RENDERCONTACTS_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/graphics/Render.hpp>

/**
 * @file rwsim/contacts/RenderContacts.hpp
 *
 * \copydoc rwsim::contacts::RenderContacts
 */

namespace rwsim { namespace contacts {
    class Contact;

    //! @addtogroup rwsim_contacts

    //! @{
    /**
     * @brief Render for contacts
     */
    class RenderContacts : public rw::graphics::Render
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr< RenderContacts > Ptr;

        /**
         * @brief Construct render with no initial contacts.
         */
        RenderContacts ();

        /**
         * @brief Constructs a render for a list of contacts.
         *
         * @param contacts [in] the list of contacts to draw.
         */
        RenderContacts (const std::vector< Contact >& contacts);

        /**
         * @brief Destructor
         */
        virtual ~RenderContacts ();

        /**
         * @brief Set which contacts to draw.
         *
         * @param contacts [in] contacts to draw.
         */
        void setContacts (const std::vector< Contact >& contacts);

        /**
         * @brief Get list of contacts that are currently used by the render.
         *
         * @return list of contacts.
         */
        std::vector< Contact > getContacts () const;

        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info,
        //! DrawableNode::DrawType type, double alpha) const
        void draw (const rw::graphics::DrawableNode::RenderInfo& info,
                   rw::graphics::DrawableNode::DrawType type, double alpha) const;

        /**
         * @brief Sets color of contact points.
         *
         * @param r [in] red color component in [0:1]
         * @param g [in] green color component in [0:1]
         * @param b [in] blue color component in [0:1]
         */
        void setColorPoints (float r, float g, float b);

        /**
         * @brief Sets color of normal arrows.
         *
         * @param r [in] red color component in [0:1]
         * @param g [in] green color component in [0:1]
         * @param b [in] blue color component in [0:1]
         */
        void setColorNormal (float r, float g, float b);

        /**
         * @brief Get color of contact points.
         *
         * @return color of contact point as 3D vector for r-, g-, and b-components (in [0:1]).
         */
        rw::math::Vector3D< float > getColorPoint () const;

        /**
         * @brief Get color of contact points.
         *
         * @return color of contact point as 3D vector for r-, g-, and b-components (in [0:1]).
         */
        rw::math::Vector3D< float > getColorNormal () const;

        /**
         * @brief Get the sphere radius.
         * @return the sphere radius (in meters).
         */
        double getSphereRadius () const { return _sphereRadius; }

        /**
         * @brief Get the normal length.
         * @return the normal length (in meters).
         */
        double getNormalLength () const { return _normalLength; }

        /**
         * @brief Check if contact points are shown.
         * @return a pair with the status for the first and second contact point respectively.
         */
        std::pair< bool, bool > showPoints () const { return _showPoints; }

        /**
         * @brief Check if contact normals are shown.
         * @return a pair with the status for the first and second contact normal respectively.
         */
        std::pair< bool, bool > showNormals () const { return _showNormals; }

        /**
         * @brief Set the sphere radius for the contact points.
         * @param radius [in] the new radius (default is 5 mm)
         */
        void setSphereRadius (double radius = 0.005) { _sphereRadius = (radius > 0) ? radius : 0; }

        /**
         * @brief Set the normal length for the contact normals.
         * @param length [in] the new length (default is 5 cm)
         */
        void setNormalLength (double length = 0.05) { _normalLength = (length > 0) ? length : 0; }

        /**
         * @brief Set if the contact points should be shown.
         * @param pointA [in] true if point on first object should be shown.
         * @param pointB [in] true if point on second object should be shown.
         */
        void showPoints (bool pointA, bool pointB)
        {
            _showPoints = std::make_pair (pointA, pointB);
        }

        /**
         * @brief Set if the contact normals should be shown.
         * @param normalA [in] true if first contact normal should be shown.
         * @param normalB [in] true if second contact normal should be shown.
         */
        void showNormals (bool normalA, bool normalB)
        {
            _showNormals = std::make_pair (normalA, normalB);
        }

      private:
        std::vector< Contact > _contacts;
        rw::math::Vector3D< float > _colorPoint, _colorNormal;
        struct GLData;
        const GLData* const _gl;
        double _sphereRadius;
        double _normalLength;
        std::pair< bool, bool > _showPoints;
        std::pair< bool, bool > _showNormals;
    };
    //! @}
}}     // namespace rwsim::contacts
#endif /* RWSIM_CONTACTS_RENDERCONTACTS_HPP_ */
