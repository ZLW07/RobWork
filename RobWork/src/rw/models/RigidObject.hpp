/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MODELS_RIGIDOBJECT_HPP_
#define RW_MODELS_RIGIDOBJECT_HPP_

#if !defined(SWIG)
#include "Object.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/geometry/Geometry.hpp>

#include <vector>
#endif 

namespace rw { namespace kinematics {
    class Frame;
}}    // namespace rw::kinematics
namespace rw { namespace kinematics {
    class State;
}}    // namespace rw::kinematics

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief the RigidObject defines a physical object in the workcell that is rigid in the sence
     * that the geometry does not change. The rigid object also have basic properties such as
     * Inertia and mass. These are default 1.0 kg and inertia of solid sphere with mass 1.0kg and
     * radius of 10cm. The center of mass defaults to origin of the base frame.
     */
    class RigidObject : public Object
    {
      public:
        //! smart pointer
        typedef rw::core::Ptr< RigidObject > Ptr;
        //! const smart pointer
        typedef rw::core::Ptr< const RigidObject > CPtr;

        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         */
        RigidObject (rw::kinematics::Frame* baseframe);

        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         * @param geom [in] the Geometry Forming the object
         */
        RigidObject (rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom);

        /**
         * @brief constructor
         * @param baseFrame [in] base frame of the object
         * @param geom [in] the Geometry Forming the object
         */
        RigidObject (rw::kinematics::Frame* baseframe,
                     std::vector< rw::geometry::Geometry::Ptr > geom);

        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         */
        RigidObject (std::vector< rw::kinematics::Frame* > frames);

        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         * @param geom [in] the Geometry Forming the object
         */
        RigidObject (std::vector< rw::kinematics::Frame* > frames,
                     rw::geometry::Geometry::Ptr geom);

        /**
         * @brief constructor
         * @param frames [in] first frame is base frame of the object
         * @param geom [in] a list of geometries to form the object
         */
        RigidObject (std::vector< rw::kinematics::Frame* > frames,
                     std::vector< rw::geometry::Geometry::Ptr > geom);

        //! destructor
        virtual ~RigidObject ();

        /**
         * @brief add collision geometry from this object
         * @param geom [in] the geometry to add
         */
        void addGeometry (rw::geometry::Geometry::Ptr geom);

        /**
         *  @brief remove collision geometry from this object
         *  @param geom [in] the geometry to remove
         */
        void removeGeometry (rw::geometry::Geometry::Ptr geom);

        /**
         * @brief add visualization model to this object
         * @param model [in] the model to be added
         */
        void addModel (rw::geometry::Model3D::Ptr model);

        /**
         * @brief remove visualization model to this rigid object
         * @param model [in] the model to be removed
         */
        void removeModel (rw::geometry::Model3D::Ptr model);

        /**
         * @brief returns the mass of this RigidObject
         * @return mass of the Object
         */
        double getMass () const { return _mass; };

        /**
         * @brief set mass of this RigidObject
         * @param mass [in] the mass of this object
         */
        void setMass (double mass) { _mass = mass; };

        /**
         * @brief get the inertia matrix of this rigid body seen in the base frame
         * @return IntertiaMatrix
         */
        rw::math::InertiaMatrix<> getInertia () const { return _Ibody; };

        /**
         *  @brief set inertia of this rigid object
         * @param initia [in] the inertia of this object
         */
        void setInertia (const rw::math::InertiaMatrix<>& inertia) { _Ibody = inertia; }

        /**
         * @brief get the center of mass of this rigid body seen in the base frame
         * @return the center of mass 3D coordinate
         */
        rw::math::Vector3D<double> getCOM () const { return _com; };
        //! @brief set the center of mass of this rigid body seen in the base frame
        void setCOM (const rw::math::Vector3D<double>& com) { _com = com; }

        /**
         * @brief approximates inertia based on geometry, mass and center of mass properties
         */
        void approximateInertia ();

        /**
         * @brief approximates inertia and center of mass based on geometry and mass properties
         */
        void approximateInertiaCOM ();

        /**
         * @brief get geometry of this rigid object
         * @return a list of all Geometries
         */
        const std::vector< rw::geometry::Geometry::Ptr >& getGeometry () const;

        /**
         * @brief get visualization models for this rigid object
         * @return a list of all models
         */
        const std::vector< rw::geometry::Model3D::Ptr >& getModels () const;

        //! @copydoc Object::getMass
        double getMass (rw::kinematics::State& state) const { return getMass (); };

        //! @copydoc Object::getInertia
        rw::math::InertiaMatrix<> getInertia (rw::kinematics::State& state) const
        {
            return getInertia ();
        }

        //! @copydoc Object::getCOM
        rw::math::Vector3D<double> getCOM (rw::kinematics::State& state) const { return _com; }

      protected:
        friend class WorkCell;

        //! @copydoc Object::getGeometry
        const std::vector< rw::geometry::Geometry::Ptr >&
        doGetGeometry (const rw::kinematics::State& state) const
        {
            return getGeometry ();
        }
        //! @copydoc Object::getModels
        const std::vector< rw::geometry::Model3D::Ptr >&
        doGetModels (const rw::kinematics::State& state) const
        {
            return getModels ();
        }

      private:
        std::vector< rw::geometry::Geometry::Ptr > _geometry;
        std::vector< rw::geometry::Model3D::Ptr > _models;
        double _mass;
        rw::math::InertiaMatrix<> _Ibody;
        rw::math::Vector3D<double> _com;
    };

    /*@}*/
}}    // namespace rw::models

#endif /* OBJECT_HPP_ */
