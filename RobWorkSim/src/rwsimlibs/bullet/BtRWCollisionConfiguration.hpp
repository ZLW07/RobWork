/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_BULLET_BTRWCOLLISIONCONFIGURATION_HPP_
#define RWSIMLIBS_BULLET_BTRWCOLLISIONCONFIGURATION_HPP_

/**
 * @file BtRWCollisionConfiguration.hpp
 *
 * \copydoc rwsimlibs::bullet::BtRWCollisionConfiguration
 */

#include <rw/core/Ptr.hpp>

#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <LinearMath/btScalar.h>

namespace rwsim { namespace contacts {
    class ContactDetector;
}}    // namespace rwsim::contacts

namespace rwsimlibs { namespace bullet {
    //! @addtogroup rwsimlibs_bullet

    //! @{
    /**
     * @brief A collision configuration that uses the btCompoundCompoundCollisionAlgorithm and
     * BtRWCollisionAlgorithm to handle contacts.
     */
    class BtRWCollisionConfiguration : public btCollisionConfiguration
    {
      public:
        /**
         * @brief Construct new collision configuration.
         * @param detector [in] the RobWork ContactDetector to use.
         */
        BtRWCollisionConfiguration (
            rw::core::Ptr< const rwsim::contacts::ContactDetector > detector);

        //! @brief Destructor.
        virtual ~BtRWCollisionConfiguration ();

        /**
         * @brief Get pool of allocated manifolds.
         * @return the allocator.
         */
        virtual btPoolAllocator* getPersistentManifoldPool ();

        /**
         * @brief Get algorithm allocator.
         * @return the allocator.
         */
        virtual btPoolAllocator* getCollisionAlgorithmPool ();

#if BT_BULLET_VERSION < 282
        /**
         * @brief Get stack allocator (for older versions of Bullet).
         * @return pointer to the stack allocator.
         * @deprecated Only defined for backwards compatibility with Bullet 2.81.
         */
        virtual btStackAlloc* getStackAllocator ();
#endif

        /**
         * @brief Get the algorithm allocator to use.
         *
         * If both are compounds the btCompoundCompoundCollisionAlgorithm is used, otherwise
         * BtRWCollisionAlgorithm is used.
         *
         * @param proxyType0 [in] the type of first object.
         * @param proxyType1 [in] the type of second object.
         * @return the collision algorithm allocator.
         */
        virtual btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc (int proxyType0,
                                                                                 int proxyType1);

#if BT_BULLET_VERSION >= 286
        /**
         * @brief Get the algorithm allocator to use.
         * @param proxyType0 [in] the type of first object.
         * @param proxyType1 [in] the type of second object.
         * @return the collision algorithm allocator.
         */
        virtual btCollisionAlgorithmCreateFunc*
        getClosestPointsAlgorithmCreateFunc (int proxyType0, int proxyType1);
#endif

      private:
        btCollisionAlgorithmCreateFunc* m_compoundCompoundCreateFunc;
        btCollisionAlgorithmCreateFunc* m_compoundCreateFunc;
        btCollisionAlgorithmCreateFunc* m_swappedCompoundCreateFunc;
        btCollisionAlgorithmCreateFunc* _func;
        btPoolAllocator* m_persistentManifoldPool;
        btPoolAllocator* m_collisionAlgorithmPool;
#if BT_BULLET_VERSION < 282
        btStackAlloc* m_stackAlloc;
#endif
    };
    //! @}
}}     // namespace rwsimlibs::bullet
#endif /* RWSIMLIBS_BULLET_BTRWCOLLISIONCONFIGURATION_HPP_ */
