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

#ifndef RW_KINEMATICS_FKTABLE_HPP
#define RW_KINEMATICS_FKTABLE_HPP

/**
 * @file FKTable.hpp
 */
#if !defined(SWIG)
#include "FrameMap.hpp"
#include "State.hpp"

#include <rw/math/Transform3D.hpp>
#endif
/*
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index_container.hpp>
*/

namespace rw { namespace kinematics {

    class Frame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Forward kinematics for a set of frames.
     *
     * FKTable finds transforms for frames for a given fixed work cell state.
     * The frame transforms are calculated relative to the world frame.
     */
    class FKTable
    {
      public:
        /**
         * @brief Forward kinematics for the work cell state \b state.
         *
         * @param state [in] The work state for which world transforms are to be
         * calculated.
         */
        FKTable (const rw::kinematics::State& state);
#if !defined(SWIGPYTHON) && !defined(SWIGJAVA) && !defined(SWIGLUA)
        /**
         * @brief Forward kinematics for the work cell state \b state.
         * @param state [in] The work state for which world transforms are to be
         * calculated.
         */
        FKTable (const rw::kinematics::State* state);
#endif
        /**
         * @brief The world transform for the frame \b frame.
         *
         * @param frame [in] The frame for which to find the world transform.
         *
         * @return The transform of the frame relative to the world.
         */
        const rw::math::Transform3D<>& get (const rw::kinematics::Frame& frame) const;

        //! @copydoc get(const Frame&) const
        inline const rw::math::Transform3D<>& get (const rw::kinematics::Frame* frame) const { return get (*frame); }

        /**
         * @brief Returns State associated with the FKTable
         *
         * The State returned is the State used to calculate the forward kinematics.
         *
         * @return State used to calculate the forward kinematics
         */
        const rw::kinematics::State& getState () const { return *_sp; }

        /**
         * @brief resets the FKTable to \b state
         * @param state
         */
        void reset (const rw::kinematics::State& state);

      private:
        const State* _sp;
        State _state;

        typedef FrameMap< math::Transform3D<> > TransformMap;

        mutable TransformMap _transforms;

      private:
        FKTable (const FKTable&);
    };

    /*@}*/
}}    // namespace rw::kinematics

#endif    // end include guard
