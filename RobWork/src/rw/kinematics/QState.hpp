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

#ifndef RW_KINEMATICS_QSTATE_HPP
#define RW_KINEMATICS_QSTATE_HPP

/**
 * @file QState.hpp
 */
#if !defined(SWIG)
#include <rw/core/Ptr.hpp>
#include <rw/core/macros.hpp>
#include <rw/math/Q.hpp>
#endif
namespace rw { namespace kinematics {
    class StateSetup;
    class StateData;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief The configuration state of a work cell.
     *
     * The configuration state contains state data values for all
     * valid StateData in a StateStructure. The validity is defined by the
     * StateSetup.
     *
     * See Frame::getTransform() for the calculation of the relative transform
     * of a frame for a given configuration state.
     *
     * Configuration states can be freely copied and assigned.
     *
     * The configuration state is a part of the StateStructure state (see
     * State).
     */
    class QState
    {
      public:
        /**
         * @brief Constructs an empty QState
         */
        QState ();

        /**
         * @brief A configuration state.
         *
         * This constructor is not available for use outside of RobWork. Instead
         * your configuration states should be constructed via the copy
         * constructor.
         *
         * @param setup [in] The shared setup for configuration states.
         */
        explicit QState (rw::core::Ptr< rw::kinematics::StateSetup > setup);

        //! destructor
        virtual ~QState ();

        /**
         * @brief An array of length frame.getDOF() containing the joint values
         * for \b frame.
         *
         * It is OK to call this method also for frames with zero degrees of
         * freedom.
         *
         * @return The joint values for the frame.
         */
        double* getQ (const rw::kinematics::StateData& data);

        /**
         * @brief non const version of getQ.
         */
        const double* getQ (const rw::kinematics::StateData& data) const;

        /**
         * @brief Assign for \b frame the frame.getDOF() joint values of the
         * array \b vals.
         *
         * The array \b vals must be of length at least frame.getDOF().
         *
         * @param data [in] The StateData for which the joint values are assigned.
         *
         * @param vals [in] The joint values to assign.
         *
         * setQ() and getQ() are related as follows:
         * \code
         * q_state.setQ(frame, q_in);
         * const double* q_out = q_state.getQ(frame);
         * for (int i = 0; i < frame.getDOF(); i++)
         *   q_in[i] == q_out[i];
         * \endcode
         */
        void setQ (const rw::kinematics::StateData& data, const double* vals);

#if !defined(SWIG)
        /**
         * @brief streaming operator
         *
         * @param os [in] output stream
         *
         * @param state [in] state to stream out
         *
         * @return the stream
         */
        friend std::ostream& operator<< (std::ostream& os, const QState& state)
        {
            os << state._contents;
            return os;
        }
#else
        TOSTRING (rw::kinematics::QState);
#endif

        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        QState operator* (double scale) const
        {
            return QState (scale * this->_contents, this->_setup);
        }

        /**
         * @brief Scaling of a configuration state by division
         */
        QState operator/ (double scale) const
        {
            return QState (this->_contents / scale, this->_setup);
        }
#if !defined(SWIGPYTHON)
        /**
         * @brief Scaling of a configuration state by a scalar.
         */
        friend QState operator* (double scale, const QState& q)
        {
            return QState (scale * q._contents, q._setup);
        }
#endif

        /**
         * @brief Addition of configuration states.
         */
        QState operator+ (const QState& b) const
        {
            // It does not matter here if we use the setup of a or b.
            // They are assumed to be the identical.
            return QState (this->_contents + b._contents, this->_setup);
        }

        /**
         * @brief Subtraction of configuration states.
         */
        QState operator- (const QState& b) const
        {
            return QState (this->_contents - b._contents, this->_setup);
        }

        /**
         * @brief Unary minus operator.
         */
        QState operator- () const { return QState (-_contents, _setup); }

        /**
         * @brief returns the StateSetup
         */
        rw::core::Ptr< rw::kinematics::StateSetup > getStateSetup () const { return _setup; }
 
        /*
         * @brief Assignment operator.
         * @param rhs [in] the other QState to assign to this.
         * @return a reference to this QState.
         */
        QState& operator= (const QState& rhs);

        /**
           @brief The dimension of the state vector.
         */
        size_t size () const { return _contents.size (); }

#if !defined(SWIG)
        /**
         * @brief Get element of state.
         * @param index [in] the index.
         * @return the value at given index.
         */
        double& operator() (size_t index)
        {
            RW_ASSERT (index < size ());
            return _contents (index);
        }

        //! @copydoc operator()(size_t)
        const double& operator() (size_t index) const
        {
            RW_ASSERT (index < size ());
            return _contents (index);
        }

        /**
         * @brief Get element of state.
         * @param index [in] the index.
         * @return the value at given index.
         */
        double& operator[] (size_t index)
        {
            RW_ASSERT (index < size ());
            return _contents (index);
        }

        /**
         * @brief Get element of state.
         * @param index [in] the index.
         * @return the value at given index.
         */
        double operator[] (size_t index) const
        {
            RW_ASSERT (index < size ());
            return _contents (index);
        }
#else
        ARRAYOPERATOR (double);
#endif
      private:
        QState (const math::Q& contents, rw::core::Ptr< StateSetup > setup) :
            _contents (contents), _setup (setup)
        {}

      private:
        rw::math::Q _contents;
        rw::core::Ptr< rw::kinematics::StateSetup > _setup;
    };

    /*@}*/
}}    // namespace rw::kinematics

#endif
