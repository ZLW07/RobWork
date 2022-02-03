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

#ifndef RW_MODELS_DHPARAMETERSET_HPP
#define RW_MODELS_DHPARAMETERSET_HPP

#if !defined(SWIG)
#include <rw/core/Ptr.hpp>

#include <string>
#include <vector>
#endif 

namespace rw { namespace core {
    class PropertyMap;
}}    // namespace rw::core
namespace rw { namespace kinematics {
    class Frame;
}}    // namespace rw::kinematics

namespace rw { namespace models {

    class SerialDevice;
    class Joint;

    /**
     * @brief Simple class to help represent a set of Denavit-Hartenberg
     * parameters
     */
    class DHParameterSet
    {
      public:
        /**
         * @brief Constructor for DHParameters initialized to zero.
         */
        DHParameterSet () :
            _alpha (0), _a (0), _d (0), _theta (0), _beta (0), _b (0), _isParallel (false)
        {}

        /**
         * @brief Constructor
         * @param alpha [in] \f$ \alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param d [in] \f$ d_{i}\f$
         * @param theta [in] \f$ \theta_{i-1}\f$
         */
        DHParameterSet (double alpha, double a, double d, double theta) :
            _alpha (alpha), _a (a), _d (d), _theta (theta), _beta (0), _b (0), _isParallel (false)
        {}

        /**
         * @brief Constructor
         * @param alpha [in] \f$ \alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param d [in] \f$ d_{i}\f$
         * @param theta [in] \f$ \theta_{i-1}\f$
         * @param type documentation missing !
         *
         */
        DHParameterSet (double alpha, double a, double d, double theta, const std::string& type) :
            _alpha (alpha), _a (a), _d (d), _theta (theta), _beta (0), _b (0), _isParallel (false),
            _type (type)
        {}

        /**
         * @brief Constructor
         * @param alpha [in] \f$ \alpha_{i-1}\f$
         * @param a [in] \f$ a_{i-1}\f$
         * @param beta [in] documentation missing !
         * @param b [in] documentation missing !
         * @param parallel [in] documentation missing !
         * @param theta [in] \f$ \theta_{i-1}\f$
         */
        DHParameterSet (double alpha, double a, double beta, double b, bool parallel) :
            _alpha (alpha), _a (a), _d (0), _theta (0), _beta (beta), _b (b),
            _isParallel (parallel), _type ("HGP")
        {}

        /** @brief \f$ \alpha_{i-1}\f$ **/
        double alpha () const { return _alpha; }

        /** @brief \f$ a_{i-1}\f$ **/
        double a () const { return _a; }

        /** @brief \f$ d_{i} \f$ **/
        double d () const { return _d; }

        /** $brief \f$ \theta_{i} \f$ **/
        double theta () const { return _theta; }

        double b () const { return _b; }

        double beta () const { return _beta; }

        bool isParallel () const { return _isParallel; }

        /**
         * @brief the DH-convention type
         */
        std::string getType () const { return _type; }

        /**
         * @brief Returns the DH-Parameters for a SerialDevice.
         *
         * If no or only a partial DH representation exists only the list will be empty or
         * non-complete.
         *
         * @param device [in] SerialDevice for which to get the DH parameters
         * @return The set of DH parameters
         */
        static std::vector< DHParameterSet > getDHParameters (rw::core::Ptr< rw::models::SerialDevice > device);

        static const DHParameterSet* get (const rw::core::PropertyMap& pmap);

        static const DHParameterSet* get (const rw::models::Joint* joint);

        static void set (const DHParameterSet& dhset, rw::core::PropertyMap& pmap);

        static void set (const DHParameterSet& dhset, rw::kinematics::Frame* joint);

      private:
        /** @brief \f$ \alpha_{i-1}\f$ **/
        double _alpha;
        /** @brief \f$ a_{i-1}\f$ **/
        double _a;
        /** @brief \f$ d_{i} \f$ **/
        double _d;
        /** $brief \f$ \theta_{i} \f$ **/
        double _theta;
        double _beta;
        double _b;
        bool _isParallel;
        std::string _type;
    };

}}    // namespace rw::models

#endif    //#ifndef RW_MODELS_DHPARAMETERSET_HPP
