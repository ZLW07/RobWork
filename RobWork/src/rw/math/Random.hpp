/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_MATH_RANDOM_HPP_
#define RW_MATH_RANDOM_HPP_

/**
 * @file Random.hpp
 *
 * \copydoc rw::math::Random
 */

namespace rw { namespace math {
    //! @addtogroup math
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief Generation of random numbers.
     */
    class Random
    {
      public:
        Random ()  = delete;
        ~Random () = delete;

        /**
         * @brief A random double in the range [0, 1[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran ();

        /**
         * @brief Seeds the random number generator.
         *
         * @note Uses boost::random
         */
        static void seed (unsigned seed);

        /**
         * @brief Seeds the random number generator with current time of day
         *
         * @note Uses boost::random
         */
        static void seed ();

        /**
         * @brief A random double in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static double ran (double from, double to);

        /**
         * @brief A random integer in the range [from, to[ using a uniform distribution.
         *
         * @note Uses boost::random
         */
        static int ranI (int from, int to);

        /**
         * @brief Returns a random sample around \b mean with standard deviation \b sigma  using the
         * normal distribution.
         *
         * @note Uses boost::random
         *
         * @param mean [in] Means value
         * @param sigma [in] Standard deviation
         * @return Random sample
         */
        static double ranNormalDist (double mean, double sigma);
    };
    //! @}
}}    // namespace rw::math

#endif /* RW_MATH_RANDOM_HPP_ */
