/*
Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

*/

#ifndef RWLIBS_SOFTBODY_FDUTIL_HPP
#define RWLIBS_SOFTBODY_FDUTIL_HPP

#include <Eigen/Core>

namespace rwlibs { namespace softbody {
    /** @addtogroup softbody */
    /*@{*/

    /**
     * @brief Various numerical methods using finite-differences
     **/
    class FdUtil
    {
      public:
        /**
         * @brief calculates the derivatives of a vector
         *
         * calculates the derivatives of a vector using second-order accurate, centered FD
         *expressions at the interior points and first-order accurate forward/backward differences
         * at the endpoints
         *
         * @param f vector of function values
         * @param df vector to put the derivatives in
         * @param h stepsize
         **/
        static void vectorDerivative (const Eigen::VectorXd& f, Eigen::VectorXd& df,
                                      const double h);
    };
    /*@}*/
}}    // namespace rwlibs::softbody

#endif    // FDUTIL_HPP
