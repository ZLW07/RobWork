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

#ifndef RW_CORE_PAIRITERATOR_HPP
#define RW_CORE_PAIRITERATOR_HPP

#if !defined(SWIG)
#include <utility>
#endif

namespace rw { namespace core {

    /** @addtogroup core */
    /*@{*/

    /**
     * @brief this is a conversion class to make for that c++ 11 ranged for loops
     * can't iterate over a pair of iterators like BOOST_FOREACH could.
     * use case:
     * for (auto item&: getIter_par()) do_somthingg();
     */
    template< typename I > class iter_pair : public std::pair< I, I >
    {
      public:

        using std::pair<I,I>::pair;

        iter_pair (std::pair< I, I > input) : std::pair< I, I > (input) {}
        /**
         * @brief get the beginning of the iterator list
         * @return iterator
         */
        I begin () { return this->first; }

        /**
         * @brief get the end of the iterator list
         * @return iterator
         */
        I end () { return this->second; }

    }; 

    template< typename T1 > constexpr iter_pair< T1 > make_iterPair (T1 x, T1 y)
    {
        return std::pair<T1,T1> (std::forward< T1 > (x), std::forward< T1 > (y));
    }

}}    // namespace rw::core

#endif