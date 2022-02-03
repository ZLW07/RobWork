/*
 * Traits.hpp
 *
 *  Created on: Jan 11, 2012
 *      Author: jimali
 */

#ifndef RW_TRAITS_HPP_
#define RW_TRAITS_HPP_

namespace rw {

//! this is a forward declaration of the traits class
template< typename T > struct Traits;

template<> struct Traits<double>
{
    typedef double value_type;
};

template<> struct Traits<float>
{
    typedef float value_type;
};

template<class R, template < class > class T > struct Traits<T < R > >
{
    typedef typename Traits<R>::value_type value_type;
};

}    // namespace rw

#endif /* TRAITS_HPP_ */
