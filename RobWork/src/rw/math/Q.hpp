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

#ifndef RW_MATH_Q_HPP
#define RW_MATH_Q_HPP

/**
 * @file Q.hpp
 */

#if !defined(SWIG)
#include <rw/common/Serializable.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/macros.hpp>

#include <Eigen/Core>
#include <boost/serialization/split_free.hpp>
#include <vector>
#endif

namespace rw { namespace math {

    /**
     * @brief Configuration vector
     */
    class Q
    {
      private:
      public:
        //! Eigen vector used as internal datastructure.
        typedef Eigen::Matrix< double, Eigen::Dynamic, 1 > Base;

        //! Value type.
        typedef double value_type;

        /**
         * @brief A configuration of vector of length \b dim.
         */
        Q (size_t dim) : _vec (dim) {}

        Q (const Q& q) : _vec (q._vec) {}

        /**
         * @brief Default constructor.
         *
         * The vector will be of dimension zero.
         */
        Q () : _vec ((Base::Index) 0) {}

        /**
         * @brief Construct a configuration vector from a std::vector
         * expression.
         *
         * @param r [in] An expression for a vector of doubles
         */
        Q (const std::vector< double >& r);

        /**
         * @brief Creates a Q  initialized with values from \b values
         * @param n [in] this value describes the length of the conficuration. This behavior is
         * depreacted. if RW_Q_USE_NEW_CONSTRUCTOR is not defined an exception is thrown if the
         * number of extra arguments doesn't match the value. if RW_Q_USE_NEW_CONSTRUCTOR is defined
         * n is counted as a value unless it matches the extra number of arguments.
         * @param args [in] the values of the configuration
         * @deprecated the use of n to describe length is deprecated.
         */
        template< typename... ARGS > explicit Q (size_t n, ARGS... args) : _vec (n)
        {
            int i = 0;
            ParamExpansion (i, args...);
            //_vec(i--) = arg0;
            if (i == 0) {
                _vec (i--) = n;
            }
        }

        /**
         * @brief Creates a Q  initialized with values from \b values
         * @param n [in] this value describes the length of the conficuration. This behavior is
         * depreacted. if RW_Q_USE_NEW_CONSTRUCTOR is not defined an exception is thrown if the
         * number of extra arguments doesn't match the value. if RW_Q_USE_NEW_CONSTRUCTOR is defined
         * n is counted as a value unless it matches the extra number of arguments.
         * @param args [in] the values of the configuration
         * @deprecated the use of n to describe length is deprecated.
         */
        template< typename... ARGS > explicit Q (int n, ARGS... args) : _vec (n)
        {
            int i = 0;
            ParamExpansion (i, args...);
            //_vec(i--) = arg0;
            if (i == 0) {
                _vec (i--) = n;
            }
        }

        /**
         * @brief Creates a Q  initialized with values from \b values
         * @param arg0 [in] first value
         * @param args [in] the values of the configuration
         */
        template< typename... ARGS > explicit Q (double arg0, ARGS... args)
        {
            int i = 1;
            ParamExpansion (i, args...);
            //_vec(i--) = arg1;
            _vec (i--) = arg0;
        }
#if defined(SWIG)
        Q_SWIG_CONSTRUCTORS;
#endif
        /**
         * @brief Construct from Eigen base.
         * @param q [in] Eigen base.
         */
        Q (const Base& q) : _vec (q.rows ())
        {
            for (int i = 0; i < q.size (); i++)
                _vec (i) = q (i, 0);
        }

        //! @brief Destructor.
        virtual ~Q ();

        /**
         * @brief Returns Q of length \b n initialized with 0's
         */
        static Q zero (std::size_t n)
        {
            return Q (Eigen::Matrix< double, Eigen::Dynamic, 1 >::Zero (n));
        }

        /**
         * @brief The dimension of the configuration vector.
         */
        size_t size () const { return _vec.rows (); }

        /**
         * @brief True if the configuration is of dimension zero.
         */
        bool empty () const { return size () == 0; }

        /**
         * @brief Accessor for the internal Eigen vector state.
         */
        const Base& e () const { return _vec; }

        /**
         * @brief Accessor for the internal Eigen vector state.
         */
        Base& e () { return _vec; }

        /**
         * @brief Extracts a sub part (range) of this Q.
         * @param start [in] Start index
         * @param cnt [in] the number of elements to include
         * @return
         */
        const Q getSubPart (size_t start, size_t cnt) const
        {
            RW_ASSERT (start + cnt <= size ());

            Q res (cnt);
            for (size_t i = 0; i < cnt; i++) {
                res (i) = (*this)[start + i];
            }
            return res;
        }

        /**
         * @brief Set subpart of vector.
         * @param index [in] the initial index.
         * @param part [in] the part to insert beginning from \b index.
         */
        void setSubPart (size_t index, const Q& part)
        {
            RW_ASSERT (index + part.size () <= size ());
            for (size_t i = 0; i < part.size (); i++) {
                (*this)[index + i] = part (i);
            }
        }

        //----------------------------------------------------------------------
        // Norm utility methods

        /**
         * @brief Returns the Euclidean norm (2-norm) of the configuration
         * @return the norm
         */
        double norm2 () const
        {
            return _vec.norm ();
            // return norm_2(m());
        }

        /**
         * @brief Returns the Manhatten norm (1-norm) of the configuration
         * @return the norm
         */
        double norm1 () const
        {
            return _vec.lpNorm< 1 > ();
            // return norm_1(m());
        }

        /**
         * @brief Returns the infinte norm (\f$\inf\f$-norm) of the configuration
         * @return the norm
         */
        double normInf () const
        {
            return _vec.lpNorm< Eigen::Infinity > ();
            // return norm_inf(m());
        }

        //----------------------------------------------------------------------
        // Various operators
#if !defined(SWIG)
        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return const reference to element
         */
        const double& operator() (size_t i) const { return _vec (i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return reference to element
         */
        double& operator() (size_t i) { return _vec (i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return const reference to element
         */
        const double& operator[] (size_t i) const { return _vec (i); }

        /**
         * @brief Returns reference to vector element
         * @param i [in] index in the vector
         * @return reference to element
         */
        double& operator[] (size_t i) { return _vec (i); }
#else
        ARRAYOPERATOR (double);
#endif

        /**
         * @brief Scalar division.
         */
        const Q operator/ (double s) const { return Q (_vec / s); }

        /**
         * @brief Scalar multiplication.
         */
        const Q operator* (double s) const { return Q (_vec * s); }

        /**
         * @brief Scalar multiplication.
         */
        friend const Q operator* (double s, const Q& v) { return Q (s * v.e ()); }

#if !defined(SWIGPYTHON)
        /**
         * @brief Scalar division.
         */
        friend const Q operator/ (double s, const Q& v)
        {
            Q res = v;
            for (size_t i = 0; i < v.size (); i++)
                res (i) = s / v (i);
            return res;
        }
#endif

        /**
         * @brief Vector subtraction.
         */
        const Q operator- (const Q& b) const { return Q (_vec - b.e ()); }

        /**
         * @brief Vector addition.
         */
        const Q operator+ (const Q& b) const { return Q (_vec + b.e ()); }

        /**
         * @brief Compares \b this and \b q2 for equality.
         *
         * \b this and \b q2 are considered equal if and only if they have equal
         * length and if q1(i) == q2(i) for all i.
         * @param q2 [in]
         * @return True if this equals q2, false otherwise.
         */
        bool operator== (const Q& q2) const;

        /**
         * @brief Inequality operator
         * The inverse of operator==().
         */
        inline bool operator!= (const Q& q2) const { return !((*this) == q2); }

        /**
         * @brief Scalar multiplication.
         */
        Q& operator*= (double s)
        {
            _vec *= s;
            return *this;
        }

        /**
         * @brief Scalar division.
         */
        Q& operator/= (double s)
        {
            _vec /= s;
            return *this;
        }

        /**
         * @brief Vector addition.
         */
        Q& operator+= (const Q& v)
        {
            _vec += v.e ();
            return *this;
        }

        /**
         * @brief Vector subtraction.
         */
        Q& operator-= (const Q& v)
        {
            _vec -= v.e ();
            return *this;
        }

        /**
         * @brief Unary minus.
         */
        Q operator- () const { return Q (-_vec); }

        /**
         * @brief Compares whether this is less than \b q
         *
         * The less operator is defined such that the first index is the most significant. That is
         * if (*this)[0] < q[0] then true is returned. If (*this)[0] > q[0] false is returned and
         * only if (*this)[0] == q[0] is the next index considered.
         */
        bool operator< (const Q& q) const
        {
            RW_ASSERT (size () == q.size ());
            for (size_t i = 0; i < size (); i++) {
                if (_vec[i] < q[i])
                    return true;
                else if (_vec[i] > q[i])
                    return false;
            }
            return false;
        }

        /**
         * @brief Convert to a standard vector.
         * @param v [out] the result.
         */
        void toStdVector (std::vector< double >& v) const
        {
            v.resize (size ());
            for (size_t i = 0; i < size (); i++) {
                v[i] = _vec[i];
            }
        }

        /**
         * @brief Convert to a standard vector.
         * @return the result.
         */
        std::vector< double > toStdVector () const
        {
            std::vector< double > v (size ());
            toStdVector (v);
            return v;
        }
#if defined(SWIG)
        TOSTRING ();
#endif

      private:
        void ParamExpansion (int& i)
        {
            if (_vec.rows () == 0) {
                _vec = Base (i--);
            }
            else if (i == 0) {
                // do noting
            }
            else if (_vec.rows () != i) {
#ifndef RW_Q_USE_NEW_CONSTRUCTOR
                if (_vec.rows () < i) {
                    RW_THROW ("n.size ("
                              << _vec.rows () << ") != args.size (" << i << "). "
                              << "#define RW_Q_USE_NEW_CONSTRUCTOR if you want 'n' to be an "
                              << "argumeent instead of the size of the configuration");
                }
#else
                _vec = Base (i + 1);
#endif
            }
            else {
                i--;
            }
        }

        template< typename T > void ParamExpansion (int& i, T arg)
        {
            ParamExpansion (++i);
            _vec (i--) = arg;
        }

        template< typename T, typename... ARGS > void ParamExpansion (int& i, T arg, ARGS... args)
        {
            ParamExpansion (++i, args...);
            _vec (i--) = arg;
        }

        Base _vec;
    };

    /**
     * @brief Creates a Q of length \b n and initialized with values from \b values
     * The method reads n values from \b values and do not check whether reading out of bounds
     * @param n [in] Length of q.
     * @param values [in] Values to initialize with
     */
    template<> Q::Q (size_t n, double* values);

    /**
     * @brief Creates a Q of length \b n and initialized with values from \b values
     * The method reads n values from \b values and do not check whether reading out of bounds
     * @param n [in] Length of q.
     * @param values [in] Values to initialize with
     */
    template<> Q::Q (size_t n, const double* values);

    /**
     * @brief Creates a Q of length \b n and initialized with values from \b values
     * The method reads n values from \b values and do not check whether reading out of bounds
     * @param n [in] Length of q.
     * @param values [in] Values to initialize with
     */
    template<> Q::Q (int n, double* values);

    /**
     * @brief Creates a Q of length \b n and initialized with values from \b values
     * The method reads n values from \b values and do not check whether reading out of bounds
     * @param n [in] Length of q.
     * @param values [in] Values to initialize with
     */
    template<> Q::Q (int n, const double* values);

    /**
     * @brief Creates a Q of length \b n and initialize all values in \Q to \b value
     * @param n [in] Length of q.
     * @param value [in] Value to initialize
     */
    template<> Q::Q (int n, double values);

    /**
     * @brief Creates a Q of length \b n and initialize all values in \Q to \b value
     * @param n [in] Length of q.
     * @param value [in] Value to initialize
     */
    template<> Q::Q (size_t n, double values);

    /**
     * @brief Streaming operator.
     *
     * @relates Q
     */
    std::ostream& operator<< (std::ostream& out, const Q& v);

#if !defined(SWIG)
    /**
     * @brief Input streaming operator
     *
     * Parse input stream according to how operator<< streams out
     *
     * @relates Q
     * @param in [in] Input stream
     * @param q [in] Target of q read in
     * @return reference to \b in
     */
    std::istream& operator>> (std::istream& in, Q& q);
#endif

    /**
     * @brief The dot product (inner product) of \b a and \b b.
     * @relates Q
     */
    double dot (const Q& a, const Q& b);

    /**
     * @brief concatenates q1 onto q2 such that the returned q has
     * the configurations of q1 in [0;q1.size()[ and has q2 in
     * [q1.size();q1.size()+q2.size()[
     * @param q1 [in] the first Q
     * @param q2 [in] the second Q
     * @return the concatenation of q1 and q2
     */
    rw::math::Q concat (const Q& q1, const Q& q2);

    /*@}*/
}}    // namespace rw::math

namespace rw { namespace core {
    //! @copydoc rw::core::PropertyMap::findProperty(const std::string& identifier) const
    template<>
    rw::core::Ptr< Property< rw::math::Q > >
    PropertyMap::findProperty (const std::string& identifier) const;
}}    // namespace rw::core

namespace rw { namespace common {
    class OutputArchive;
    class InputArchive;
    namespace serialization {
        /**
         * @copydoc rw::common::serialization::write
         * @relatedalso rw::math::Q
         */
        template<>
        void write (const rw::math::Q& sobject, rw::common::OutputArchive& oarchive,
                    const std::string& id);

        /**
         * @copydoc rw::common::serialization::read
         * @relatedalso rw::math::Q
         */
        template<>
        void read (rw::math::Q& sobject, rw::common::InputArchive& iarchive, const std::string& id);
    }    // namespace serialization
}}       // namespace rw::common

namespace boost { namespace serialization {
    /**
     * @brief Boost serialization.
     * @param archive [in] the boost archive to read from or write to.
     * @param q [in/out] the vector to read/write.
     * @param version [in] class version (currently version 0).
     * @relatedalso rw::math::Q
     */
    template< class Archive >
    void serialize (Archive& archive, rw::math::Q& q, const unsigned int version)
    {
        split_free (archive, q, version);    // split into load and save
    }

    /**
     * @brief Boost serialization.
     * @param archive [in] the boost archive to write to.
     * @param q [in] the vector to write.
     * @param version [in] class version (currently version 0).
     * @relatedalso rw::math::Q
     */
    template< class Archive >
    void save (Archive& archive, const rw::math::Q& q, const unsigned int version)
    {
        const rw::math::Q::Base& e = q.e ();
#if BOOST_VERSION >= 105900
        archive << e.size ();
#else
        rw::math::Q::Base::Index size = e.size ();
        archive << size;
#endif
        for (rw::math::Q::Base::Index i = 0; i < e.size (); i++) {
            archive << e[i];
        }
    }

    /**
     * @brief Boost serialization.
     * @param archive [in] the boost archive to read from.
     * @param q [out] the vector to read.
     * @param version [in] class version (currently version 0).
     * @relatedalso rw::math::Q
     */
    template< class Archive >
    void load (Archive& archive, rw::math::Q& q, const unsigned int version)
    {
        rw::math::Q::Base& e = q.e ();
        rw::math::Q::Base::Index size;
        archive >> size;
        e.resize (size);
        for (rw::math::Q::Base::Index i = 0; i < size; i++) {
            archive >> e[i];
        }
    }
}}    // namespace boost::serialization

#endif    // end include guard
