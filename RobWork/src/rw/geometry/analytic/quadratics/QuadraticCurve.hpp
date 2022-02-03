/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_

/**
 * @file QuadraticCurve.hpp
 *
 * \copydoc rw::geometry::QuadraticCurve
 */
#if !defined(SWIG)
#include <rw/geometry/OBB.hpp>
#include <rw/geometry/analytic/ParametricCurve.hpp>
#include <rw/math/Vector3D.hpp>
#endif
namespace rw { namespace geometry {
    //! @addtogroup geometry
#if !defined(SWIG)
    //! @{
#endif
    /**
     * @brief A quadratic curve.
     *
     * A quadratic curve is given explicitly by the expression \f$ \mathbf{p} = \mathbf{c} + r(t)
     * \mathbf{u} + s(t) \mathbf{v} \f$ where \f$ \mathbf{c},\mathbf{u},\mathbf{v},\mathbf{p} \in
     * \mathbb{R}^3 \f$ and \f$ \mathbf{u}^T \mathbf{v} = 0\f$ .
     *
     * The following four types of curves are possible:
     * - Ellipse: \f$ (r,s)=(\sin t,\cos t) \f$
     * - Hyperbola: \f$ (r,s)=(\sinh t,\cosh t) \f$
     * - Line: \f$ (r,s)=(t,0) \f$
     * - Parabola: \f$ (r,s)=(t,t^2) \f$
     */
#if !defined(SWIGJAVA)
    class QuadraticCurve : public ParametricCurve
#else
    class QuadraticCurve
#endif
    {
      public:
        //! @brief Smart pointer type for QuadraticCurve.
        typedef rw::core::Ptr< QuadraticCurve > Ptr;

        //! @brief Smart pointer type for a const QuadraticCurve.
        typedef rw::core::Ptr< const QuadraticCurve > CPtr;

        //! @brief The four possible curve types.
        typedef enum Type {
            Elliptic,     //!< Ellipse \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} \sin t + \mathbf{v}
                          //!< \cos t\f$
            Hyperbola,    //!< Hyperbola \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} \sinh t +
                          //!< \mathbf{v} \cosh t\f$
            Line,         //!< Line \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} t\f$
            Parabola    //!< Parabola \f$ \mathbf{p} = \mathbf{c} + \mathbf{u} t + \mathbf{v} t^2\f$
        } Type;

        /**
         * @brief Construct quadratic curve given by the expression \f$ \mathbf{p} = \mathbf{c} +
         * r(t) \mathbf{u} + s(t) \mathbf{v} \f$, where u and v are orthogonal vectors.
         * @param c [in] offset of curve.
         * @param u [in] first direction.
         * @param v [in] second direction.
         * @param type [in] the type of curve.
         */
        QuadraticCurve (const rw::math::Vector3D<double>& c, const rw::math::Vector3D<double>& u,
                        const rw::math::Vector3D<double>& v, const Type& type);


        QuadraticCurve ():_type(Type::Parabola){}

        //! @brief Destrcutor.
        virtual ~QuadraticCurve ();

        // Curve interface
        //! @copydoc ParametricCurve::transform(const rw::math::Transform3D<>&) const
        QuadraticCurve::Ptr transform (const rw::math::Transform3D< double>& T) const;

        //! @copydoc ParametricCurve::transform(const rw::math::Vector3D<double>&) const
        QuadraticCurve::Ptr transform (const rw::math::Vector3D< double >& P) const;

        //! @copydoc ParametricCurve::scale
        QuadraticCurve::Ptr scale (double factor) const;

        //! @copydoc ParametricCurve::reverse
        QuadraticCurve::Ptr reverse () const;

        //! @copydoc ParametricCurve::clone
        QuadraticCurve::Ptr clone () const;

        //! @copydoc ParametricCurve::extremums
        virtual std::pair< double, double > extremums (const rw::math::Vector3D<double>& dir) const;

        //! @copydoc ParametricCurve::discretizeAdaptive
        virtual std::list< rw::math::Vector3D< double > >
        discretizeAdaptive (double stepsPerRevolution) const;

        //! @copydoc ParametricCurve::obr
        virtual OBB<> obr () const;

        //! @copydoc ParametricCurve::closestPoints
        virtual std::vector< rw::math::Vector3D<double> >
        closestPoints (const rw::math::Vector3D<double>& p) const;

        //! @copydoc ParametricCurve::equals
        virtual bool equals (rw::core::Ptr<const rw::geometry::Curve> curve, double eps) const;

        // ParametricCurve interface
        //! @copydoc ParametricCurve::x
        virtual rw::math::Vector3D<double> x (double t) const;

        //! @copydoc ParametricCurve::dx
        virtual rw::math::Vector3D<double> dx (double t) const;

        //! @copydoc ParametricCurve::ddx
        virtual rw::math::Vector3D<double> ddx (double t) const;

#if !defined(SWIG)
        //! @copydoc ParametricCurve::x(double) const
        virtual rw::math::Vector3D<double> operator() (double t) const;
#else 
        CALLOPERATOR(rw::math::Vector3D<double>,double );
#endif 

        //! @copydoc ParametricCurve::hasLimits
        virtual bool hasLimits () const { return _hasLimits; }

        //! @copydoc ParametricCurve::limits
        virtual const std::pair< double, double >& limits () const { return _limits; }

        //! @copydoc ParametricCurve::inLimits
        virtual bool inLimits (double t) const;

        //! @copydoc ParametricCurve::setLimits
        virtual void setLimits (const std::pair< double, double >& limits);

        //! @copydoc ParametricCurve::curvature
        virtual double curvature (double t) const;

        //! @copydoc ParametricCurve::closestTimes
        virtual std::vector< double > closestTimes (const rw::math::Vector3D<double>& p) const;

        //! @copydoc ParametricCurve::closestTime
        virtual double closestTime (const rw::math::Vector3D<double>& p) const;

        // QuadraticCurve
        /**
         * @brief The point c.
         * @return the point \f$ c \in \mathbb{R}^3 \f$.
         */
        const rw::math::Vector3D<double>& c () const { return _c; }

        /**
         * @brief The vector u.
         * @return the vector \f$ u \in \mathbb{R}^3 \f$.
         */
        const rw::math::Vector3D< double>& u () const { return _u; }

        /**
         * @brief The vector v.
         * @return the vector \f$ v \in \mathbb{R}^3 \f$.
         */
        const rw::math::Vector3D<double>& v () const { return _v; }

        /**
         * @brief Get the type of curve.
         * @return the type of curve.
         */
        Type type () const { return _type; }

        QuadraticCurve & operator=(const QuadraticCurve &rhs){
            *this=QuadraticCurve(rhs);
            return *this;
        }

      private:
        double r (double t) const;
        double s (double t) const;

        std::list< double > discretizeEllipse (double stepsPerRevolution) const;
#if !defined(SWIGJAVA)
        virtual rw::geometry::ParametricCurve::Ptr doScaleParametricCurve (double factor) const
        {
            return scale (factor);
        }
        virtual rw::geometry::ParametricCurve::Ptr
        doTransformParametricCurve (const rw::math::Vector3D<double>& P) const
        {
            return transform (P);
        }
        virtual rw::geometry::ParametricCurve::Ptr
        doTransformParametricCurve (const rw::math::Transform3D<>& T) const
        {
            return transform (T);
        }
        virtual rw::geometry::ParametricCurve::Ptr doReverseParametricCurve () const
        {
            return reverse ();
        }
        virtual rw::geometry::ParametricCurve::Ptr doCloneParametricCurve () const
        {
            return clone ();
        }
#endif
        const rw::math::Vector3D<double> _c;
        const rw::math::Vector3D<double> _u;
        const rw::math::Vector3D<double> _v;
        const Type _type;

        bool _hasLimits;
        std::pair< double, double > _limits;
    };
#if !defined(SWIG)
//! @}
#endif
}}    // namespace rw::geometry

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_QUADRATICCURVE_HPP_ */
