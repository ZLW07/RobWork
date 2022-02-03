#ifndef RW_SENSOR_CONTACT2D_HPP_
#define RW_SENSOR_CONTACT2D_HPP_

#if !defined(SWIG)
#include <rw/math/Vector2D.hpp>
#endif 
namespace rw { namespace sensor {

    /**
     * @brief data structure for describing a contact in 2D
     */
    class Contact2D
    {
      public:
        //! @brief Contact position
        rw::math::Vector2D< double > p;

        //! @brief Surface contact normal
        rw::math::Vector2D< double > n;

        //! @brief surface curvature
        double curvature;

        //! @brief double moving average of the curvature
        double avgCurvature;

        //! @brief coulomb friction coefficient
        double mu;
    };

}}    // namespace rw::sensor

#endif /*CONTACT_HPP_*/
