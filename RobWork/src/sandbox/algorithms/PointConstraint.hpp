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
 
 

#ifndef RW_ALGORITHMS_POINTCONSTRAINT_HPP
#define RW_ALGORITHMS_POINTCONSTRAINT_HPP



/**
 * @file PointConstraint.hpp
 */

#include <rw/math/Vector3D.hpp>

#include "ConstraintModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A point constraint model.
 * 
 * Describes a point constraint, e.g. a part feeder.
 */
class PointConstraint : public ConstraintModel {
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::core::Ptr<PointConstraint> Ptr;
		
		//! @copydoc ConstraintModel::MinSamples
		static const int MinSamples = 1;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		PointConstraint() {};
		
		//! @brief Constructor.
		PointConstraint(const std::vector<rw::math::Transform3D<> >& data)
		{
			refit(data);
		}
		
		//! @brief Destructor.
		virtual ~PointConstraint() {};

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(rw::math::Transform3D<> sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Transform3D<> >& samples);
		
		//! @copydoc RANSACModel::getMinReqData
		static int getMinReqData() { return MinSamples; }
		
		//! @copydoc ConstraintModel::update
		virtual void update(rw::math::Transform3D<> sample);
		
		//! @copydoc ConstraintModel::update
		virtual void update(std::vector<rw::math::Transform3D<> > sample);

	protected: // body
		rw::math::Vector3D<double> _point;
};



}} // /namespaces

#endif // include guard
