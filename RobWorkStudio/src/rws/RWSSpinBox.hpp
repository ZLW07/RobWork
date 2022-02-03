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

#ifndef RWS_RWSSPINBOX_HPP_
#define RWS_RWSSPINBOX_HPP_

#include <QDoubleSpinBox>
#include <QValidator>


class QString;

namespace rws {


//! @brief Implementation of QDoubleSpinBox with better fixup function
class RWSSpinBox : public QDoubleSpinBox
{
  public:
    /**
     * @brief a QDoubleSpinBox custom designed to for the application
     * @param low [in] minimum value
     * @param high [in] maximum value
     */
    RWSSpinBox (double low, double high);

    /**
     * @brief Default constructor
     */
    RWSSpinBox() : QDoubleSpinBox() {}

    /**
     * @brief overriden virtual function from QDoubleSpinBox. Removes letters and reduces to the
     * right number of decimals
     * @param input [in] text to fix
     */
    void fixup (QString& input) const;

    /**
     * @brief overriden virtual function from QDoubleSpinBox. Validates that text is a number
     * @param text [in] text to be validated
     * @param pos [in] index of changed parameter
     */
    QValidator::State validate (QString& text, int& pos) const;

    /**
     * @brief overriden virtual function from QDoubleSpinBox. convert text to double
     * @param text [in] text to be converted to number
     */
    double valueFromText (const QString& text) const;
};
}    // namespace rws

#endif