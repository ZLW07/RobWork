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

#include "PropertyViewDialog.hpp"

#include "ui_PropertyViewDialog.h"

using namespace rw::core;
using namespace rw::math;

PropertyViewDialog::PropertyViewDialog (rw::core::PropertyMap::Ptr map, QWidget* parent) :
    QDialog (parent), _pOriginalProperties (map), _workingCopy (*map.get ())
{
    ui = new Ui_PropertyViewDialog ();
    ui->setupUi (this);
    ui->propertyViewEditor->setPropertyMap (&_workingCopy);

    connect (ui->buttonBox, SIGNAL (accepted ()), this, SLOT (acceptPressed ()));
    connect (ui->buttonBox, SIGNAL (rejected ()), this, SLOT (rejectPressed ()));
}

void PropertyViewDialog::acceptPressed ()
{
    (*_pOriginalProperties) = _workingCopy;
    accept ();
}

void PropertyViewDialog::rejectPressed ()
{
    reject ();
}
