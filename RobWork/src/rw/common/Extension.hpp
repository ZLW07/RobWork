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

#ifndef RW_COMMON_EXTENSION_HPP_
#define RW_COMMON_EXTENSION_HPP_

#include <rw/core/Extension.hpp>
#include <rw/core/os.hpp>
#ifdef RW_WIN32
#pragma message( \
    "#include <rw/common/Extension.hpp> is deprecated use #include <rw/core/Extension.hpp> instead")
#else
#warning \
    "#include <rw/common/Extension.hpp> is deprecated use #include <rw/core/Extension.hpp> instead"
#endif

#endif