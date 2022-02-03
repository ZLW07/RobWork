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

#include "LogMultiWriter.hpp"

using namespace rw::common;
using namespace rw::core;

LogMultiWriter::LogMultiWriter ()
{}

LogMultiWriter::~LogMultiWriter ()
{
    flush ();
}

void LogMultiWriter::addWriter (LogWriter::Ptr writer)
{
    _writers.push_back (writer);
}

void LogMultiWriter::doWrite (const std::string& str)
{
    for (LogWriter::Ptr writer : _writers) {
        writer->write (str);
    }
}

void LogMultiWriter::doFlush ()
{
    for (LogWriter::Ptr writer : _writers) {
        writer->flush ();
    }
}

void LogMultiWriter::doSetTabLevel (int tabLevel)
{
    for (LogWriter::Ptr writer : _writers) {
        writer->setTabLevel (tabLevel);
    }
}