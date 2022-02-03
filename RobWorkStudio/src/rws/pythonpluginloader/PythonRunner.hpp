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

#ifndef RWS_PYTHONPLUGIN_PYTHONRUNNER_HPP
#define RWS_PYTHONPLUGIN_PYTHONRUNNER_HPP

#include <string>

struct _ts;

namespace rws { namespace python {
    class PythonLock;
    using PyThreadState = _ts;
    /**
     * @brief a class for running multiple python environments in a threadsafe manner
     */
    class PythonRunner
    {
      public:
        /**
         * @brief construct a Python interpretor
         */
        PythonRunner ();

        /**
         * @brief destruct the python interpertor
         */
        ~PythonRunner ();

        /**
         * @brief run code with python lock
         * @param code [in] python code to be run
         * @return 0 if succesfull
         */
        int runCode (std::string code);

      private:
        void initPython ();

        PyThreadState* _threadState;
    };

    /**
     * @brief a Python lock for making sure only one thread can talk with python at a time
     */
    class PythonLock
    {
      public:
        /**
         * @brief get the lock for Threadstate
         */
        PythonLock (PyThreadState*);

        /**
         * @brief release lock
         */
        ~PythonLock ();

      private:
        PyThreadState* _threadState;
    };

}}    // namespace rws::python
#endif