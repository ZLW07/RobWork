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

#ifndef RWS_ROBWORKSTUDIOAPP_HPP_
#define RWS_ROBWORKSTUDIOAPP_HPP_
#include "RobWorkStudio.hpp"

#include <boost/thread.hpp>
#include <thread>

#define RWS_START(rwsapp)                                \
    rws::RobWorkStudioApp& rws_macro_interface = rwsapp; \
    std::thread rws_macro_thread([&] { while(!rwsapp.isRunning ()){ rw::common::TimerUtil::sleepMs (1);}
#define RWS_END(end)            \
    });                         \
    rws_macro_thread.detach (); \
    rws_macro_interface.run ();

namespace rws {

/**
 * @brief a RobWorkStudio main application which may be instantiated in its own thread.
 * The app can be started with either a call to the run() function or the start() function depending
 * on weather you want to run it from current thread or start it up in another tread.
 *
 * For convinienve when running the app from the main tread the macros 
 * RWS_START(RobWorkStudioApp app) and RWS_END() can be used to capsulate code to run in a new thread
 */
class RobWorkStudioApp
{
  public:
    /**
     * @brief constructor
     * @param args [in] command line arguments for RobWorkStudio
     */
    RobWorkStudioApp (const std::string& args);

    //! destructor
    virtual ~RobWorkStudioApp ();

    /**
     * @brief start RobWorkStudio in its own thread, the function is blocking until rws is up and
     * running
     */
    void start ();

    /**
     * @brief start RobWorkStudio in this thread. Notice this method call will
     * block until RobWorkStudio is exited.
     * @return zero if exited normally.
     */
    int run ();

    /**
     * @brief check if RobwWrkStudio is running
     * @return true if running false otherwise
     */
    bool isRunning () { return _isRunning; }

    /**
     * @brief Close RobWorkStudio. Blocking until rws is closed. This might take awaile.
     */
    void close ();

    /**
     * @brief get handle to the running RobWorkStudio instance.
     * @note do not directly change Qt visualization objects, this will
     * produce segfaults. Instead use Qt events and the post* handles on
     * RobWorkStudio interface.
     * @return handle to RobWorkStudio
     */
    RobWorkStudio* getRobWorkStudio () { return _rwstudio; };

  private:
    RobWorkStudio* _rwstudio;
    std::string _args;
    boost::thread* _thread;
    bool _isRunning;
};
}    // namespace rws

#endif /* ROBWORKSTUDIOAPP_HPP_ */
