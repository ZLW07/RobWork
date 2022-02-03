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

#define QT_NO_EMIT

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#ifdef _WIN32
#include <windows.h>
#endif    //#ifdef _WIN32

int main (int argc, char** argv)
{
    std::string arg;
    for(int i = 0; i < argc; i++){
        arg += argv[i];
        arg += " ";
    }
    rws::RobWorkStudioApp app(arg);
    app.run();
    return 0;
}

#ifdef _WIN32
int WINAPI WinMain (HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
    return main (__argc, __argv);
}
#endif
