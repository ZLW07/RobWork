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

#ifndef RW_CORE_OS_HPP
#define RW_CORE_OS_HPP
#if !defined(SWIG)
#include <boost/filesystem.hpp>
#include <string>

#endif

#if defined(__CYGWIN__)
#define RW_CYGWIN
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define RW_WIN32
#elif defined(_WIN64) || defined(__WIN64__) || defined(WIN64)
#define RW_WIN64
#elif defined(macintosh) || defined(__APPLE__) || defined(__APPLE_CC__)
#define RW_MACOS
#elif defined(linux) || defined(__linux) || defined(__linux__)
#define RW_LINUX
#endif

#if defined(RW_WIN32) || defined(RW_WIN64)
#define RW_WIN
#endif 

#ifdef RW_WIN
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT extern "C"
#endif

#if !defined(SWIG)
#ifdef RW_WIN
#include <windows.h>
#endif
#endif

/**
 * @brief encapsulates os dependent functionality
 */
class OS
{
  public:
    /**
     * @brief get the extension for dynamic linked libraries
     * @return returns the extension without a . infront
     */
    static std::string getDLLExtension ()
    {
#if defined(RW_WIN)
        return "dll";
#elif defined(RW_MACOS)
        return "dylib";
#else
        return "so";
#endif
    }

    /**
     * @brief find the location of where installed robwork plugins are placed.
     * @param pack [in] this is used for some systems where RobWork, RobWorkSim, RobWorkHardware and
     * RobWorkStudio don't install the plugins in the same location. This variable takes one of four
     * strings as argument {RobWork, RobWorkSim, RobWorkHardware and RobWorkStudio}
     * @return return "" if location not found
     */
    static std::string InstallPluginLocation (std::string pack = "RobWork")
    {
#if defined(RW_WIN)
        HKEY hKey          = 0;
        char buf[1024]     = {0};
        DWORD dwType       = 0;
        DWORD dwBufSize    = sizeof (buf);
        std::string subkey = "Software\\Kitware\\CMake\\Packages\\" + pack;
        if (RegOpenKey (HKEY_CURRENT_USER, subkey.c_str (), &hKey) == ERROR_SUCCESS) {
            dwType = REG_SZ;
            if (RegQueryValueEx (hKey, "Location", 0, &dwType, (BYTE*) buf, &dwBufSize) ==
                ERROR_SUCCESS) {
                return std::string (buf) + "\\lib\\RobWork\\rwplugins";
            }
            RegCloseKey (hKey);
        }
        return std::string ();
#else
        if (boost::filesystem::exists ("/usr/lib/")) {    // Add default plugin location

            boost::filesystem::path p ("/usr/lib");
            std::string rwpluginFolder = "";

            // Find the architecture dependendt folder containing the rwplugins folder
            // Search all files and folders
            for (boost::filesystem::directory_iterator i (p);
                 i != boost::filesystem::directory_iterator ();
                 i++) {
                // If is directory
                if (boost::filesystem::is_directory (i->path ())) {
                    rwpluginFolder = "/usr/lib/";
                    rwpluginFolder += i->path ().filename ().string ();
                    rwpluginFolder += "/RobWork/rwplugins";
                    if (boost::filesystem::exists (rwpluginFolder)) {
                        break;
                    }
                    else {
                        rwpluginFolder = "";
                    }
                }
            }
            return rwpluginFolder;
        }
        return "";

#endif
    }
};

#endif /*RW_COMMOM_OS_HPP*/
