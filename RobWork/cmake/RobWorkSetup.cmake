# ##################################################################################################
#
# This file is for setting up a build of RobWork. It should NOT be used for including RobWork in
# other projects, for that use FindRobWork.cmake
#
# Requirements:
#
# RW_ROOT             - Must be set to the root dir of RobWork
#
# Following is a list of variables that is set by this script:
#
# ROBWORK_INCLUDE_DIR - Where to find RobWork include sub-directory. ROBWORK_LIBRARIES   - List of
# libraries when using RobWork (includes all libraries that RobWork depends on). ROBWORK_LIBARY_DIRS
# - List of directories where libraries of RobWork are located.
#

#
# Check if RW_ROOT path are setup correctly
#
include(ExternalProject)
find_file(RW_ROOT_PATH_TEST RobWorkSetup.cmake ${RW_ROOT}/cmake NO_DEFAULT_PATH)
if(NOT RW_ROOT_PATH_TEST)
    message(
        SEND_ERROR
            "RobWork: Path to RobWork root (RW_ROOT) is incorrectly setup! \nRW_ROOT == ${RW_ROOT}"
    )
endif()
message(STATUS "RobWork: ROOT - ${RW_ROOT}")

#
# Setup the default include and library dirs for RobWork
#
# INCLUDE("${RW_ROOT}/build/RobWorkConfig${CMAKE_BUILD_TYPE}.cmake")

# ##################################################################################################
# DEPENDENCIES - REQUIRED Check for all dependencies, this adds LIBRARY_DIRS and include dirs that
# the configuration depends on
#
set(ROBWORK_LIBRARIES_INTERNAL)
#
# some of the FIND_PACKAGE modules are located in the build directory
#
set(CMAKE_MODULE_PATH ${RW_ROOT}/cmake/Modules ${CMAKE_MODULE_PATH})

#
# We need the Boost package and some of its components. Test libraries are optional and can be
# compiled from header instead.
#

if(NOT DEFINED USE_BOOST_STATIC)
    set(USE_BOOST_STATIC OFF)
    if(DEFINED WIN32)
        set(USE_BOOST_STATIC ON)
    endif()
endif()

set(Boost_USE_STATIC_LIBS ${USE_BOOST_STATIC})
unset(Boost_FIND_QUIETLY)

if(${RW_BUILD_TYPE} STREQUAL "release")
    set(Boost_USE_DEBUG_LIBS OFF) # ignore debug libs and
    set(Boost_USE_RELEASE_LIBS ON) # only find release libs
endif()

# hunter_add_package(Boost COMPONENTS filesystem serialization system thread program_options)
find_package(
    Boost QUIET
    COMPONENTS filesystem serialization system thread program_options date_time
    CONFIG
    HINTS "C:\\local"
)
if(NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread program_options date_time)
endif()
if(TARGET Boost::headers)
    set(Boost_LIBRARIES ${Boost_LIBRARIES} Boost::headers)
    get_target_property(Boost_INCLUDE_DIR Boost::headers INTERFACE_INCLUDE_DIRECTORIES)
endif()
if(NOT DEFINED Boost_LIBRARY_DIRS OR "${Boost_LIBRARY_DIRS}" STREQUAL "")
    rw_get_lib_from_target(Boost::filesystem Boost_LIBRARY_DIRS)
    get_filename_component(Boost_LIBRARY_DIRS ${Boost_LIBRARY_DIRS} DIRECTORY)
endif()

message(
    STATUS
        "RobWork: Boost version ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION}.${Boost_SUBMINOR_VERSION} found!"
)

enable_language(CXX)

# ##################################################################################################
# DEPENDENCIES - OPTIONAL these dependencies are optional, which is the user can switch off modules

#
# For some libs we need the opengl package, though it is OPTIONAL
#
find_package(OpenGL)
include(CMakeDependentOption)

#
# for some libs we need the glut package, this is an optional dependency
#
set(RW_HAVE_GLUT False)
set(FreeGLUT_FOUND False)
find_package(GLUT QUIET)

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(NOT GLUT_FOUND) # Check if free glut exsist
        find_package(FreeGLUT QUIET)
        if(FreeGLUT_FOUND)
            set(GLUT_glut_LIBRARY FreeGLUT::freeglut_static)
            set(GLUT_FOUND ${FreeGLUT_FOUND})
        endif()
    endif()
endif()

if(OPENGL_FOUND AND GLUT_FOUND)
    set(RW_HAVE_GLUT True)
    message(STATUS "RobWork: OpenGL and GLUT ENABLED! FOUND!")
else()
    set(GLUT_glut_LIBRARY "")
    message(STATUS "RobWork: OpenGL and GLUT NOT FOUND! code disabled!")
endif()

#
# For some of the xml parsing we need xerces, though it is OPTIONAL
#
set(RW_HAVE_XERCES False)
find_package(XercesC QUIET)
if(XERCESC_FOUND)
    set(RW_HAVE_XERCES True)
    message(STATUS "RobWork: XERCES ENABLED! FOUND!")
else()
    message(
        STATUS
            "RobWork: Xerces NOT FOUND! Xerces code disabled! (Check if XERCESC_ROOT or XERCESC_INCLUDE_DIR and XERCESC_LIB_DIR is set correctly if you need it)!"
    )
endif()

#
# If the user wants to use yaobi then search for it, OPTIONAL
#
set(RW_HAVE_YAOBI False)
cmake_dependent_option(
    RW_USE_YAOBI
    "Set to ON to include Yaobi support.
                Set YAOBI_INCLUDE_DIR and YAOBI_LIBRARY_DIR
                to specify your own YAOBI else RobWork YAOBI will
                be used!"
    ON
    "NOT RW_DISABLE_YAOBI"
    OFF
)
if(RW_USE_YAOBI)
    find_package(Yaobi QUIET)
    if(YAOBI_FOUND)
        message(STATUS "RobWork: Yaobi ENABLED! FOUND!")
        set(RW_HAVE_YAOBI True)
        set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${YAOBI_LIBRARIES})
    else()

        set(RW_ENABLE_INTERNAL_YAOBI_TARGET ON)
        message(STATUS "RobWork: Yaobi ENABLED! NOT FOUND! Using RobWork native Yaobi.")
        set(YAOBI_INCLUDE_DIR "${RW_ROOT}/ext/rwyaobi/include")
        set(YAOBI_LIBRARIES "RW::yaobi")
        set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} ${YAOBI_LIBRARIES})
        set(YAOBI_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
        set(RW_HAVE_YAOBI True)

    endif()
else()
    message(STATUS "RobWork: Yaobi DISABLED!")
    set(YAOBI_INCLUDE_DIR "")
endif()

#
# If the user wants to use PQP then search for it or use the default
#
set(RW_HAVE_PQP False)
cmake_dependent_option(
    RW_USE_PQP "Set to ON to include PQP support.
                    RobWork PQP will allways be used!" ON "NOT RW_DISABLE_PQP" OFF
)
if(RW_USE_PQP)
    set(RW_ENABLE_INTERNAL_PQP_TARGET ON)
    message(STATUS "RobWork: PQP ENABLED! Using RobWork native PQP.")
    set(PQP_INCLUDE_DIR "${RW_ROOT}/ext/rwpqp")
    set(PQP_LIBRARIES "pqp")
    set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} ${PQP_LIBRARIES})
    set(PQP_LIBRARY_DIRS ${RW_LIBRARY_OUT_DIR})
    set(RW_HAVE_PQP True)
else()
    message(STATUS "RobWork: PQP DISABLED!")
    set(PQP_INCLUDE_DIR "")
endif()

#
# If the user wants to use FCL then search for it, OPTIONAL
#
set(RW_HAVE_FCL False)
cmake_dependent_option(RW_USE_FCL "Set to ON to include FCL support." ON "NOT RW_DISABLE_FCL" OFF)
if(RW_USE_FCL)
    find_package(FCL QUIET)
    if(FCL_FOUND)
        message(STATUS "RobWork: Native FCL installation FOUND! - version ${FCL_VERSION}")
        set(RW_HAVE_FCL True)
        set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${FCL_LIBRARIES})
    else()
        message(STATUS "RobWork: FCL not foun, and is DISABLED!")
    endif()
else()
    message(STATUS "RobWork: FCL DISABLED!")
endif()

find_package(Eigen3 QUIET)
if(EIGEN3_FOUND)
    message(STATUS "RobWork: EIGEN3 installation FOUND! - version ${EIGEN3_VERSION}")
else()
    hunter_add_package(Eigen)
    find_package(Eigen3 REQUIRED)
    message(STATUS "RobWork: EIGEN3 installed through Hunter! - version ${EIGEN3_VERSION}")
endif()

#
# Find Qhull
#
find_package(Qhull QUIET MODULE)
if(Qhull_FOUND)
    message(STATUS "RobWork: Qhull found")
    if(NOT DEFINED QHULL_ROOT)
        set(QHULL_ROOT "${QHULL_INCLUDE_DIRS}/..")
    endif()
else()
    hunter_add_package(qhull)
    find_package(Qhull REQUIRED)
endif()

# CSGJS
option(RW_USE_CSGJS "Set to ON to use ext CSGJS." ON)
if(RW_USE_CSGJS)
    message(STATUS "Using CsgJs.")
    set(CSGJS_INCLUDE_DIRS "${RW_ROOT}/ext/csgjs/src")
    set(CSGJS_LIBRARIES "sdurw_csgjs")
    set(CSGJS_DEFINITIONS "")
    set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} ${CSGJS_LIBRARIES})
endif()

find_package(Bullet QUIET)
set(RW_HAVE_BULLET FALSE)
if(BULLET_FOUND)
    message("Bullet found! ${BULLET_INCLUDE_DIRS}")
    message("Bullet libs: ${BULLET_LIBRARIES}")
    set(BULLET_LIBRARIES "")
    set(BULLET_INCLUDE_DIRS "")
else()
    set(BULLET_INCLUDE_DIRS "")
endif()

#
# If the user wants to use LUA then search for it or use the default
#
set(RW_HAVE_LUA False)
set(RW_HAVE_SWIG False)

find_package(SWIG 3.0.0 QUIET) # At least SWIG 3 to support C++11
if(SWIG_FOUND)
    set(RW_HAVE_SWIG true)
else()

endif()

cmake_dependent_option(
    RW_USE_LUA "Set to ON to include LUA support. " ON "SWIG_FOUND;NOT RW_DISABLE_LUA" OFF
)

if(RW_USE_LUA)

    find_package(Lua 5.4 QUIET)
    if(NOT LUA_FOUND)
        find_package(Lua QUIET)
    endif()
    if(LUA_FOUND)
        if(LUA_VERSION_MAJOR GREATER "5" OR LUA_VERSION_MINOR GREATER "0")
            message(STATUS "RobWork: lua ${LUA_VERSION_STRING} FOUND!")
            set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${LUA_LIBRARIES})
        endif()
    else()
        message(STATUS "RobWork: Lua not FOUND")
    endif()

    if(SWIG_FOUND AND LUA_FOUND)
        message(STATUS "RobWork: LUA ENABLED! Both SWIG ${SWIG_VERSION} and Lua FOUND!")
        set(RW_HAVE_LUA true)
        set(RW_LUA_LIBS
            sdurw_core_lua_s
            sdurw_common_lua_s
            sdurw_math_lua_s
            sdurw_lua_s
            sdurw_assembly_lua_s
            sdurw_control_lua_s
            sdurw_pathoptimization_lua_s
            sdurw_pathplanners_lua_s
            sdurw_proximitystrategies_lua_s
            sdurw_simulation_lua_s
            sdurw_task_lua_s
        )

        set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} ${RW_LUA_LIBS})
    else()
        set(RW_HAVE_LUA False)
        set(LUA_INCLUDE_DIR)
        message(STATUS "RobWork: Lua DISABLED! Since SWIG 3+ or LUA was NOT FOUND!")
    endif()
else()
    message(STATUS "RobWork: LUA DISABLED!")
    set(LUA_INCLUDE_DIR)
endif()

if(RW_BUILD_SANDBOX)
    message(STATUS "RobWork: Sandbox ENABLED!")
    set(SANDBOX_LIB "sdurw_sandbox")
    set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} ${SANDBOX_LIB})
else()
    message(STATUS "RobWork: Sandbox DISABLED!")
endif()

#
# If the user wants to use the softbody package
#

option(RW_BUILD_SOFTBODY "Set when you want to build softbody module" OFF)
if(RW_BUILD_SOFTBODY)
    message(STATUS "RobWork: Softbody ENABLED!")
    # Make sure to have set environment variable, e.g. in .bashrc export
    # IPOPT_HOME=/home/arf/Documents/Ipopt-3.10.3
    find_package(MUMPS REQUIRED)
    find_package(IPOPT REQUIRED)
    set(SOFTBODY_LIBRARY_DIRS ${IPOPT_LIBRARY_DIRS} ${MUMPS_LIBRARY_DIRS})
    set(SOFTBODY_LIB sdurw_softbody ${MUMPS_LIBRARIES} ${IPOPT_LIBRARIES})
    set(SOFTBODY_INCLUDE_DIRS ${MUMPS_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})

    set(ROBWORK_LIBRARIES_INTERNAL ${ROBWORK_LIBRARIES_INTERNAL} sdurw_softbody)
    set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${MUMPS_LIBRARIES}
                                   ${IPOPT_LIBRARIES}
    )
else()
    message(STATUS "RobWork: Softbody DISABLED!")
endif()

#
# If the user wants to use the Assimp package then search for it or build internal Assimp. Set
# RW_DISABLE_ASSIMP to ON to disable Assimp completely.
#

set(RW_HAVE_ASSIMP FALSE)

# Make option for user to disable Assimp
cmake_dependent_option(
    RW_USE_ASSIMP
    "Set to ON to include Assimp support.
                Set ASSIMP_INCLUDE_DIR and ASSIMP_LIBRARY_DIR
                to specify your own Assimp else RobWork Assimp will
                be used!"
    ON
    "NOT RW_DISABLE_ASSIMP"
    OFF
)

set(ASSIMP_INCLUDE_DIRS "")
set(ASSIMP_LIBRARIES "")
if(RW_USE_ASSIMP)
    # Now try to find Assimp
    find_package(Assimp QUIET)
    if(ASSIMP_FOUND)
        message(STATUS "RobWork: Native Assimp installation FOUND!")
        set(RW_HAVE_ASSIMP TRUE)
        set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${ASSIMP_LIBRARIES})
    else()
        set(ASSIMP_LIBRARIES)
        set(ASSIMP_INCLUDE_DIRS)
        message(STATUS "RobWork: Assimp not found and is now DISABLED!")
    endif()
else()
    message(STATUS "RobWork: Assimp DISABLED!")
endif()

# Find Python - prefer version 3 (should be done before GTest) set(Python_ADDITIONAL_VERSIONS 3.8)
if(CMAKE_VERSION VERSION_LESS "3.12")
    find_package(PythonInterp 3 QUIET)
    find_package(PythonLibs 3 QUIET)

    if(NOT PYTHONINTERP_FOUND)
        find_package(PythonInterp QUIET)
    endif()
    if(NOT PYTHONLIBS_FOUND)
        find_package(PythonLibs QUIET)
    endif()

    if(PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
        if(NOT ((PYTHONLIBS_VERSION_MAJOR STREQUAL PYTHON_VERSION_MAJOR)
                AND (PYTHONLIBS_VERSION_MINOR STREQUAL PYTHON_VERSION_MINOR))
        )
            string(ASCII 27 Esc)
            message(
                WARNING
                    "${Esc}[33mMatching Versions of python intepretor and python library NOT FOUND. \r"
                    "Found versions are python libs ${PYTHONLIBS_VERSION_STRING} and python intepretor ${PYTHON_VERSION_STRING}. \n"
                    "This can be because you haven't installed python${PYTHON_VERSION_MAJOR}-dev package\n${Esc}[m"
            )
        endif()
    endif()
else()
    find_package(Python3 QUIET COMPONENTS Interpreter Development)
    if(Python3_FOUND)
        set(PYTHONINTERP_FOUND ${Python3_Interpreter_FOUND})
        set(PYTHONLIBS_FOUND ${Python3_Development_FOUND})
        set(PYTHON_LIBRARIES ${Python3_LIBRARIES})
        set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
        set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})
    endif()
endif()

if(PYTHONINTERP_FOUND)
    message(STATUS "Found Python interpreter ${PYTHON_VERSION_STRING}")
endif()
if(PYTHONLIBS_FOUND)
    message(STATUS "Found Python libraries ${PYTHONLIBS_VERSION_STRING}")
endif()
if(PYTHON_LIBRARY STREQUAL "NOTFOUND")
    set(PYTHON_LIBRARY "")
endif()

#
# Use numpy for swing bindings if available
#
if(SWIG_FOUND)
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c
                "try: \n\timport numpy; print(\"ON\");\nexcept ImportError:\n\tprint(\"OFF\");"
        OUTPUT_VARIABLE RW_USE_NUMPY
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(RW_USE_NUMPY)
        message(STATUS "RobWork is compiled with Numpy")
        execute_process(
            COMMAND python3 -c "import numpy; print(numpy.__file__);"
            OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        get_filename_component(NUMPY_INCLUDE_DIR "${NUMPY_INCLUDE_DIR}" DIRECTORY)
        set(NUMPY_INCLUDE_DIR "${NUMPY_INCLUDE_DIR}/core/include")
        message(STATUS "numpy found at: ${NUMPY_INCLUDE_DIR}")
    else()
        message(STATUS "RobWork can't find Numpy.")
    endif()
endif()

#
# If the user wants to use the Google Test package then search for it. Set RW_DISABLE_GTEST to ON to
# disable Google Test completely.
#
# Make option for user to disable Google Test
cmake_dependent_option(
    RW_USE_GTEST
    "Set to ON to include Google Test support. Set GTEST_ROOT or GTEST_SOURCE to specify your own Google Test installation."
    ON
    "NOT RW_DISABLE_GTEST"
    OFF
)
set(RW_HAVE_GTEST FALSE)
set(RW_GTEST_FROM_HUNTER FALSE)
if(RW_USE_GTEST)
    # Now try to find Google Test
    set(gtest_force_shared_crt
        ON
        CACHE BOOL "Use /MD on Windows systems."
    )
    find_package(GTest QUIET)
    if(GTEST_FOUND)
        set(GTEST_SHARED_LIBS ${BUILD_SHARED_LIBS})
        message(STATUS "RobWork: Google Test installation FOUND!")
        set(RW_HAVE_GTEST TRUE)
        if(TARGET ${GTEST_LIBRARY} AND TARGET ${GTEST_MAIN_LIBRARY})
            add_library(RW::${GTEST_LIBRARY} ALIAS ${GTEST_LIBRARY})
            add_library(RW::${GTEST_MAIN_LIBRARY} ALIAS ${GTEST_MAIN_LIBRARY})
            set(RW_ENABLE_INTERNAL_GTEST_TARGET ON)
            set(GTEST_BOTH_LIBRARIES RW::${GTEST_MAIN_LIBRARY} RW::${GTEST_LIBRARY})
        endif()
    else()
        hunter_add_package(GTest)
        find_package(GTest REQUIRED)
        set(RW_GTEST_FROM_HUNTER TRUE)
    endif()
else()
    message(STATUS "RobWork: Google Test DISABLED!")
endif()

# Mathematica
cmake_dependent_option(
    RW_USE_MATHEMATICA "Set to ON to include Mathematica support." ON "RW_ENABLE_MATHEMATICA" OFF
)
if(RW_USE_MATHEMATICA)
    find_package(Mathematica QUIET)
    if(Mathematica_WSTP_FOUND)
        message(STATUS "RobWork: Mathematica WSTP installation FOUND!")
        foreach(math_lib_dirs ${Mathematica_LIBRARY_DIRS})
            if(${math_lib_dirs} MATCHES "/Libraries/")
                set(UUID_LIB_DIR ${math_lib_dirs})
            endif()
        endforeach()
        if(DEFINED UUID_LIB_DIR)
            set(Mathematica_WSTP_LIBRARIES ${Mathematica_WSTP_LIBRARIES} ${UUID_LIB_DIR}/libuuid.a)
        endif()
        set(RW_MATHEMATICA_LIB sdurw_mathematica)
        set(RW_HAVE_MATHEMATICA TRUE)

        set(ROBWORK_LIBRARIES_EXTERNAL ${ROBWORK_LIBRARIES_EXTERNAL} ${RW_MATHEMATICA_LIB})
    else()
        message(STATUS "RobWork: Mathematica NOT FOUND!")
        set(RW_HAVE_MATHEMATICA FALSE)
    endif()
else()
    message(STATUS "RobWork: Mathematica DISABLED!")
    set(RW_HAVE_MATHEMATICA FALSE)
endif()

# ##################################################################################################
# COMPILER FLAGS AND MACRO SETUP
#

#
# Enable the RW_ASSERT() macro.
#
option(RW_ENABLE_ASSERT "Enables RW_ASSERT macro: on|off" ON)
if(RW_ENABLE_ASSERT)
    message(STATUS "RobWork: RW_ASSERT enabled.")
    add_definitions(-DRW_ENABLE_ASSERT)
else()
    message(STATUS "RobWork: RW_ASSERT disabled.")
endif()

rw_is_release(IS_RELEASE)
#
# Set extra compiler flags. The user should be able to change this
#

if("${RW_C_FLAGS}" STREQUAL "")
    # GCC and MinGW
    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        # Necessary Linux-GCC flag
        if(DEFINED UNIX)
            set(RW_C_FLAGS_TMP "${RW_C_FLAGS_TMP} -fPIC")
        endif()
    endif()

    if(DEFINED RW_C_FLAGS_EXTRA)
        set(RW_C_FLAGS_TMP "${RW_C_FLAGS_TMP} ${RW_C_FLAGS_EXTRA}")
    endif()

    set(RW_C_FLAGS
        "${RW_C_FLAGS_TMP}"
        CACHE STRING "Change this to force using your own
					  flags and not those of RobWork" FORCE
    )
endif()

if("${RW_CXX_FLAGS}" STREQUAL "")
    # GCC and MinGW
    if((CMAKE_COMPILER_IS_GNUCXX)
       OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
       OR (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    )
        # Turn off annoying GCC warnings
        set(RW_CXX_FLAGS_TMP "-Wall -Wno-strict-aliasing -Wno-unused-function")
        if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang"))
            set(RW_CXX_FLAGS_TMP "-Wall -Wno-strict-aliasing -Wno-unused-function")
        endif()

        # Necessary Linux-GCC flag
        if(DEFINED UNIX)
            set(RW_CXX_FLAGS_TMP "${RW_CXX_FLAGS_TMP} -fPIC")
        endif()
    endif()

    # Setup crucial MSVC flags, without these RobWork does not compile
    if(DEFINED MSVC)
        set(RW_CXX_FLAGS_TMP "-EHa -bigobj /MP")
    endif()

    # Set C++11 standard (except if user has specified this explicitly in the RW_CXX_FLAGS_EXTRA
    # variable).
    set(RW_CXX_FLAGS_SET_STD FALSE)
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "6.1.0" OR CMAKE_CXX_COMPILER_ID STREQUAL
                                                              "AppleClang"
        ) # from GNU 6.1 gnu++14 should be the
            # default
            set(RW_CXX_FLAGS_SET_STD TRUE)
            foreach(flag ${RW_CXX_FLAGS_EXTRA})
                string(REGEX MATCH ".*-std=.*" flag ${flag})
                if(flag)
                    set(RW_CXX_FLAGS_SET_STD FALSE)
                endif()
            endforeach()
        endif()
    endif()
    if(RW_CXX_FLAGS_SET_STD)
        set(RW_CXX_FLAGS_TMP "${RW_CXX_FLAGS_TMP} -std=c++11")
    endif()

    if(DEFINED RW_CXX_FLAGS_EXTRA)
        set(RW_CXX_FLAGS_TMP "${RW_CXX_FLAGS_TMP} ${RW_CXX_FLAGS_EXTRA}")
    endif()

    set(RW_CXX_FLAGS
        "${RW_CXX_FLAGS_TMP}"
        CACHE STRING "Change this to force using your own
					  flags and not those of RobWork" FORCE
    )
endif()

#
# Enable the use of OMP definitions.
#
option(RW_ENABLE_OMP "Enables use of OpenMP #pragmas: on|off" ON)
if(RW_ENABLE_OMP)
    message(STATUS "RobWork: OpenMP enabled.")
    find_package(OpenMP QUIET)
    if(${CMAKE_VERSION} VERSION_LESS "3.9")
        if(OPENMP_FOUND)
            if(${CMAKE_VERSION} VERSION_LESS "3.7")
                message(STATUS "RobWork: OpenMP CXX FOUND!")
            else()
                message(
                    STATUS "RobWork: OpenMP CXX FOUND! - Specification date ${OpenMP_CXX_SPEC_DATE}"
                )
            endif()
            set(RW_HAVE_OMP TRUE)
            set(RW_CXX_FLAGS "${RW_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        else()
            message(STATUS "RobWork: OpenMP CXX NOT FOUND!")
            set(RW_HAVE_OMP FALSE)
        endif()
    else() # CMake 3.9 and newer
        if(OpenMP_CXX_FOUND)
            message(
                STATUS
                    "RobWork: OpenMP ${OpenMP_CXX_VERSION} CXX FOUND! - Specification date ${OpenMP_CXX_SPEC_DATE}"
            )
            set(RW_HAVE_OMP TRUE)
            set(RW_CXX_FLAGS "${RW_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
            # Todo: use OpenMP_CXX_LIB_NAMES, OpenMP_CXX_LIBRARY and/or OpenMP_CXX_LIBRARIES ?
        else()
            message(STATUS "RobWork: OpenMP CXX NOT FOUND!")
            set(RW_HAVE_OMP FALSE)
        endif()
    endif()
else()
    message(STATUS "RobWork: OpenMP disabled.")
endif()

if("${RW_DEFINITIONS}" STREQUAL "")
    # GCC and MinGW
    if((CMAKE_COMPILER_IS_GNUCXX) OR (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
        set(RW_DEFINITIONS_TMP)

        if(IS_RELEASE)
            list(APPEND RW_DEFINITIONS_TMP "-DBOOST_DISABLE_ASSERTS")
        endif()

        if(DEFINED MINGW AND AMD64)
            list(APPEND RW_DEFINITIONS_TMP "-DBOOST_USE_WINDOWS_H")
        endif()
    endif()

    # Setup crucial MSVC flags, without these RobWork does not compile
    if(DEFINED MSVC)
        set(RW_DEFINITIONS_TMP # Remove the min()/max() macros or else RobWork won't compile.
            "-DNOMINMAX"
            # Without this define for boost-bindings we can't link with lapack.
            "-DBIND_FORTRAN_LOWERCASE_UNDERSCORE"
            "-DWIN32_LEAN_AND_MEAN"
            "-D_WIN32_WINNT=0x0501"
            "-D_SCL_SECURE_NO_WARNINGS"
            "-D_CRT_SECURE_NO_WARNINGS"
            "-D_CRT_SECURE_NO_DEPRECATE"
        )

        if(BOOST_TEST_NO_LIB)
            list(APPEND RW_DEFINITIONS_TMP "-DBOOST_TEST_NO_LIB")
        endif()
        if(BOOST_ALL_DYN_LINK)
            list(APPEND RW_DEFINITIONS_TMP "-DBOOST_ALL_DYN_LINK")
        endif()

        # Current issues addressed for MSVC 64 bit: - MSVC 64-bit does not support __asm keyword
        # which is used by default in Yaobi. Therefore, we only define YAOBI_USE_FCOMI in
        # ext/yaobi/yaobi_settings.h for 32 bit architectures.
        if(AMD64)
            list(APPEND RW_DEFINITIONS_TMP "-DMSVC_AMD64")
        endif()
    endif()

    # Set necessary options for Win32 environments if static version of Xerces is used
    if(RW_HAVE_XERCES AND XERCES_USE_STATIC_LIBS)
        list(APPEND RW_DEFINITIONS_TMP "-DXERCES_STATIC_LIBRARY")
    endif()

    if(DEFINED RW_DEFINITIONS_EXTRA)
        set(RW_DEFINITIONS_EXTRA_TMP "${RW_DEFINITIONS_EXTRA_TMP} ${RW_DEFINITIONS_EXTRA_EXTRA}")
    endif()

    set(RW_DEFINITIONS
        "${RW_DEFINITIONS_TMP}"
        CACHE STRING "Change this to force using your own
					  flags and not those of RobWork" FORCE
    )
endif()

if("${RW_CXX_FLAGS}" STREQUAL "")
    message(
        WARNING
            "Something might be wrong. No CXX FLAGS have been specified. You may be using an unsupported compiler!!"
    )
endif()

add_definitions(${RW_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${RW_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${RW_C_FLAGS}")
message(STATUS "RobWork: RW C flags: ${RW_C_FLAGS}")
message(STATUS "RobWork: RW CXX flags: ${RW_CXX_FLAGS}")
message(STATUS "RobWork: RW definitions: ${RW_DEFINITIONS}")

#
# Set extra linker flags. The user should be able to change this
#
if(NOT DEFINED RW_LINKER_FLAGS)
    # Set necessary linker options for Win32 environments if static version of Xerces is used
    if(RW_HAVE_XERCES)
        if(MSVC AND XERCES_USE_STATIC_LIBS)
            if(NOT IS_RELEASE)
                set(RW_LINKER_FLAGS "/NODEFAULTLIB:LIBCMTD")
            else()
                set(RW_LINKER_FLAGS "/NODEFAULTLIB:LIBCMT")
            endif()
        endif()
    endif()
endif()
set(RW_BUILD_WITH_LINKER_FLAGS "${RW_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
    CACHE STRING "" FORCE
)
set(CMAKE_MODULE_LINKER_FLAGS
    "${CMAKE_MODULE_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
    CACHE STRING "" FORCE
)
if(WIN32)
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} ${RW_LINKER_FLAGS}"
        CACHE STRING "" FORCE
    )
endif()
message(STATUS "RobWork: RW linker flags: ${RW_LINKER_FLAGS}")

# MESSAGE(" ${Boost_MAJOR_VERSION} ${Boost_MINOR_VERSION} ")
if(${Boost_MINOR_VERSION} VERSION_LESS 41)
    # proerty tree is not included in earlier versions 1.41 of boost
    message(
        FATAL_ERROR
            "RobWork: Boost ${Boost_MAJOR_VERSION}.${Boost_MINOR_VERSION} found, no support for property_tree. Please choose Boost version 1.41 or newer!"
    )
endif()

if(${Boost_MINOR_VERSION} VERSION_LESS 44)
    add_definitions("-DBOOST_FILESYSTEM_VERSION=2")
elseif(${Boost_MINOR_VERSION} VERSION_LESS 46) # version 3 is the default for Boost 1.46 and later
    add_definitions("-DBOOST_FILESYSTEM_VERSION=3")
endif()

if(MSVC)
    add_definitions("-DEIGEN_DONT_ALIGN_STATICALLY=1")
endif()

# ##################################################################################################
# SETTING UP VARS here we setup the output variables
#

#
# The include dirs
#
set(ROBWORK_INCLUDE_DIR
    ${EIGEN3_INCLUDE_DIR}
    ${SOFTBODY_INCLUDE_DIRS}
    ${ADDITIONAL_BOOST_BINDINGS}
    ${RW_ROOT}/src
    ${OPENGL_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${XERCESC_INCLUDE_DIR}
    ${YAOBI_INCLUDE_DIR}
    ${PQP_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${QHULL_INCLUDE_DIRS}
    ${CSGJS_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${Mathematica_WSTP_INCLUDE_DIR}
)

#
# The library dirs
#
set(ROBWORK_LIBRARY_DIRS
    ${SOFTBODY_LIBRARY_DIRS}
    ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}
    ${RW_CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
    ${Boost_LIBRARY_DIRS}
    ${XERCESC_LIB_DIR}
    ${YAOBI_LIBRARY_DIRS}
    ${PQP_LIBRARY_DIRS}
    ${FCL_LIBRARY_DIRS}
    ${LUA_LIBRARY_DIRS}
    ${BULLET_LIBRARY_DIRS}
    ${ASSIMP_LIBRARY_DIRS}
    ${Mathematica_WSTP_INCLUDE_DIR}
)

#
# Setup the Library List here. We need to make sure the correct order is maintained which is crucial
# for some compilers.
#
set(ROBWORK_LIBRARIES_EXTERNAL
    ${ROBWORK_LIBRARIES_EXTERNAL}
    ${OPENGL_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${BLAS_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${Mathematica_WSTP_LIBRARIES}
    ${Boost_LIBRARIES}
)

set(ROBWORK_LIBRARIES_INTERNAL
    ${ROBWORK_LIBRARIES_INTERNAL}
    sdurw_algorithms
    sdurw_pathplanners
    sdurw_pathoptimization
    sdurw_simulation
    sdurw_opengl
    sdurw_assembly
    sdurw_task
    sdurw_calibration
    sdurw_csg
    sdurw_control
    sdurw_proximitystrategies
    sdurw
    sdurw_core
    sdurw_common
    sdurw_math
    sdurw_plugin
    sdurw_kinematics
    sdurw_geometry
    sdurw_graphics
    sdurw_graspplanning
    sdurw_invkin
    sdurw_kinematics
    sdurw_loaders
    sdurw_pathplanning
    sdurw_trajectory
    sdurw_sensor
    sdurw_models
    sdurw_proximity
)

set(ROBWORK_LIBRARIES)
foreach(l ${ROBWORK_LIBRARIES_EXTERNAL})
    unset(tmp CACHE)
    if(NOT ("${l}" STREQUAL "debug" OR "${l}" STREQUAL "optimized"))
        find_library(
            tmp ${l}
            PATHS ${ROBWORK_LIBRARY_DIRS}
            NO_DEFAULT_PATH
        )
        if(tmp)
            list(APPEND ROBWORK_LIBRARIES ${tmp})
        else()
            list(APPEND ROBWORK_LIBRARIES ${l})
        endif()
    endif()
endforeach(l)

set(ROBWORK_LIBRARIES ${ROBWORK_LIBRARIES_INTERNAL} ${ROBWORK_LIBRARIES})
