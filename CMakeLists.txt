# Test CMake version
cmake_minimum_required(VERSION 3.10  )

# The name of the project.
project(RobWorkProject)

set(RWP_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT ${ROOT}/RobWork/)
set(RWS_ROOT ${ROOT}/RobWorkStudio/)
set(RWSIM_ROOT ${ROOT}/RobWorkSim/)

option(WITH_RWS "Set true to enable compilation of RobWorkStudio!" TRUE)
option(WITH_RWSIM "Set true to enable compilation of RobWorkSim!" TRUE)

unset(SLIB_COMPILE_BUFFER CACHE) 

add_subdirectory(RobWork)
if(WITH_RWS)
    add_subdirectory(RobWorkStudio)
endif()
if(WITH_RWSIM)

    find_package(
        Qt6
        COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets
        QUIET
    )
    if(Qt6Core_FOUND
    AND Qt6Gui_FOUND
    AND Qt6Widgets_FOUND
    AND Qt6OpenGL_FOUND
    AND Qt6OpenGLWidgets_FOUND
    )
        set(QT_LIBRARIES ${Qt6Core_LIBRARIES} ${Qt6Gui_LIBRARIES} ${Qt6Widgets_LIBRARIES}
                        ${Qt6OpenGL_LIBRARIES} ${Qt6OpenGLWidgets_LIBRARIES}
        )
    else()
        find_package(Qt5Core 5.5.1 REQUIRED)
        find_package(Qt5Gui 5.5.1 REQUIRED)
        find_package(Qt5Widgets 5.5.1 REQUIRED)
        find_package(Qt5OpenGL 5.5.1 REQUIRED)
        get_target_property(QT_UIC_EXECUTABLE Qt5::uic LOCATION)
        set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Widgets_LIBRARIES}
                        ${Qt5OpenGL_LIBRARIES}
        )
        set(QT_INCLUDES ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
                        ${Qt5OpenGL_INCLUDE_DIRS}
        )
    endif()

    add_subdirectory(RobWorkSim)
endif()

# #######################################################33 now configure documentation
configure_file(${ROOT}/doc/Doxyfile.in ${ROOT}/doc/Doxyfile)

find_program(doc_executable doxygen HINTS ${DOXYGEN_PATH} $ENV{DOXYGEN_PATH})
if(doc_executable)
    add_custom_target(sdurwdoc ${doc_executable} WORKING_DIRECTORY ${ROOT}/doc)
endif()
