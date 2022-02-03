set(TargetName qtpropertybrowser)

set(MYROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_AUTOMOC ON)
# As moc files are generated in the binary dir, tell CMake to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions("-DQT_DISABLE_DEPRECATED_BEFORE=0")
add_definitions("-DQT_NO_DEBUG")
add_definitions("-DQT_WIDGETS_LIB")
add_definitions("-DQT_GUI_LIB")
add_definitions("-DQT_CORE_LIB")

# Standard files to compile:
set(SRC_FILES_CPP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils.cpp
    # ${CMAKE_CURRENT_BINARY_DIR}/src/moc_qtpropertybrowserutils_p.cpp
    ${UIS_OUT_H}
)

set(SRC_FILES_HPP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertymanager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qteditorfactory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtvariantproperty.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qttreepropertybrowser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtbuttonpropertybrowser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtgroupboxpropertybrowser.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowserutils_p.h
)

# Resource files to rcc and compile:
if(DEFINED Qt6Core_VERSION)
    qt_add_resources(ResourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.qrc)
else()
    qt5_add_resources(ResourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/src/qtpropertybrowser.qrc)
endif()

# We create a static library named qtpropertybrowser that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
add_library(${TargetName} ${SRC_FILES_CPP} ${ResourceFiles} )
add_library(${PROJECT_PREFIX}::${TargetName} ALIAS ${TargetName})
target_include_directories(
    ${TargetName}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
              $<INSTALL_INTERFACE:${EXT_INSTALL_DIR}/qtpropertybrowser>
)
target_link_libraries(${TargetName} PUBLIC ${QT_LIBRARIES})
set_target_properties(${TargetName} PROPERTIES LINK_FLAGS -Wl,--as-needed,--no-undefined)
set_target_properties(${TargetName} PROPERTIES VERSION 1 SOVERSION 1.0)

# Install stuff
install(
    TARGETS ${TargetName}
    EXPORT ${PROJECT_PREFIX}Targets
    DESTINATION "${LIB_INSTALL_DIR}"
)
install(
    DIRECTORY "${RWS_ROOT}/ext/qtpropertybrowser/src/"
    DESTINATION "${EXT_INSTALL_DIR}/qtpropertybrowser/"
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "Qt*"
    PATTERN ".svn" EXCLUDE
    PATTERN "*.cpp" EXCLUDE
    PATTERN "*.qrc" EXCLUDE
    PATTERN "*.pri" EXCLUDE
    PATTERN "*.png" EXCLUDE
)
