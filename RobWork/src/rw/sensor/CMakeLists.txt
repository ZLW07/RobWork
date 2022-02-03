set(SUBSYS_NAME sdurw_sensor)
set(SUBSYS_DESC "This is a gathering of all RobWorks sensor systems, and models")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_geometry sdurw_kinematics)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        Camera.cpp
        CameraFirewire.cpp
        CameraListener.cpp
        CameraModel.cpp
        Contact2D.cpp
        Contact3D.cpp
        FTSensor.cpp
        FTSensorModel.cpp
        Image.cpp
        ImageUtil.cpp
        RGBDCameraModel.cpp
        Scanner.cpp
        Scanner1D.cpp
        Scanner25D.cpp
        Scanner25DModel.cpp
        Scanner2D.cpp
        Scanner2DModel.cpp
        Sensor.cpp
        SensorModel.cpp
        StereoCameraModel.cpp
        TactileArray.cpp
        TactileArrayModel.cpp
        TactileArrayUtil.cpp
    )

    set(FILES_HPP
        Camera.hpp
        CameraFirewire.hpp
        CameraListener.hpp
        CameraModel.hpp
        Contact2D.hpp
        Contact3D.hpp
        FTSensor.hpp
        FTSensorModel.hpp
        Image.hpp
        ImageUtil.hpp
        RGBDCameraModel.hpp
        Scanner.hpp
        Scanner1D.hpp
        Scanner25D.hpp
        Scanner25DModel.hpp
        Scanner2D.hpp
        Scanner2DModel.hpp
        Sensor.hpp
        SensorModel.hpp
        StereoCameraModel.hpp
        TactileArray.hpp
        TactileArrayModel.hpp
        TactileArrayUtil.hpp
    )
    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/sensor" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../sensor.hpp)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()
