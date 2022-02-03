set(SUBSYS_NAME sdurw_calibration)
set(SUBSYS_DESC "Calibration of devices")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT TRUE)
set(REASON)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES xml/XmlCalibrationLoader.cpp xml/XmlCalibrationSaver.cpp Calibration.cpp FixedFrameCalibration.cpp
                  WorkCellCalibration.cpp)

    set(SRC_FILES_HPP xml/XmlCalibrationLoader.hpp xml/XmlCalibrationSaver.hpp Calibration.hpp FixedFrameCalibration.hpp
                      WorkCellCalibration.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/calibration" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
