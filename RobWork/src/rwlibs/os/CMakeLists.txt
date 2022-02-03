set(SUBSYS_NAME sdurw_os)
set(SUBSYS_DESC "A module with OS specifc files")
set(SUBSYS_DEPS sdurw_core)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    set(SRC_FILES_HPP rwgl.hpp glext_win32.h glext_win32_old.h)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/os" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
