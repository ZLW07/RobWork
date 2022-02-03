set(SUBSYS_NAME sdurw_simplefinger)
set(SUBSYS_DESC "A module for duing analytic geometry")
set(SUBSYS_DEPS sdurw sdurw_csg)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    rw_add_library(${SUBSYS_NAME} SimpleFinger.cpp SimpleFinger.hpp)
    target_link_libraries(${SUBSYS_NAME} PRIVATE sdurw_csg PUBLIC sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
	
    rw_add_plugin(${SUBSYS_NAME}.rwplugin MODULE SimpleFingerPlugin.cpp SimpleFingerPlugin.hpp)
    target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw)

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
