set(SUBSYS_NAME sdurw_proximitystrategies)
set(SUBSYS_DESC "Collection of proximity strategies!")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_geometry sdurw_proximity sdurw_kinematics sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(OptionalFiles)
    if(RW_HAVE_PQP)
        list(APPEND OptionalFiles "./ProximityStrategyPQP.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyPQP.hpp")
    else()
        message(
            STATUS "sdurw_proximitystrategies: Not building PQP interface since PQP is disabled!"
        )
    endif()

    if(RW_HAVE_YAOBI)
        list(APPEND OptionalFiles "./ProximityStrategyYaobi.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyYaobi.hpp")
    else()
        message(
            STATUS
                "sdurw_proximitystrategies: Not building Yaobi interface since Yaobi is disabled!"
        )
    endif()

    if(RW_HAVE_BULLET)
        list(APPEND OptionalFiles "./ProximityStrategyBullet.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyBullet.hpp")
    else()
        message(
            STATUS
                "sdurw_proximitystrategies: Not building Bullet interface since Bullet is disabled!"
        )
    endif()

    if(RW_HAVE_FCL)
        list(APPEND OptionalFiles "./ProximityStrategyFCL.cpp")
        list(APPEND OptionalFiles_hpp "./ProximityStrategyFCL.hpp")
    else()
        message(
            STATUS "sdurw_proximitystrategies: Not building FCL interface since FCL is disabled!"
        )
    endif()

    set(SRC_FILES ${RW_PROXIMITYSTRATEGIES_USER_FILES} ${OptionalFiles}
                  ProximityStrategyFactory.cpp)

    set(SRC_FILES_HPP ${OptionalFiles_hpp} ProximityStrategyFactory.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/proximitystrategies" ${SRC_FILES_HPP})
    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC ${SUBSYS_DEPS} ${PQP_LIBRARIES} ${BULLET_LIBRARIES}
        PRIVATE ${YAOBI_LIBRARIES} ${FCL_LIBRARIES}
    )
    
    if(CMAKE_VERSION VERSION_GREATER 3.13.0)
        target_link_directories(${SUBSYS_NAME} PRIVATE ${FCL_LIBRARY_DIRS})
    else()
        link_directories(${FCL_LIBRARY_DIRS})
    endif()

    target_include_directories(${SUBSYS_NAME} PRIVATE ${FCL_INCLUDE_DIRS})
    target_include_directories(
        ${SUBSYS_NAME}
        INTERFACE $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    set(SUBSYS_NAME_PLUGIN ${SUBSYS_NAME}.rwplugin)
    set(SUBSYS_DESC_PLUGIN "Plugin providing a collection of proximity strategies!")
    set(SUBSYS_DEPS_PLUGIN ${SUBSYS_NAME})

    set(build_plugin TRUE)
    rw_subsys_option(
        build_plugin ${SUBSYS_NAME_PLUGIN} ${SUBSYS_DESC_PLUGIN} ${DEFAULT}
        DEPENDS ${SUBSYS_DEPS_PLUGIN}
        ADD_DOC
    )

    if(build_plugin)
        rw_add_plugin(${SUBSYS_NAME}.rwplugin MODULE ProximityStrategyPlugin.cpp)
        target_link_libraries(${SUBSYS_NAME}.rwplugin PRIVATE ${SUBSYS_NAME} PUBLIC sdurw)
        target_include_directories(${SUBSYS_NAME}.rwplugin PUBLIC ${FCL_INCLUDE_DIRS})
    endif()
endif()
