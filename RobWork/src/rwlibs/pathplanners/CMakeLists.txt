set(SUBSYS_NAME sdurw_pathplanners)
set(SUBSYS_DESC "A collection of path planners!")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_kinematics sdurw_models sdurw_trajectory sdurw_algorithms sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)

    set(
        SRC_FILES
        ./z3/Z3QToQPlanner.cpp
        ./z3/Z3Planner.cpp
        ./prm/PartialIndexTable.cpp
        ./prm/PRMPlanner.cpp
        ./rrt/RRTQToQPlanner.cpp
        ./rrt/RRTPlanner.cpp
        ./arw/ARWExpand.cpp
        ./arw/ARWPlanner.cpp
        ./arw/ARWQToQPlanner.cpp
        ./sbl/SBLExpand.cpp
        ./sbl/SBLInternal.cpp
        ./sbl/SBLSetup.cpp
        ./sbl/SBLOptions.cpp
        ./sbl/SBLPlanner.cpp
    )

    set(SRC_FILES_HPP_Z3 ./z3/Z3QToQPlanner.hpp ./z3/Z3Planner.hpp)
    set(SRC_FILES_HPP_PRM ./prm/PartialIndexTable.hpp ./prm/PRMPlanner.hpp)
    set(SRC_FILES_HPP_RRT ./rrt/RRTQToQPlanner.hpp ./rrt/RRTPlanner.hpp)
    set(SRC_FILES_HPP_ARW ./arw/ARWExpand.hpp ./arw/ARWPlanner.hpp ./arw/ARWQToQPlanner.hpp)
    set(SRC_FILES_HPP_SBL ./sbl/SBLExpand.hpp ./sbl/SBLInternal.hpp ./sbl/SBLSetup.hpp ./sbl/SBLOptions.hpp
                          ./sbl/SBLPlanner.hpp)
    set(SRC_FILES_HPP ${SRC_FILES_HPP_Z3} ${SRC_FILES_HPP_PRM} ${SRC_FILES_HPP_RRT} ${SRC_FILES_HPP_ARW}
                      ${SRC_FILES_HPP_SBL})

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathplanners/z3" ${SRC_FILES_HPP_z3})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathplanners/prm" ${SRC_FILES_HPP_PRM})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathplanners/rrt" ${SRC_FILES_HPP_RRT})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathplanners/arw" ${SRC_FILES_HPP_ARW})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathplanners/sbl" ${SRC_FILES_HPP_SBL})

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
