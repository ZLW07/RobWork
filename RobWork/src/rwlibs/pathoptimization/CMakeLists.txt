set(SUBSYS_NAME sdurw_pathoptimization)
set(SUBSYS_DESC "Path optimization algorithms")
set(SUBSYS_DEPS sdurw_proximitystrategies sdurw sdurw_core sdurw_common sdurw_math sdurw_kinematics sdurw_models sdurw_proximity)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES ./clearance/ClearanceOptimizer.cpp ./clearance/ClearanceCalculator.cpp
                  ./clearance/MinimumClearanceCalculator.cpp ./pathlength/PathLengthOptimizer.cpp)

    set(SRC_FILES_HPP ./clearance/ClearanceOptimizer.hpp ./clearance/ClearanceCalculator.hpp
                      ./clearance/MinimumClearanceCalculator.hpp ./pathlength/PathLengthOptimizer.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/pathoptimization" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PRIVATE sdurw_proximitystrategies PUBLIC sdurw)
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
