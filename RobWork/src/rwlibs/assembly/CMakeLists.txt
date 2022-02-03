set(SUBSYS_NAME sdurw_assembly)
set(SUBSYS_DESC "Assembly task and result formats and strategies.")
set(SUBSYS_DEPS sdurw sdurw_task sdurw_core sdurw_math)

set(DEFAULT TRUE)
set(REASON)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(
        SRC_FILES
        ./AssemblyControlResponse.cpp
        ./AssemblyControlStrategy.cpp
        ./AssemblyParameterization.cpp
        ./AssemblyRegistry.cpp
        ./AssemblyResult.cpp
        ./AssemblyState.cpp
        ./AssemblyTask.cpp
        ./CircularPiHControlStrategy.cpp
        ./CircularPiHParameterization.cpp
        ./PiHParameterization.cpp
        ./PiHStrategy.cpp
        ./SpiralParameterization.cpp
        ./SpiralStrategy.cpp
    )

    set(
        SRC_FILES_HPP
        ./AssemblyControlResponse.hpp
        ./AssemblyControlStrategy.hpp
        ./AssemblyParameterization.hpp
        ./AssemblyRegistry.hpp
        ./AssemblyResult.hpp
        ./AssemblyState.hpp
        ./AssemblyTask.hpp
        ./CircularPiHControlStrategy.hpp
        ./CircularPiHParameterization.hpp
        ./PiHParameterization.hpp
        ./PiHStrategy.hpp
        ./SpiralParameterization.hpp
        ./SpiralStrategy.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP} ${LOADER_SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/assembly" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurw PRIVATE sdurw_task)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
