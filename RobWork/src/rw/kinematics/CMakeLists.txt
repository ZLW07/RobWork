set(SUBSYS_NAME sdurw_kinematics)
set(SUBSYS_DESC "This is a gathering of all RobWorks kinamatics")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)


SET(FILES_CPP
  FixedFrame.cpp
  FKRange.cpp
  FKTable.cpp
  Frame.cpp
  FrameType.cpp
  FrameMap.cpp
  Kinematics.cpp
  MovableFrame.cpp
  State.cpp
  QState.cpp
  TreeState.cpp
  StateCache.cpp
  StateStructure.cpp
  StateData.cpp
  StateSetup.cpp
  FramePairMap.cpp
  StatelessData.cpp
  Stateless.cpp
)

SET(FILES_HPP
  FixedFrame.hpp
  FKRange.hpp
  FKTable.hpp
  Frame.hpp
  FrameType.hpp
  FrameMap.hpp
  Kinematics.hpp
  MovableFrame.hpp
  State.hpp
  StateCache.hpp
  QState.hpp
  TreeState.hpp
  StateStructure.hpp
  StateData.hpp
  StateSetup.hpp
  FramePairMap.hpp
  StatelessData.hpp
  Stateless.hpp
)

rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
rw_add_includes(${SUBSYS_NAME} "rw/kinematics" ${FILES_HPP})
target_link_libraries(${SUBSYS_NAME} PUBLIC sdurw_core sdurw_common sdurw_math)


target_include_directories(
    ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                          $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)

rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../kinematics.hpp)
endif() 