set(SUBSYS_NAME sdurw_plugin)
set(SUBSYS_DESC
    "This is the library used for creating plugins for robwork"
)
set(SUBSYS_DEPS sdurw_core)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

SET(FILES_CPP
  DynamicLibraryLoader.cpp
  DynamicLibraryLoaderBase.cpp
  PluginFactory.cpp
  PluginFactoryBase.cpp
  PluginRepository.cpp
)

SET(FILES_HPP
  DynamicLibraryLoader.hpp
  DynamicLibraryLoaderBase.hpp
  PluginFactory.hpp
  PluginFactoryBase.hpp
  PluginRepository.hpp
  FactoryMacro.hpp
)

rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
rw_add_includes(${SUBSYS_NAME} "rw/plugin" ${FILES_HPP})
target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${CMAKE_DL_LIBS} sdurw_core)

target_include_directories(
    ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                          $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)
rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../plugin.hpp)


endif()