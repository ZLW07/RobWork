cmake_minimum_required(VERSION 3.10  )

project(yaobi)

set(YAOBI_HPP_FILES
    include/yaobi/yaobi.h
    include/yaobi/yaobi_config.h
    include/yaobi/yaobi_fpu.h
    include/yaobi/yaobi_matvec.h
    include/yaobi/yaobi_mesh_interface.h
    include/yaobi/yaobi_moments.h
    include/yaobi/yaobi_obb_node.h
    include/yaobi/yaobi_settings.h
    include/yaobi/yaobi_tree_builder.h
    include/yaobi/yaobi_tri_overlap.h
    include/yaobi/yaobi_vector.h
)

add_library(yaobi src/yaobi_mesh_interface.cpp src/yaobi_moments.cpp src/yaobi_obb_node.cpp
                  src/yaobi_tree_builder.cpp src/yaobi_tri_overlap.cpp src/yaobi.cpp
)
add_library(RW::yaobi ALIAS yaobi)

target_include_directories(
    yaobi
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/ext/rwyaobi/>
    PRIVATE include/yaobi
)

set_target_properties(yaobi PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
set_target_properties(
            yaobi PROPERTIES VERSION 1 SOVERSION 1.0
        )
        
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h.cmake
               ${CMAKE_CURRENT_SOURCE_DIR}/include/yaobi/yaobi_config.h
)

install(
    TARGETS yaobi
    EXPORT ${PROJECT_PREFIX}Targets
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT yaobi
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT yaobi
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT yaobi
)
install(
    FILES ${YAOBI_HPP_FILES}
    DESTINATION "${INCLUDE_INSTALL_DIR}/ext/rwyaobi/"
    COMPONENT yaobi
)
