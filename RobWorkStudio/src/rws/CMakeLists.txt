set(SUBSYS_NAME sdurws)
set(SUBSYS_DESC "Visulization tool")
set(SUBSYS_DEPS RW::sdurw)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    # if we want to use ui files add them here
    set(UIS_FILES AboutBox.ui propertyview/PropertyViewEditor.ui propertyview/PropertyViewDialog.ui)

    if(DEFINED Qt6Core_VERSION)
        qt_wrap_ui(UIS_OUT_H ${UIS_FILES})
    else()
        qt5_wrap_ui(UIS_OUT_H ${UIS_FILES})
    endif()

    # Standard files to compile:
    set(SRC_FILES_CPP
        ArcBallController.cpp
        # FixedAxisController.cpp
        RWSImageLoaderPlugin.cpp
        ImageUtil.cpp
        RobWorkStudio.cpp
        RWStudioView3D.cpp
        SceneOpenGLViewer.cpp
        RobWorkStudioPlugin.cpp
        AboutBox.cpp
        HelpAssistant.cpp
        RWSSpinBox.cpp
        ImageView.cpp
        propertyview/PropertyViewEditor.cpp
        propertyview/PropertyViewDialog.cpp
    )

    set(SRC_FILES_HPP
        ArcBallController.hpp
        RWSImageLoaderPlugin.hpp
        ImageUtil.hpp
        # FixedAxisController.hpp
        RobWorkStudio.hpp
        RWSSpinBox.hpp
        SceneOpenGLViewer.hpp
        RobWorkStudioPlugin.hpp
        RWStudioView3D.hpp
        AboutBox.hpp
        HelpAssistant.hpp
        ImageView.hpp
    )
    set(SRC_FILES_HPP_propertyview propertyview/PropertyViewEditor.hpp
                                   propertyview/PropertyViewDialog.hpp
    )

    # Files to moc and compile:
    set(MocHeaderFiles
        RobWorkStudioPlugin.hpp
        RobWorkStudio.hpp
        SceneOpenGLViewer.hpp
        AboutBox.hpp
        RWStudioView3D.hpp
        ImageView.hpp
        propertyview/PropertyViewEditor.hpp
        propertyview/PropertyViewDialog.hpp
    )

    if(RWS_USE_PYTHON)
        set(SRC_FILES_CPP ${SRC_FILES_CPP} pythonpluginloader/PyPlugin.cpp
                          pythonpluginloader/PythonRunner.cpp
        )
        set(SRC_FILES_HPP_python pythonpluginloader/PyPlugin.hpp
                                 pythonpluginloader/PythonRunner.hpp
        )
        set(MocHeaderFiles ${MocHeaderFiles} pythonpluginloader/PyPlugin.hpp)
    endif()

    set_source_files_properties(${SRC_FILES_CPP} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # set(MocSrcFiles) qt5_wrap_cpp(MocSrcFiles ${MocHeaderFiles} TARGET ${SUBSYS_NAME})
    # set(CMAKE_AUTOMOC ON)
    set_source_files_properties(${MocSrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

    # Rcc the files:
    if(DEFINED Qt6Core_VERSION)
        qt_add_resources(ResourceFiles rwstudio_resources.qrc)
    else()
        qt5_add_resources(ResourceFiles rwstudio_resources.qrc)
    endif()
    # We create a static library named rwstudio that all plugins should link to. Our RobWorkStudio
    # executable links to this library also.
    rw_add_library(
        ${SUBSYS_NAME}
        ${UIS_OUT_H}
        ${SRC_FILES_CPP}
        ${SRC_FILES_HPP_python}
        ${SRC_FILES_HPP_propertyview}
        ${MocSrcFiles}
        ${ResourceFiles}
    )

    # Add the cmake binary dir because this is where the ui files are placed
    target_include_directories(
        sdurws
        PUBLIC $<BUILD_INTERFACE:${RWS_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${PYTHON_INCLUDE_DIRS}
    )

    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC RWS::qtpropertybrowser
               ${PYTHON_LIBRARIES}
               ${QT_LIBRARIES}
               RW::sdurw_proximitystrategies
               RW::sdurw_opengl
               RW::sdurw
               ${OPENGL_LIBRARIES}
        PRIVATE ${GLUT_glut_LIBRARY} ${PYTHON_LIBRARIES}
    )

    set_target_properties(sdurws PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

    rw_add_includes(sdurws "rws" ${SRC_FILES_HPP})
    rw_add_includes(sdurws "rws/pythonpluginloader" ${SRC_FILES_HPP_python})
    rw_add_includes(sdurws "rws/propertyview" ${SRC_FILES_HPP_propertyview})

    if(RWS_USE_PYTHON)
        target_compile_definitions(sdurws PRIVATE PYTHON_VERSION_MINOR=${PYTHON_VERSION_MINOR})
    endif()
    
    set(SUBSYS_NAME_PLUGIN ${SUBSYS_NAME}_plugin.rwplugin)
    set(SUBSYS_DESC_PLUGIN "Plugin providing extensions to RobWork from RobWorkStudio!")
    set(SUBSYS_DEPS_PLUGIN ${SUBSYS_NAME})

    set(build_plugin TRUE)
    rw_subsys_option(
        build_plugin ${SUBSYS_NAME_PLUGIN} ${SUBSYS_DESC_PLUGIN} ${DEFAULT}
        DEPENDS ${SUBSYS_DEPS_PLUGIN}
        ADD_DOC
    )

    if(build_plugin)
        rw_add_plugin(${SUBSYS_NAME_PLUGIN} MODULE RWSImageLoaderPlugin.cpp
                      RWSImageLoaderPlugin.hpp
        )
        target_link_libraries(${SUBSYS_NAME_PLUGIN} sdurws ${QT_LIBRARIES} RW::sdurw)
    endif()

endif()
