set(SUBSYS_NAME sdurw_opengl)
set(SUBSYS_DESC "Opengl visualization for RobWork")
set(SUBSYS_DEPS sdurw sdurw_core sdurw_sensor sdurw_models)

if(OPENGL_FOUND AND OPENGL_GLU_FOUND)
    set(DEFAULT TRUE)
    set(REASON)
else()
    set(DEFAULT FALSE)
    set(REASON "OpenGl or Glu was not found.")
endif()

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(
        SRC_FILES
        ${RW_DRAWABLE_USER_FILES}
        Drawable.cpp
        RenderArrow.cpp
        RWGLFrameBuffer.cpp
        DrawableFactory.cpp
        DrawableUtil.cpp
        RenderFrame.cpp
        RenderGeometry.cpp
        RenderLines.cpp
        RenderMatrix.cpp
        SceneOpenGL.cpp
        TactileArrayRender.cpp
        RWGLTexture.cpp
        RenderImage.cpp
        RenderModel3D.cpp
        RenderUtil.cpp
        RenderScan.cpp
        RenderPointCloud.cpp
        RenderCameraFrustum.cpp
        DrawableGeometry.cpp
        RenderSmoothSkin.cpp
        RenderForceTorque.cpp
        RenderVelocity.cpp
        RenderText.cpp
    )

    set(
        SRC_FILES_HPP
        Drawable.hpp
        RWGLFrameBuffer.hpp
        DrawableFactory.hpp
        DrawableUtil.hpp
        RenderFrame.hpp
        RenderGeometry.hpp
        RenderLines.hpp
        RenderMatrix.hpp
        SceneOpenGL.hpp
        TactileArrayRender.hpp
        RWGLTexture.hpp
        RenderImage.hpp
        RenderModel3D.hpp
        RenderUtil.hpp
        RenderScan.hpp
        RenderPointCloud.hpp
        RenderCameraFrustum.hpp
        DrawableGeometry.hpp
        RenderSmoothSkin.hpp
        RenderForceTorque.hpp
        RenderVelocity.hpp
        RenderText.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/opengl" ${SRC_FILES_HPP})

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS} ${OPENGL_LIBRARIES} PRIVATE ${GLUT_glut_LIBRARY})
    target_include_directories(${SUBSYS_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()
