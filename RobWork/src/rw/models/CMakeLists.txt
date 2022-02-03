set(SUBSYS_NAME sdurw_models)
set(SUBSYS_DESC "A Libray for modeling of robots and workcells")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_kinematics sdurw_math sdurw_geometry sdurw_sensor)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        CompositeDevice.cpp
        CompositeJointDevice.cpp
        ControllerModel.cpp
        DeformableObject.cpp
        DependentJoint.cpp
        DependentPrismaticJoint.cpp
        DependentRevoluteJoint.cpp
        Device.cpp
        DeviceJacobianCalculator.cpp
        DHParameterSet.cpp
        JacobianCalculator.cpp
        JacobianUtil.cpp
        Joint.cpp
        JointDevice.cpp
        JointDeviceJacobianCalculator.cpp
        MobileDevice.cpp
        Object.cpp
        ParallelDevice.cpp
        ParallelLeg.cpp
        PrismaticJoint.cpp
        PrismaticSphericalJoint.cpp
        PrismaticUniversalJoint.cpp
        RevoluteJoint.cpp
        RigidBodyInfo.cpp
        RigidObject.cpp
        SE3Device.cpp
        SerialDevice.cpp
        SphericalJoint.cpp
        TreeDevice.cpp
        UniversalJoint.cpp
        VirtualJoint.cpp
        WorkCell.cpp
        Models.cpp
    )

    set(FILES_HPP
        CompositeDevice.hpp
        CompositeJointDevice.hpp
        ControllerModel.hpp
        DeformableObject.hpp
        DependentJoint.hpp
        DependentPrismaticJoint.hpp
        DependentRevoluteJoint.hpp
        Device.hpp
        DeviceJacobianCalculator.hpp
        DHParameterSet.hpp
        JacobianCalculator.hpp
        JacobianUtil.hpp
        Joint.hpp
        JointDevice.hpp
        JointDeviceJacobianCalculator.hpp
        MobileDevice.hpp
        Models.hpp
        Object.hpp
        ParallelDevice.hpp
        ParallelLeg.hpp
        PrismaticJoint.hpp
        PrismaticSphericalJoint.hpp
        PrismaticUniversalJoint.hpp
        RevoluteJoint.hpp
        RigidBodyInfo.hpp
        RigidObject.hpp
        SE3Device.hpp
        SerialDevice.hpp
        SphericalJoint.hpp
        TreeDevice.hpp
        UniversalJoint.hpp
        VirtualJoint.hpp
        WorkCell.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/models" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_add_includes(${SUBSYS_NAME} "rw" ${CMAKE_CURRENT_SOURCE_DIR}/../models.hpp)

endif()
