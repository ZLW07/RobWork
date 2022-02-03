set(SUBSYS_NAME sdurw_geometry)
set(SUBSYS_DESC "The core RobWork library!")
set(SUBSYS_DEPS sdurw_core sdurw_common sdurw_math sdurw_kinematics)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)
    set(FILES_CPP
        AABB.cpp
        BSphere.cpp
        BV.cpp
        BVCollider.cpp
        BVDistanceCalc.cpp
        Box.cpp
        Cone.cpp
        Contour2D.cpp
        ConvexHull2D.cpp
        ConvexHull3D.cpp
        ConvexHullND.cpp
        Covariance.cpp
        Cylinder.cpp
        Delaunay.cpp
        DistanceUtil.cpp
        Geometry.cpp
        GeometryData.cpp
        GeometryUtil.cpp
        HyperSphere.cpp
        IndexedArray.cpp
        IndexedTriArray.cpp
        IndexedTriMesh.cpp
        IndexedTriangle.cpp
        IntersectUtil.cpp
        Line.cpp
        Model3D.cpp
        OBB.cpp
        OBBCollider.cpp
        OBBFactory.cpp
        OBBToleranceCollider.cpp
        PlainTriMesh.cpp
        Plane.cpp
        PointCloud.cpp
        PolygonUtil.cpp
        Primitive.cpp
        Pyramid.cpp
        QHull3D.cpp
        QHullND.cpp
        Ray.cpp
        Sphere.cpp
        SphereDistanceCalc.cpp
        TriMesh.cpp
        TriMeshSurfaceSampler.cpp
        TriTriIntersectDeviller.cpp
        TriTriIntersectMoller.cpp
        Triangle.cpp
        TriangleUtil.cpp
        Triangulate.cpp
        Tube.cpp
        analytic/AnalyticUtil.cpp
        analytic/BREP.cpp
        analytic/Curve.cpp
        analytic/Face.cpp
        analytic/GenericFace.cpp
        analytic/ImplicitBREP.cpp
        analytic/ImplicitFace.cpp
        analytic/ImplicitShell.cpp
        analytic/ImplicitSurface.cpp
        analytic/ImplicitTorus.cpp
        analytic/IndexedFaceArray.cpp
        analytic/ParametricCurve.cpp
        analytic/Shell.cpp
        analytic/Surface.cpp
        analytic/quadratics/IndexedQuadraticFaceArray.cpp
        analytic/quadratics/PlainQuadraticShell.cpp
        analytic/quadratics/QuadraticBREP.cpp
        analytic/quadratics/QuadraticCurve.cpp
        analytic/quadratics/QuadraticFace.cpp
        analytic/quadratics/QuadraticShell.cpp
        analytic/quadratics/QuadraticSurface.cpp
        analytic/quadratics/QuadraticUtil.cpp
    )

    set(FILES_HPP
        AABB.hpp
        BSphere.hpp
        BV.hpp
        BVCollider.hpp
        BVDistanceCalc.hpp
        Box.hpp
        Cone.hpp
        Contour2D.hpp
        ConvexHull2D.hpp
        ConvexHull3D.hpp
        Covariance.hpp
        Cylinder.hpp
        Delaunay.hpp
        DistanceUtil.hpp
        Geometry.hpp
        GeometryData.hpp
        GeometryUtil.hpp
        HyperSphere.hpp
        IndexedArray.hpp
        IndexedPolygon.hpp
        IndexedTriArray.hpp
        IndexedTriMesh.hpp
        IndexedTriangle.hpp
        Line.hpp
        Model3D.hpp
        OBB.hpp
        OBBCollider.hpp
        OBBFactory.hpp
        OBBToleranceCollider.hpp
        OBBToleranceCollider.hpp
        OBVDistanceCalc.hpp
        PlainTriMesh.hpp
        Plane.hpp
        PointCloud.hpp
        Polygon.hpp
        PolygonUtil.hpp
        Primitive.hpp
        Pyramid.hpp
        QHull3D.hpp
        QHullND.hpp
        Ray.hpp
        Sphere.hpp
        SphereDistanceCalc.hpp
        TriMesh.hpp
        TriMeshSurfaceSampler.hpp
        TriTriIntersectDeviller.hpp
        TriTriIntersectMoller.hpp
        Triangle.hpp
        TriangleUtil.hpp
        Tube.hpp
    )
    set(FILES_HPP_analytic
        analytic/AnalyticUtil.hpp
        analytic/BREP.hpp
        analytic/Curve.hpp
        analytic/Face.hpp
        analytic/GenericFace.hpp
        analytic/ImplicitBREP.hpp
        analytic/ImplicitFace.hpp
        analytic/ImplicitShell.hpp
        analytic/ImplicitSurface.hpp
        analytic/ImplicitTorus.hpp
        analytic/IndexedFaceArray.hpp
        analytic/ParametricCurve.hpp
        analytic/Shell.hpp
        analytic/Surface.hpp
    )
    set(FILES_HPP_analytic_quadratic
        analytic/quadratics/IndexedQuadraticFaceArray.hpp
        analytic/quadratics/PlainQuadraticShell.hpp
        analytic/quadratics/QuadraticBREP.hpp
        analytic/quadratics/QuadraticCurve.hpp
        analytic/quadratics/QuadraticFace.hpp
        analytic/quadratics/QuadraticShell.hpp
        analytic/quadratics/QuadraticSurface.hpp
        analytic/quadratics/QuadraticUtil.hpp
    )

    if(RW_HAVE_PQP)
        set(FILES_CPP ${FILES_CPP} TriDistanceCalc.cpp TriTriToleranceIntersect.cpp)
        set(FILES_HPP ${FILES_HPP} TriDistanceCalc.hpp TriTriToleranceIntersect.hpp)
    endif()

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP} ${FILES_HPP_analytic} ${FILES_HPP_analytic_quadratic})
    rw_add_includes(${SUBSYS_NAME} "rw/geometry" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/geometry/analytic" ${FILES_HPP_analytic})
    rw_add_includes(${SUBSYS_NAME} "rw/geometry/analytic/quadratics" ${FILES_HPP_analytic_quadratic})
    rw_add_includes(${SUBSYS_NAME} "rw" ../geometry.hpp)

    if(TARGET qhull_build)
        add_dependencies(${SUBSYS_NAME} qhull_build)
        target_include_directories(${SUBSYS_NAME} PRIVATE ${QHULL_NATIVE_ROOT}/include)
    endif()

    target_link_libraries(
        ${SUBSYS_NAME}
        PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS}
        PRIVATE ${QHULL_LIBRARIES}
    )

    target_include_directories(
        ${SUBSYS_NAME}
        PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${PQP_INCLUDE_DIR}>
               $<BUILD_INTERFACE:${RW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

    rw_include_eigen(${SUBSYS_NAME})

    if(RW_ENABLE_INTERNAL_PQP_TARGET)
        target_include_directories(
            ${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${RW_EXT_INSTALL_DIR}/rwpqp>
        )
    else()
        target_include_directories(${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${PQP_INCLUDE_DIR}>)
    endif()

endif()
