%module sdurw_geometry

%include <stl.i>
%include <std_vector.i>
%include <rwlibs/swig/swig_macros.i>
%include <rwlibs/swig/ext_i/os.i>

%import <rwlibs/swig/ext_i/std.i>

%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>
%import <rwlibs/swig/sdurw_math.i>
%import <rwlibs/swig/sdurw_kinematics.i>


%{
	#include <rw/math.hpp>
    #include <rw/kinematics.hpp>
    #include <rw/common/Traits.hpp>
%}

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
import org.robwork.sdurw_math.*;
import org.robwork.sdurw_kinematics.*;
%}

%{
	#include <rw/geometry/BV.hpp>
%}
%include <rw/geometry/BV.hpp>
//NAMED_OWNEDPTR(BV,rw::geometry::BV);

%{
	#include <rw/geometry/AABB.hpp>
%}
%include <rw/geometry/AABB.hpp>
//NAMED_OWNEDPTR(AABB,rw::geometry::AABB);

%ignore rw::geometry::BSPhere::getPosition () const;
%{
	#include <rw/geometry/BSphere.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using BSphere = rw::geometry::BSphere<T>;
	#endif 
%}
%include <rw/geometry/BSphere.hpp>
NAMED_OWNEDPTR(BSphere,rw::geometry::BSphere<double>);
NAMED_OWNEDPTR(BSphere_f,rw::geometry::BSphere<float>);

%{
	#include <rw/geometry/BVCollider.hpp>
%}
%include <rw/geometry/BVCollider.hpp>
//NAMED_OWNEDPTR(BVCollider,rw::geometry::BVCollider);

%{
	#include <rw/geometry/BVDistanceCalc.hpp>
%}
%include <rw/geometry/BVDistanceCalc.hpp>
//NAMED_OWNEDPTR(BVDistanceCalc,rw::geometry::BVDistanceCalc);

%{
	#include <rw/geometry/GeometryData.hpp>
%}
%include <rw/geometry/GeometryData.hpp>
NAMED_OWNEDPTR(GeometryData,rw::geometry::GeometryData);

%{
	#include <rw/geometry/Primitive.hpp>
%}
%include <rw/geometry/Primitive.hpp>
NAMED_OWNEDPTR(Primitive,rw::geometry::Primitive);

%{
	#include <rw/geometry/Box.hpp>
%}
%include <rw/geometry/Box.hpp>
NAMED_OWNEDPTR(Box,rw::geometry::Box);

%{
	#include <rw/geometry/Cone.hpp>
%}
%include <rw/geometry/Cone.hpp>
NAMED_OWNEDPTR(Cone,rw::geometry::Cone);

%{
	#include <rw/geometry/Covariance.hpp>
%}
%include <rw/geometry/Covariance.hpp>
//NAMED_OWNEDPTR(Covariance,rw::geometry::Covariance);

#if defined(SWIGJAVA)
%ignore rw::geometry::Contour2D::Point::P() const;
%ignore rw::geometry::Contour2D::Point::N() const;
%ignore rw::geometry::Contour2D::center() const;
%ignore rw::geometry::Contour2D::points() const;
#endif
%{
	#include <rw/geometry/Contour2D.hpp>
%}
%include <rw/geometry/Contour2D.hpp>
NAMED_OWNEDPTR(Contour2D,rw::geometry::Contour2D);

%{
	#include <rw/geometry/ConvexHull2D.hpp>
%}
%include <rw/geometry/ConvexHull2D.hpp>
NAMED_OWNEDPTR(ConvexHull2D,rw::geometry::ConvexHull2D);

%{
	#include <rw/geometry/ConvexHull3D.hpp>
%}
%include <rw/geometry/ConvexHull3D.hpp>
NAMED_OWNEDPTR(ConvexHull3D,rw::geometry::ConvexHull3D);


%{
	#include <rw/geometry/Cylinder.hpp>
%}
%include <rw/geometry/Cylinder.hpp>
NAMED_OWNEDPTR(Cylinder,rw::geometry::Cylinder);

%nodefaultctor rw::geometry::Delaunay;
%nodefaultdtor rw::geometry::Delaunay;
%{
	#include <rw/geometry/Delaunay.hpp>
%}
%include <rw/geometry/Delaunay.hpp>

%{
	#include <rw/geometry/DistanceUtil.hpp>
%}
%include <rw/geometry/DistanceUtil.hpp>
NAMED_OWNEDPTR(DistanceUtil,rw::geometry::DistanceUtil);

#if defined(SWIGJAVA)
%ignore rw::geometry::Geometry::getGeometryData() const; 
%ignore rw::geometry::Geometry::getFrame() const;
#elif defined(SWIGLUA)
%ignore rw::geometry::Geometry::setColor(float,float,float);
#endif
%{
	#include <rw/geometry/Geometry.hpp>
%}
%include <rw/geometry/Geometry.hpp>
NAMED_OWNEDPTR(Geometry,rw::geometry::Geometry);
%template (VectorGeometryPtr) std::vector<rw::core::Ptr<rw::geometry::Geometry> >;

%{
	#include <rw/geometry/GeometryUtil.hpp>
%}
%include <rw/geometry/GeometryUtil.hpp>
NAMED_OWNEDPTR(GeometryUtil,rw::geometry::GeometryUtil);

%{
	#include <rw/geometry/HyperSphere.hpp>
%}
%include <rw/geometry/HyperSphere.hpp>
NAMED_OWNEDPTR(HyperSphere,rw::geometry::HyperSphere);

#if defined(SWIGJAVA)
%ignore rw::geometry::TriMesh::getTriMesh(bool) const;
%ignore rw::geometry::TriMesh::getTriMesh() const;
#endif
%{
	#include <rw/geometry/TriMesh.hpp>
%}
%include <rw/geometry/TriMesh.hpp>
NAMED_OWNEDPTR(TriMesh,rw::geometry::TriMesh);

%{
	#include <rw/geometry/IndexedArray.hpp>
%}
%include <rw/geometry/IndexedArray.hpp>
//NAMED_OWNEDPTR(IndexedArray,rw::geometry::IndexedArray);

#if defined(SWIGJAVA)
%ignore rw::geometry::IndexedPolygon< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygon< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonN< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonN< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonNN< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonNN< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonNN< uint16_t >::getNormalIdx(size_t) const;
%ignore rw::geometry::IndexedPolygonNN< uint32_t >::getNormalIdx(size_t) const;
#endif 
%{
	#include <rw/geometry/IndexedPolygon.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedPolygon = rw::geometry::IndexedPolygon<T>;
	#endif 
%}
%include <rw/geometry/IndexedPolygon.hpp>
//%template(VectorIndexedPolygon) std::vector<rw::geometry::IndexedPolygon<uint16_t>>;
NAMED_OWNEDPTR(IndexedPolygon,rw::geometry::IndexedPolygon<uint16_t>);
NAMED_OWNEDPTR(IndexedPolygon_32,rw::geometry::IndexedPolygon<uint32_t>);


%{
	#include <rw/geometry/IndexedTriArray.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedTriArray = rw::geometry::IndexedTriArray<T>;
	#endif 
%}
%include <rw/geometry/IndexedTriArray.hpp>
NAMED_OWNEDPTR(IndexedTriArray,rw::geometry::IndexedTriArray<std::size_t>);


#if defined(SWIGJAVA)
	%ignore rw::geometry::IndexedTriMesh< double >::getNormals();
	%ignore rw::geometry::IndexedTriMesh< double >::getVertices();
	%ignore rw::geometry::IndexedTriMesh< double >::getVertex(size_t);
	%ignore rw::geometry::IndexedTriMesh< double >::getVertexNormal(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMesh< double >::getVertex(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMesh< float >::getNormals();
	%ignore rw::geometry::IndexedTriMesh< float >::getVertices();
	%ignore rw::geometry::IndexedTriMesh< float >::getVertex(size_t);
	%ignore rw::geometry::IndexedTriMesh< float >::getVertexNormal(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMesh< float >::getVertex(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMeshN0< double ,uint16_t>::getVertexNormal(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMeshN0< double ,uint16_t>::getVertex(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMeshN0< double ,uint16_t>::getTriVertex(size_t,size_t);
	%ignore rw::geometry::IndexedTriMeshN0< double ,uint16_t>::getTriangles();
	%ignore rw::geometry::IndexedTriMeshN0< float ,uint16_t>::getVertexNormal(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMeshN0< float ,uint16_t>::getVertex(size_t,rw::geometry::VertexIdx);
	%ignore rw::geometry::IndexedTriMeshN0< float ,uint16_t>::getTriVertex(size_t,size_t);
	%ignore rw::geometry::IndexedTriMeshN0< float ,uint16_t>::getTriangles();
#endif

%nodefaultdtor rw::geometry::IndexedTriMesh;
%{
	#include <rw/geometry/IndexedTriMesh.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedTriMesh = rw::geometry::IndexedTriMesh<T>;
	#endif
	#if SWIG_VERSION < 0x040000 || defined(WIN32)
		template<class T , class R>
		using IndexedTriMeshN0 = rw::geometry::IndexedTriMeshN0<T,R>;
	#endif 
%}
%include <rw/geometry/IndexedTriMesh.hpp>
%template(IndexedTriMeshPtr) rw::core::Ptr<rw::geometry::IndexedTriMesh<double>>;
%template(IndexedTriMesh_fPtr) rw::core::Ptr<rw::geometry::IndexedTriMesh<float>>;

#define INDEXEDTRIMESHN0_TYPE(type) rw::geometry::IndexedTriMeshN0<type,uint16_t>
NAMED_OWNEDPTR(IndexedTriMeshN0,INDEXEDTRIMESHN0_TYPE(double));
NAMED_OWNEDPTR(IndexedTriMeshN0_f,INDEXEDTRIMESHN0_TYPE(float));

#if defined(SWIGJAVA)
%ignore rw::geometry::IndexedTriangle< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangle< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN1< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN1< uint16_t >::getNormalIdx() const;
%ignore rw::geometry::IndexedTriangleN1< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN1< uint32_t >::getNormalIdx() const;
%ignore rw::geometry::IndexedTriangleN3< uint16_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN3< uint16_t >::getNormalIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN3< uint32_t >::getVertexIdx(size_t) const;
%ignore rw::geometry::IndexedTriangleN3< uint32_t >::getNormalIdx(size_t) const;
#endif 
%{
	#include <rw/geometry/IndexedTriangle.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedTriangle = rw::geometry::IndexedTriangle<T>;
	#endif 
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedTriangleN1 = rw::geometry::IndexedTriangleN1<T>;
	#endif 
	#if SWIG_VERSION < 0x040000
		template<class T >
		using IndexedTriangleN3 = rw::geometry::IndexedTriangleN3<T>;
	#endif 
%}
%include <rw/geometry/IndexedTriangle.hpp>
%template(VectorIndexedTriangle) std::vector<rw::geometry::IndexedTriangle<uint16_t>>;
NAMED_OWNEDPTR(IndexedTriangle,rw::geometry::IndexedTriangle<uint16_t>);
NAMED_OWNEDPTR(IndexedTriangle_32,rw::geometry::IndexedTriangle<uint32_t>);
NAMED_OWNEDPTR(IndexedTriangleN1,rw::geometry::IndexedTriangleN1<uint16_t>);
NAMED_OWNEDPTR(IndexedTriangleN1_32,rw::geometry::IndexedTriangleN1<uint32_t>);
NAMED_OWNEDPTR(IndexedTriangleN3,rw::geometry::IndexedTriangleN3<uint16_t>);
NAMED_OWNEDPTR(IndexedTriangleN3_32,rw::geometry::IndexedTriangleN3<uint32_t>);


#if defined(SWIGJAVA)
%ignore rw::geometry::Line::p1() const;
%ignore rw::geometry::Line::p2() const;
#endif
%{
	#include <rw/geometry/Line.hpp>
%}
%include <rw/geometry/Line.hpp>
NAMED_OWNEDPTR(Line,rw::geometry::Line);
NAMED_OWNEDPTR(MetricLine,rw::math::Metric<rw::geometry::Line>);
%template(VectorLine) std::vector<rw::geometry::Line>;

%{
	#include <rw/geometry/Model3D.hpp>
%}
%include <rw/geometry/Model3D.hpp>
NAMED_OWNEDPTR(Model3D,rw::geometry::Model3D);
%template (Model3DPtrVector) std::vector<rw::core::Ptr<rw::geometry::Model3D> >;

%{
	#include <rw/geometry/OBB.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using OBB = rw::geometry::OBB<T>;
	#endif 
%}
%include <rw/geometry/OBB.hpp>
NAMED_OWNEDPTR(OBB,rw::geometry::OBB<double>);

%{
	#include <rw/geometry/OBBCollider.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using OBBCollider = rw::geometry::OBBCollider<T>;
	#endif 
%}
%include <rw/geometry/OBBCollider.hpp>
NAMED_OWNEDPTR(OBBCollider,rw::geometry::OBBCollider<double>);

%{
	#include <rw/geometry/OBBFactory.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using OBBFactory = rw::geometry::OBBFactory<T>;
	#endif 
%}
%include <rw/geometry/OBBFactory.hpp>
NAMED_OWNEDPTR(OBBFactory,rw::geometry::OBBFactory<double>);

%{
	#include <rw/geometry/OBBToleranceCollider.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using OBBToleranceCollider = rw::geometry::OBBToleranceCollider<T>;
	#endif 
%}
%include <rw/geometry/OBBToleranceCollider.hpp>
NAMED_OWNEDPTR(OBBToleranceCollider,rw::geometry::OBBToleranceCollider<double>);

%{
	#include <rw/geometry/OBVDistanceCalc.hpp>
%}
%include <rw/geometry/OBVDistanceCalc.hpp>
//NAMED_OWNEDPTR(OBVDistanceCalc,rw::geometry::OBVDistanceCalc);

#if defined(SWIGJAVA)
%ignore rw::geometry::TriangleN1< double >::getFaceNormal() const;
%ignore rw::geometry::TriangleN1< double >::getVertex(std::size_t) const;
%ignore rw::geometry::TriangleN1< double >::getTriangle() const;
%ignore rw::geometry::TriangleN3< double >::getNormal(std::size_t) const;
%ignore rw::geometry::TriangleN3< double >::getVertex(std::size_t) const;
%ignore rw::geometry::TriangleN3< double >::getTriangle() const;
%ignore rw::geometry::Triangle< float >::getVertex(std::size_t) const;
%ignore rw::geometry::Triangle< float >::getTriangle() const;
%ignore rw::geometry::TriangleN1< float >::getFaceNormal() const;
%ignore rw::geometry::TriangleN1< float >::getVertex(std::size_t) const;
%ignore rw::geometry::TriangleN1< float >::getTriangle() const;
%ignore rw::geometry::TriangleN3< float >::getNormal(std::size_t) const;
%ignore rw::geometry::TriangleN3< float >::getVertex(std::size_t) const;
%ignore rw::geometry::TriangleN3< float >::getTriangle() const;
#endif
%{
	#include <rw/geometry/Triangle.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using Triangle = rw::geometry::Triangle<T>;
		template<class T >
		using TriangleN1 = rw::geometry::TriangleN1<T>;
		template<class T >
		using TriangleN3 = rw::geometry::TriangleN3<T>;
	#endif 
%}
%include <rw/geometry/Triangle.hpp>
%template(VectorTriangle) std::vector<rw::geometry::Triangle<double>>;
%template(VectorTriangleN1) std::vector<rw::geometry::TriangleN1<double>>;
%template(VectorTriangleN3) std::vector<rw::geometry::TriangleN3<double>>;
%template(VectorTriangle_f) std::vector<rw::geometry::Triangle<float>>;
%template(VectorTriangleN1_f) std::vector<rw::geometry::TriangleN1<float>>;
%template(VectorTriangleN3_f) std::vector<rw::geometry::TriangleN3<float>>;
NAMED_OWNEDPTR(Triangle,rw::geometry::Triangle<double>);
NAMED_OWNEDPTR(TriangleN1,rw::geometry::TriangleN1<double>);
NAMED_OWNEDPTR(TriangleN3,rw::geometry::TriangleN3<double>);

%{
	#include <rw/geometry/PlainTriMesh.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using PlainTriMesh = rw::geometry::PlainTriMesh<T>;
	#endif 
%}
%include <rw/geometry/PlainTriMesh.hpp>
NAMED_OWNEDPTR(PlainTriMesh,rw::geometry::PlainTriMesh<rw::geometry::Triangle< double >>);
NAMED_OWNEDPTR(PlainTriMeshN1,rw::geometry::PlainTriMesh<rw::geometry::TriangleN1< double >>);
NAMED_OWNEDPTR(PlainTriMeshN3,rw::geometry::PlainTriMesh<rw::geometry::TriangleN3< double >>);
NAMED_OWNEDPTR(PlainTriMesh_f,rw::geometry::PlainTriMesh<rw::geometry::Triangle< float >>);
NAMED_OWNEDPTR(PlainTriMeshN1_f,rw::geometry::PlainTriMesh<rw::geometry::TriangleN1< float >>);
NAMED_OWNEDPTR(PlainTriMeshN3_f,rw::geometry::PlainTriMesh<rw::geometry::TriangleN3< float >>);


#if defined(SWIGJAVA)
%ignore rw::geometry::Plane::normal() const;
#endif
%{
	#include <rw/geometry/Plane.hpp>
%}
%include <rw/geometry/Plane.hpp>
NAMED_OWNEDPTR(Plane,rw::geometry::Plane);
NAMED_OWNEDPTR(MetricPlane,rw::math::Metric<rw::geometry::Plane>);

#if defined(SWIGJAVA)
%ignore rw::geometry::PointCloud::getData() const;
%ignore rw::geometry::PointCloud::getTriMesh(bool) const;
%ignore rw::geometry::PointCloud::getTriMesh() const;
#endif 
%{
	#include <rw/geometry/PointCloud.hpp>
%}
%include <rw/geometry/PointCloud.hpp>
NAMED_OWNEDPTR(PointCloud,rw::geometry::PointCloud);

%{
	#include <rw/geometry/Polygon.hpp>
	#if SWIG_VERSION < 0x040000 && ! defined(WIN32)
		template<class T >
		using Polygon = rw::geometry::Polygon<T>;
	#endif 
%}
%include <rw/geometry/Polygon.hpp>
NAMED_OWNEDPTR(Polygon,rw::geometry::Polygon<rw::math::Vector3D<double>>);
NAMED_OWNEDPTR(Polygon_f,rw::geometry::Polygon<rw::math::Vector3D<float>>);

%{
	#include <rw/geometry/PolygonUtil.hpp>
%}
%include <rw/geometry/PolygonUtil.hpp>

%{
	#include <rw/geometry/Pyramid.hpp>
%}
%include <rw/geometry/Pyramid.hpp>
NAMED_OWNEDPTR(Pyramid,rw::geometry::Pyramid);

%{
	#include <rw/geometry/QHull3D.hpp>
%}
%include <rw/geometry/QHull3D.hpp>
NAMED_OWNEDPTR(QHull3D,rw::geometry::QHull3D);

%ignore rw::gemoetry::qhull::build (size_t, double*, size_t, std::vector< int >&,
                   std::vector< int >&, std::vector< double >&,std::vector< double >&);

%{
	#include <rw/geometry/QHullND.hpp>
%}
%include <rw/geometry/QHullND.hpp>
//NAMED_OWNEDPTR(QHullND,rw::geometry::QHullND);

%{
	#include <rw/geometry/Ray.hpp>
%}
%include <rw/geometry/Ray.hpp>
NAMED_OWNEDPTR(Ray,rw::geometry::Ray);

%{
	#include <rw/geometry/Sphere.hpp>
%}
%include <rw/geometry/Sphere.hpp>
NAMED_OWNEDPTR(Sphere,rw::geometry::Sphere);

%{
	#include <rw/geometry/SphereDistanceCalc.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using SphereDistanceCalc = rw::proximity::SphereDistanceCalc<T>;
	#endif 
%}
%include <rw/geometry/SphereDistanceCalc.hpp>
NAMED_OWNEDPTR(SphereDistanceCalc,rw::proximity::SphereDistanceCalc<double>);

%{
	#include <rw/geometry/TriMeshSurfaceSampler.hpp>
%}
%include <rw/geometry/TriMeshSurfaceSampler.hpp>
NAMED_OWNEDPTR(TriMeshSurfaceSampler,rw::geometry::TriMeshSurfaceSampler);

%{
	#include <rw/geometry/TriTriIntersectDeviller.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using TriTriIntersectDeviller = rw::geometry::TriTriIntersectDeviller<T>;
	#endif 
%}
%include <rw/geometry/TriTriIntersectDeviller.hpp>
NAMED_OWNEDPTR(TriTriIntersectDeviller,rw::geometry::TriTriIntersectDeviller<double>);
NAMED_OWNEDPTR(TriTriIntersectDeviller_f,rw::geometry::TriTriIntersectDeviller<float>);

%{
	#include <rw/geometry/TriTriIntersectMoller.hpp>
	#if SWIG_VERSION < 0x040000
		template<class T >
		using TriTriIntersectMoller = rw::geometry::TriTriIntersectMoller<T>;
	#endif 
%}
%include <rw/geometry/TriTriIntersectMoller.hpp>
NAMED_OWNEDPTR(TriTriIntersectMoller,rw::geometry::TriTriIntersectMoller<double>);
NAMED_OWNEDPTR(TriTriIntersectMoller_f,rw::geometry::TriTriIntersectMoller<float>);

#if defined(SWIGPYTHON)
RENAME(rw::geometry::TriangleUtil::SortJob::print,printJob)
RENAME(rw::geometry::TriangleUtil::SortJob::from,from_)
#endif
%{
	#include <rw/geometry/TriangleUtil.hpp>
%}
%include <rw/geometry/TriangleUtil.hpp>
NAMED_OWNEDPTR(TriangleUtil,rw::geometry::TriangleUtil);

%{
	#include <rw/geometry/Tube.hpp>
%}
%include <rw/geometry/Tube.hpp>
NAMED_OWNEDPTR(Tube,rw::geometry::Tube);

%{
	#include <rw/geometry/analytic/Face.hpp>
%}
%include <rw/geometry/analytic/Face.hpp>
NAMED_OWNEDPTR(Face,rw::geometry::Face);

%{
	#include <rw/geometry/analytic/AnalyticUtil.hpp>
%}
%include <rw/geometry/analytic/AnalyticUtil.hpp>

RENAME(rw::geometry::BREP::print,printObj);
RENAME(rw::geometry::BREP::Face,BREPFace);
%{
	#include <rw/geometry/analytic/BREP.hpp>
%}
%include <rw/geometry/analytic/BREP.hpp>
NAMED_OWNEDPTR(BREP,rw::geometry::BREP);

#if defined(SWIGJAVA)
%rename(CurveEqual) rw::geometry::Curve::equals;
#endif
%{
	#include <rw/geometry/analytic/Curve.hpp>
%}
%include <rw/geometry/analytic/Curve.hpp>
NAMED_OWNEDPTR(Curve,rw::geometry::Curve);

%{
	#include <rw/geometry/analytic/GenericFace.hpp>
%}
%include <rw/geometry/analytic/GenericFace.hpp>
NAMED_OWNEDPTR(GenericFace,rw::geometry::GenericFace);

%{
	#include <rw/geometry/analytic/ImplicitBREP.hpp>
%}
%include <rw/geometry/analytic/ImplicitBREP.hpp>
NAMED_OWNEDPTR(ImplicitBREP,rw::geometry::ImplicitBREP);

%{
	#include <rw/geometry/analytic/ImplicitFace.hpp>
%}
%include <rw/geometry/analytic/ImplicitFace.hpp>
NAMED_OWNEDPTR(ImplicitFace,rw::geometry::ImplicitFace);

%{
	#include <rw/geometry/analytic/Shell.hpp>
%}
%include <rw/geometry/analytic/Shell.hpp>
NAMED_OWNEDPTR(Shell,rw::geometry::Shell);

%{
	#include <rw/geometry/analytic/quadratics/QuadraticShell.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticShell.hpp>
NAMED_OWNEDPTR(QuadraticShell,rw::geometry::QuadraticShell);

%{
	#include <rw/geometry/analytic/ImplicitShell.hpp>
%}
%include <rw/geometry/analytic/ImplicitShell.hpp>
NAMED_OWNEDPTR(ImplicitShell,rw::geometry::ImplicitShell);

#if defined(SWIGJAVA)
%rename(SurfaceEqual) rw::geometry::Surface::equals;
#endif
%{
	#include <rw/geometry/analytic/Surface.hpp>
%}
%include <rw/geometry/analytic/Surface.hpp>
NAMED_OWNEDPTR(Surface,rw::geometry::Surface);

#if defined(SWIGJAVA)
%rename(SurfaceEqual) rw::geometry::ImplicitSurface::equals;
#endif
%{
	#include <rw/geometry/analytic/ImplicitSurface.hpp>
%}
%include <rw/geometry/analytic/ImplicitSurface.hpp>
NAMED_OWNEDPTR(ImplicitSurface,rw::geometry::ImplicitSurface);

#if defined(SWIGJAVA)
%rename(TorusEqual) rw::geometry::ImplicitTorus::equals;
#endif
%{
	#include <rw/geometry/analytic/ImplicitTorus.hpp>
%}
%include <rw/geometry/analytic/ImplicitTorus.hpp>
NAMED_OWNEDPTR(ImplicitTorus,rw::geometry::ImplicitTorus);

%{
	#include <rw/geometry/analytic/IndexedFaceArray.hpp>
%}
%include <rw/geometry/analytic/IndexedFaceArray.hpp>
NAMED_OWNEDPTR(IndexedFaceArray,rw::geometry::IndexedFaceArray);

#if defined(SWIGJAVA)
%rename(CurveEqual) rw::geometry::ParametricCurve::equals;
#endif
%{
	#include <rw/geometry/analytic/ParametricCurve.hpp>
%}
%include <rw/geometry/analytic/ParametricCurve.hpp>
NAMED_OWNEDPTR(ParametricCurve,rw::geometry::ParametricCurve);

RENAME(rw::geometry::IndexedQuadraticFaceArray::IndexedFace,QuadIndexedFace)
%{
	#include <rw/geometry/analytic/quadratics/IndexedQuadraticFaceArray.hpp>
%}
%include <rw/geometry/analytic/quadratics/IndexedQuadraticFaceArray.hpp>
NAMED_OWNEDPTR(IndexedQuadraticFaceArray,rw::geometry::IndexedQuadraticFaceArray);

%{
	#include <rw/geometry/analytic/quadratics/PlainQuadraticShell.hpp>
%}
%include <rw/geometry/analytic/quadratics/PlainQuadraticShell.hpp>
NAMED_OWNEDPTR(PlainQuadraticShell,rw::geometry::PlainQuadraticShell);

%{
	#include <rw/geometry/analytic/quadratics/QuadraticBREP.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticBREP.hpp>
NAMED_OWNEDPTR(QuadraticBREP,rw::geometry::QuadraticBREP);

#if defined(SWIGJAVA)
%rename(CurveEqual) rw::geometry::QuadraticCurve::equals;
#elif defined(SWIGPYTHON)
%rename(copy) rw::geometry::QuadraticCurve::operator=;
#endif
%{
	#include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticCurve.hpp>
%template(VectorQuadraticCurve) std::vector<rw::geometry::QuadraticCurve>;
NAMED_OWNEDPTR(QuadraticCurve,rw::geometry::QuadraticCurve);

%{
	#include <rw/geometry/analytic/quadratics/QuadraticFace.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticFace.hpp>
NAMED_OWNEDPTR(QuadraticFace,rw::geometry::QuadraticFace);

#if defined(SWIGJAVA)
%rename(SurfaceEqual) rw::geometry::QuadraticSurface::equals;
#endif
%{
	#include <rw/geometry/analytic/quadratics/QuadraticSurface.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticSurface.hpp>
NAMED_OWNEDPTR(QuadraticSurface,rw::geometry::QuadraticSurface);

%{
	#include <rw/geometry/analytic/quadratics/QuadraticUtil.hpp>
%}
%include <rw/geometry/analytic/quadratics/QuadraticUtil.hpp>
