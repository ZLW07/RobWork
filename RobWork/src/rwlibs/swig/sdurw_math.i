%module sdurw_math

%include <rwlibs/swig/swig_macros.i>


%include <stl.i>
%include <std_vector.i>


%import <rwlibs/swig/sdurw_core.i>
%import <rwlibs/swig/sdurw_common.i>

%include <rwlibs/swig/ext_i/rw_eigen.i>

%pragma(java) jniclassimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
%}
%pragma(java) moduleimports=%{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
%}
%typemap(javaimports) SWIGTYPE %{
import org.robwork.sdurw_core.*;
import org.robwork.sdurw_common.*;
%}

%rename(copy) rw::math::Rotation3DVector::operator=;
%{
    #include <rw/math/Rotation3DVector.hpp>
%}
%include <rw/math/Rotation3DVector.hpp>

%{
    #include <rw/math/CameraMatrix.hpp>
%}
%include <rw/math/CameraMatrix.hpp>


#if defined(SWIGPYTHON)
    %rename(copy) rw::math::EAA::operator=;
#endif 

%ignore rw::math::EAA::toVector3D() const;
%ignore rw::math::EAA::e() const;
%{
    #include <rw/math/EAA.hpp>
%}
%include <rw/math/EAA.hpp>
FRIEND_OPERATOR(rw::math::Rotation3D<double>, rw::math::EAA<double>, *);
FRIEND_OPERATOR(rw::math::Rotation3D<float>, rw::math::EAA<float>, *);

FRIEND_OPERATOR(rw::math::Vector3D<double>, rw::math::EAA<double>, +);
FRIEND_OPERATOR(rw::math::Vector3D<float>, rw::math::EAA<float>, +);
FRIEND_OPERATOR(rw::math::Vector3D<double>, rw::math::EAA<double>, -);
FRIEND_OPERATOR(rw::math::Vector3D<float>, rw::math::EAA<float>, -);

FRIEND_OPERATOR_RET(rw::math::Vector3D<double>, rw::math::EAA<double>, ==, bool);
FRIEND_OPERATOR_RET(rw::math::Vector3D<float>, rw::math::EAA<float>, ==, bool);
FRIEND_OPERATOR_RET(rw::math::Vector3D<double>, rw::math::EAA<double>, !=, bool);
FRIEND_OPERATOR_RET(rw::math::Vector3D<float>, rw::math::EAA<float>, !=, bool);

%template(cross) rw::math::cross<double>;
%template(cross) rw::math::cross<float>;

%ignore rw::math::EigenDecomposition::MapSort;
%{
    #include <rw/math/EigenDecomposition.hpp>
%}
%include <rw/math/EigenDecomposition.hpp>

%{
    #include <rw/math/Function.hpp>
%}
%include <rw/math/Function.hpp>


%ignore rw::math::InertiaMatrix::e() const;
%ignore rw::math::InertiaMatrix::operator();
%ignore rw::math::InertiaMatrix::operator()const;
%{
    #include <rw/math/InertiaMatrix.hpp>
%}
%include <rw/math/InertiaMatrix.hpp>
FRIEND_OPERATOR(rw::math::Rotation3D<double>,rw::math::InertiaMatrix<double>,*);
FRIEND_OPERATOR(rw::math::Rotation3D<float>,rw::math::InertiaMatrix<float>,*);


%ignore rw::math::Jacobian::e() const;
%{
    #include <rw/math/Jacobian.hpp>
%}
%include <rw/math/Jacobian.hpp>

%ignore rw::math::Line2D::p1() const;
%ignore rw::math::Line2D::p2() const;
%{
    #include <rw/math/Line2D.hpp>
%}
%include <rw/math/Line2D.hpp>

%{
    #include <rw/math/Line2DPolar.hpp>
%}
%include <rw/math/Line2DPolar.hpp>

%{
    #include <rw/math/LinearAlgebra.hpp>
%}
%include <rw/math/LinearAlgebra.hpp>

%extend rw::math::LinearAlgebra {
    static bool isSO(Eigen::Matrix<double,3,3> var){
        return rw::math::LinearAlgebra::isSO(var);
    }
}

//%template(isSO_f) rw::math::LinearAlgebra::isSO<float>;

%{
    #include <rw/math/Quaternion.hpp>
    #include <rw/math/Math.hpp>
%}
%include <rw/math/Math.hpp>
%template(quaternionToEAA) rw::math::Math::quaternionToEAA<double>;
%template(quaternionToEAA) rw::math::Math::quaternionToEAA<float>;
%template(eaaToQuaternion) rw::math::Math::eaaToQuaternion<double>;
%template(eaaToQuaternion) rw::math::Math::eaaToQuaternion<float>;
%template(zyxToRotation3D) rw::math::Math::zyxToRotation3D<double>;
%template(zyxToRotation3D) rw::math::Math::zyxToRotation3D<float>;
%template(skew) rw::math::Math::skew<double>;
%template(skew) rw::math::Math::skew<float>;
%template(ranQuaternion) rw::math::Math::ranQuaternion<double>;
%template(ranTransform3D) rw::math::Math::ranTransform3D<double>;
%template(ranRotation3D) rw::math::Math::ranRotation3D<double>;
%template(ranQuaternion_f) rw::math::Math::ranQuaternion<float>;
%template(ranTransform3D_f) rw::math::Math::ranTransform3D<float>;
%template(ranRotation3D_f) rw::math::Math::ranRotation3D<float>;
%template(fromStdVectorToMat) rw::math::Math::fromStdVectorToMat<double,rw::math::Transform3D<double>>;
%template(fromStdVectorToMat) rw::math::Math::fromStdVectorToMat<double,rw::math::Transform3D<float>>;
%template(fromStdVectorToMat_f) rw::math::Math::fromStdVectorToMat<float,rw::math::Transform3D<double>>;
%template(fromStdVectorToMat_f) rw::math::Math::fromStdVectorToMat<float,rw::math::Transform3D<float>>;
%template(fromStdVectorToMat) rw::math::Math::fromStdVectorToMat<double,rw::math::Rotation3D<double>>;
%template(fromStdVectorToMat) rw::math::Math::fromStdVectorToMat<double,rw::math::Rotation3D<float>>;
%template(fromStdVectorToMat_f) rw::math::Math::fromStdVectorToMat<float,rw::math::Rotation3D<double>>;
%template(fromStdVectorToMat_f) rw::math::Math::fromStdVectorToMat<float,rw::math::Rotation3D<float>>;

%extend rw::math::Math {
    static std::vector< double > toStdVector (const rw::math::Vector3D<double>& tmp, int size){
        return rw::math::Math::toStdVector(tmp,size);
    }
    static std::vector< double > toStdVector (const rw::math::Vector3D<float>& tmp, int size){
        return rw::math::Math::toStdVector(tmp,size);
    }
    static std::vector< double > toStdVector (const rw::math::Transform3D<double>& tmp, int size1,int size2){
        return rw::math::Math::toStdVector(tmp,size1,size2);
    }
    static std::vector< double > toStdVector (const rw::math::Transform3D<float>& tmp, int size1,int size2){
        return rw::math::Math::toStdVector(tmp,size1, size2);
    }
    static std::vector< double > toStdVector (const rw::math::Rotation3D<double>& tmp, int size1,int size2){
        return rw::math::Math::toStdVector(tmp,size1,size2);
    }
    static std::vector< double > toStdVector (const rw::math::Rotation3D<float>& tmp, int size1,int size2){
        return rw::math::Math::toStdVector(tmp,size1, size2);
    }
}


%rename(copy) rw::math::Metric::operator=;
%{
    #include <rw/math/Metric.hpp>
%}
%include <rw/math/Metric.hpp>
%template(MetricQ) rw::math::Metric<rw::math::Q>;
%template(MetricVector2D) rw::math::Metric<rw::math::Vector2D< double > >;
%template(MetricVector3D) rw::math::Metric<rw::math::Vector3D< double > >;
%template(MetricTransform3D) rw::math::Metric<rw::math::Transform3D<double>>;
%template(MetricRotation3D) rw::math::Metric<rw::math::Rotation3D<double>>;
%template(MetricTransform3D_f) rw::math::Metric<rw::math::Transform3D<float>>;
%template(MetricRotation3D_f) rw::math::Metric<rw::math::Rotation3D<float>>;

NAMED_OWNEDPTR(MetricQ, rw::math::Metric<rw::math::Q>);
NAMED_OWNEDPTR(MetricVector2D, rw::math::Metric<rw::math::Vector2D< double > >);
NAMED_OWNEDPTR(MetricVector3D, rw::math::Metric<rw::math::Vector3D< double > >);
NAMED_OWNEDPTR(MetricTransform3D, rw::math::Metric<rw::math::Transform3D<double>>);
NAMED_OWNEDPTR(MetricRotation3D, rw::math::Metric<rw::math::Rotation3D<double>>);



%{
    #include <rw/math/MetricFactory.hpp>
%}
%include <rw/math/MetricFactory.hpp>

// Functions
%template(makeEuclideanQ) rw::math::MetricFactory::makeEuclidean<rw::math::Q>;
%template(makeWeightedEuclideanQ) rw::math::MetricFactory::makeWeightedEuclidean<rw::math::Q>;
%template(makeInfinityQ) rw::math::MetricFactory::makeInfinity<rw::math::Q>;
%template(makeWeightedInfinityQ) rw::math::MetricFactory::makeWeightedInfinity<rw::math::Q>;
%template(makeManhattanQ) rw::math::MetricFactory::makeManhattan<rw::math::Q>;
%template(makeWeightedManhattanQ) rw::math::MetricFactory::makeWeightedManhattan<rw::math::Q>;
%template(makeTransform3DMetric) rw::math::MetricFactory::makeTransform3DMetric<double>;
%template(makeRotation3DMetric) rw::math::MetricFactory::makeRotation3DMetric<double>;

// TYPES
%template(ManhattenMatricQ) rw::math::ManhattanMetric< rw::math::Q >;
%template(WeightedManhattenMetricQ) rw::math::WeightedManhattanMetric< rw::math::Q >;
%template(EuclideanMetricQ ) rw::math::EuclideanMetric< rw::math::Q  >;
%template(WeightedEuclideanMetricQ) rw::math::WeightedEuclideanMetric< rw::math::Q >;
%template(InfinityMetricQ) rw::math::InfinityMetric< rw::math::Q >;
%template(WeightedInfinityMetricQ) rw::math::WeightedInfinityMetric< rw::math::Q >;

%template(ManhattanMetricVector2D) rw::math::ManhattanMetric< rw::math::Vector2D<double> >;
%template(WeightedManhattenMetricVector2D) rw::math::WeightedManhattanMetric< rw::math::Vector2D<double> >;
%template(EuclideanMetricVector2D) rw::math::EuclideanMetric< rw::math::Vector2D<double> >;
%template(WeightedEuclideanMetricVector2D) rw::math::WeightedEuclideanMetric< rw::math::Vector2D<double> >;
%template(InfinityMetricVector2D) rw::math::InfinityMetric< rw::math::Vector2D<double> >;
%template(WeightedInfinityMetricVector2D) rw::math::WeightedInfinityMetric< rw::math::Vector2D<double> >;

%template(ManhattanMetricVector3D) rw::math::ManhattanMetric< rw::math::Vector3D<double> >;
%template(WeightedManhattenMetricVector3D) rw::math::WeightedManhattanMetric< rw::math::Vector3D<double> >;
%template(EuclideanMetricVector3D) rw::math::EuclideanMetric< rw::math::Vector3D<double> >;
%template(WeightedEuclideanMetricVector3D) rw::math::WeightedEuclideanMetric< rw::math::Vector3D<double> >;
%template(InfinityMetricVector3D) rw::math::InfinityMetric< rw::math::Vector3D<double> >;
%template(WeightedInfinityMetricVector3D) rw::math::WeightedInfinityMetric< rw::math::Vector3D<double> >;

%template(Rotation3DAngleMetric_d) rw::math::Rotation3DAngleMetric<double>;
ADD_DEFINITION(Rotation3DAngleMetric_d, Rotation3DAngleMetric);
%template(Rotation3DAngleMetric_f) rw::math::Rotation3DAngleMetric<float>;

%template(Transform3DAngleMetric_d) rw::math::Transform3DAngleMetric <double>;
ADD_DEFINITION(Transform3DAngleMetric_d, Transform3DAngleMetric);
%template(Transform3DAngleMetric_f) rw::math::Transform3DAngleMetric <float>;

%{
    #include <rw/math/MetricUtil.hpp>
%}
%include <rw/math/MetricUtil.hpp>

%template(norm1) rw::math::MetricUtil::norm1<rw::math::Vector3D<double>>;
%template(norm1) rw::math::MetricUtil::norm1<rw::math::Vector3D<float>>;
%template(norm1) rw::math::MetricUtil::norm1<rw::math::Quaternion<double>>;
%template(norm1) rw::math::MetricUtil::norm1<rw::math::Quaternion<float>>;
%template(norm1) rw::math::MetricUtil::norm1<rw::math::Q>;
%template(norm2) rw::math::MetricUtil::norm2<rw::math::Vector3D<double>>;
%template(norm2) rw::math::MetricUtil::norm2<rw::math::Vector3D<float>>;
%template(norm2) rw::math::MetricUtil::norm2<rw::math::Quaternion<double>>;
%template(norm2) rw::math::MetricUtil::norm2<rw::math::Quaternion<float>>;
%template(norm2) rw::math::MetricUtil::norm2<rw::math::Q>;
%template(normInf) rw::math::MetricUtil::normInf<rw::math::Vector3D<double>>;
%template(normInf) rw::math::MetricUtil::normInf<rw::math::Vector3D<float>>;
%template(normInf) rw::math::MetricUtil::normInf<rw::math::Quaternion<double>>;
%template(normInf) rw::math::MetricUtil::normInf<rw::math::Quaternion<float>>;
%template(normInf) rw::math::MetricUtil::normInf<rw::math::Q>;
//%template(dist1) rw::math::MetricUtil::dist1<double>;
//%template(dist1) rw::math::MetricUtil::dist1<float>;


%ignore rw::math::PerspectiveTransform2D::e() const;
%{
    #include <rw/math/PerspectiveTransform2D.hpp>
%}
%include <rw/math/PerspectiveTransform2D.hpp>

%{
    #include <rw/math/PolynomialND.hpp>
%}
%include <rw/math/PolynomialND.hpp>

%template(PolynomialNDdDouble) rw::math::PolynomialND<double,double>;
%template(PolynomialNDfFloat) rw::math::PolynomialND<float,float>;
%template(PolynomialNDEigenRowVector3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,1,3>,double>;
%template(PolynomialNDEigenVector3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,3,1>,double>;
%template(PolynomialNDEigenMatrix3dDouble) rw::math::PolynomialND<Eigen::Matrix<double,3,3>,double>;
%template(PolynomialNDEigenRowVector3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,1,3>,float>;
%template(PolynomialNDEigenVector3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,3,1>,float>;
%template(PolynomialNDEigenMatrix3fFloat) rw::math::PolynomialND<Eigen::Matrix<float,3,3>,float>;
%template(PolynomialNDidComplexDouble) rw::math::PolynomialND<std::complex<double>,std::complex<double>>;
%template(PolynomialNDEigenRowVector3idComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<double>,1,3>,std::complex<double>>;
%template(PolynomialNDEigenVector3idComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<double>,3,1>,std::complex<double>>;
%template(PolynomialNDEigenMatrix3ifComplexDouble) rw::math::PolynomialND<Eigen::Matrix<std::complex<float>,3,3>,std::complex<float>>;

%rename(copy) rw::math::Polynomial::operator=;
%{
    #include <rw/math/Polynomial.hpp>
%}
%include <rw/math/Polynomial.hpp>

%{
    #include <rw/math/PolynomialSolver.hpp>
%}
%include <rw/math/PolynomialSolver.hpp>


%ignore rw::math::Pose2D::x() const;
%ignore rw::math::Pose2D::y() const;
%ignore rw::math::Pose2D::theta() const;
%ignore rw::math::Pose2D::getPos() const;
%{
    #include <rw/math/Pose2D.hpp>
%}
%include <rw/math/Pose2D.hpp>

%ignore rw::math::Pose6D::getPos() const;
%ignore rw::math::Pose6D::getEAA() const;
%ignore rw::math::Pose6D::get() const;
%{
    #include <rw/math/Pose6D.hpp>
%}
%include <rw/math/Pose6D.hpp>

%{
    #include <rw/math/ProjectionMatrix.hpp>
%}
%include <rw/math/ProjectionMatrix.hpp>

%define Q_SWIG_CONSTRUCTORS
    Q(int,double);
    Q(int,double,double);
    Q(int,double,double,double);
    Q(int,double,double,double,double);
    Q(int,double,double,double,double,double);
    Q(int,double,double,double,double,double,double);
    Q(int,double,double,double,double,double,double,double);
    Q(int,double,double,double,double,double,double,double,double);
    Q(int,double,double,double,double,double,double,double,double,double);
#if !defined(SWIGLUA)
    Q(double);
    Q(double,double);
    Q(double,double,double);
    Q(double,double,double,double);
    Q(double,double,double,double,double);
    Q(double,double,double,double,double,double);
    Q(double,double,double,double,double,double,double);
    Q(double,double,double,double,double,double,double,double);
    Q(double,double,double,double,double,double,double,double,double);
#endif
%enddef

%ignore operator* (double s, const Q& v);
%ignore rw::math::Q::e() const;
%{
    #include <rw/math/Q.hpp>
%}
%include <rw/math/Q.hpp>
%template(PairQ) std::pair<rw::math::Q,rw::math::Q>;

%rename(copy) rw::math::Quaternion::operator=;
%ignore rw::math::Quaternion::e() const;
%{
    #include <rw/math/Quaternion.hpp>
%}
%include <rw/math/Quaternion.hpp>

%{
    #include <rw/math/Random.hpp>
%}
%include <rw/math/Random.hpp>

%{
    #include <rw/math/Rotation2D.hpp>
%}
%include <rw/math/Rotation2D.hpp>

%{
    #include <rw/math/Wrench6D.hpp>
%}
%include <rw/math/Wrench6D.hpp>

FRIEND_OPERATOR(rw::math::Transform3D<float>, rw::math::Wrench6D<float>, *);
FRIEND_OPERATOR(rw::math::Transform3D<double>, rw::math::Wrench6D<double>, *);
FRIEND_OPERATOR(rw::math::Vector3D<double>, rw::math::Wrench6D<double>, *);
FRIEND_OPERATOR(rw::math::Vector3D<float>, rw::math::Wrench6D<float>, *);
FRIEND_OPERATOR(rw::math::Rotation3D<double>, rw::math::Wrench6D<double>, *);
FRIEND_OPERATOR(rw::math::Rotation3D<float>, rw::math::Wrench6D<float>, *);

%ignore rw::math::Rotation3D::e() const;
%ignore rw::math::Rotation3D::multiply() const;
%ignore rw::math::Rotation3D::inverse(bool) const;
%{
    #include <rw/math/Rotation3D.hpp>
%}
%include <rw/math/Rotation3D.hpp>

%{
    #include <rw/math/RPY.hpp>
%}
%include <rw/math/RPY.hpp>

%{
    #include <rw/math/Statistics.hpp>
%}
%include <rw/math/Statistics.hpp>

%ignore rw::math::Transform3D::R() const;
%ignore rw::math::Transform3D::P() const;
%{
    #include <rw/math/Transform3D.hpp>
%}
%include <rw/math/Transform3D.hpp>
%template(inverse) rw::math::inverse<double>;
%template(inverse) rw::math::inverse<float>;

%{
    #include <rw/math/Transform3DVector.hpp>
%}
%include <rw/math/Transform3DVector.hpp>

%ignore rw::math::Vector::e() const;
%{
    #include <rw/math/Vector.hpp>
%}
%include <rw/math/Vector.hpp>

%{
    #include <rw/math/Vector2D.hpp>
%}
%include <rw/math/Vector2D.hpp>
%template(VectorVector2D) std::vector<rw::math::Vector2D<double>>;
%template(VectorVector2D_f) std::vector<rw::math::Vector2D<float>>;

%ignore rw::math::Vector3D::e() const;
#if defined(SWIGPYTHON)
    %rename(copy) rw::math::Vector3D::operator=;
    %ignore operator* (double s, const Vector3D<double>& v);
    %ignore operator* (float s, const Vector3D<float>& v);
#endif 
%{
    #include <rw/math/Vector3D.hpp>
%}
%include <rw/math/Vector3D.hpp>
%template(VectorVector3D) std::vector<rw::math::Vector3D<double>>;
%template(VectorVector3D_f) std::vector<rw::math::Vector3D<float>>;
NAMED_OWNEDPTR(VectorVector3D,std::vector<rw::math::Vector3D<double>>);
NAMED_OWNEDPTR(VectorVector3D_f,std::vector<rw::math::Vector3D<float>>);


%rename(copy) rw::math::VectorND::operator=;
%ignore rw::math::VectorND::e() const;
%{
    #include <rw/math/VectorND.hpp>
%}
%include <rw/math/VectorND.hpp>

%{
    #include <rw/math/VelocityScrew6D.hpp>
%}
%include <rw/math/VelocityScrew6D.hpp>
FRIEND_OPERATOR(rw::math::Transform3D<float>, rw::math::VelocityScrew6D<float>, *);
FRIEND_OPERATOR(rw::math::Transform3D<double>, rw::math::VelocityScrew6D<double>, *);
FRIEND_OPERATOR(rw::math::Vector3D<double>, rw::math::VelocityScrew6D<double>, *);
FRIEND_OPERATOR(rw::math::Vector3D<float>, rw::math::VelocityScrew6D<float>, *);
FRIEND_OPERATOR(rw::math::Rotation3D<double>, rw::math::VelocityScrew6D<double>, *);
FRIEND_OPERATOR(rw::math::Rotation3D<float>, rw::math::VelocityScrew6D<float>, *);


// FUNCTION CASTS

%template (cast) rw::math::cast<float,double>;
%template (cast) rw::math::cast<double,float>;
%template (inverse) rw::math::inverse<double>;
%template (inverse) rw::math::inverse<float>;
%template (transpose) rw::math::transpose<double>;
%template (transpose) rw::math::transpose<float>;

/*
%template (ln) rw::math::ln<float>;
%template (ln) rw::math::ln<double>;
%template (exp) rw::math::exp<float>;
%template (exp) rw::math::exp<double>;
%template (pow) rw::math::pow<float>;
%template (pow) rw::math::pow<double>;
*/


%template(dot) rw::math::dot<double>;
%template(dot) rw::math::dot<float>;
%template(cross) rw::math::cross<double>;
%template(cross) rw::math::cross<float>;
%template(angle) rw::math::angle<double>;
%template(angle) rw::math::angle<float>;
%template(normalize) rw::math::normalize<double>;
%template(normalize) rw::math::normalize<float>;

%template (norm1) rw::math::norm1<float>;
%template (norm1) rw::math::norm1<double>;
%template (norm2) rw::math::norm2<float>;
%template (norm2) rw::math::norm2<double>;
%template (cross) rw::math::cross<double>;
%template (cross) rw::math::cross<float>;
%template (normInf) rw::math::normInf<float>;
%template (normInf) rw::math::normInf<double>;
%template (castToFloat) rw::math::cast<float,double>;
%template (castToDouble) rw::math::cast<double,float>;