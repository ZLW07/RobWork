
%import(module=rwlibs/swig/sdurw_core) <rwlibs/swig/ext_i/std.i>
%{
    #include <Eigen/Core>
%}
namespace Eigen{
    template<typename _Scalar, int _Rows, int _Cols>
    class Matrix
    {
      public:
        
        Matrix(int dimx, int dimy);
        
        #if !defined(SWIGJAVA)
            double& operator()(size_t row, size_t column);
            const double& operator()(size_t row, size_t column) const;
        #endif

        const Matrix operator+(const Matrix& wrench) const;    
        const Matrix operator-(const Matrix& wrench) const;
        //const Matrix operator*(const Matrix& wrench) const;

        int rows() const;
        int cols() const; 
        
        %extend {
            
            #if !defined(SWIGJAVA)
                _Scalar& elem(int x, int y){
                    return (*$self)(x,y);
                }
            #endif
            
            /* These accesors are neccesary because Python does not allow
            lvalues consisting of access operators calls (cannot assign to a function call).
            Moreover, it's not possible to dereference a pointer obtained from function returning a reference. */
            _Scalar get(int x, int y) {
                return (*$self)(x, y);
            }
            
            void set(int x, int y, _Scalar value) {
                (*$self)(x, y) = value;
            }
        }
        MATRIXOPERATOR(_Scalar);
    };

    template<typename T>
    class Quaternion{

    };
}

#if  defined(SWIGPYTHON) && RW_USE_NUMPY
%include <rwlibs/swig/ext_i/eigen.i>

#define dxx double,-1,-1
#define d22 double,2,2
#define d33 double,3,3
#define d44 double,4,4

#define fxx float,-1,-1
#define f22 float,2,2
#define f33 float,3,3
#define f44 float,4,4

#define dx1 double,-1,1
#define d21 double,2,1
#define d31 double,3,1
#define d41 double,4,1
#define d61 double,6,1
#define d71 double,7,1

#define fx1 float,-1,1
#define f21 flaot,2,1
#define f31 float,3,1
#define f41 float,4,1
#define f61 float,6,1
#define f71 float,7,1

#define f13 float,1,3
#define d13 double,1,3
#define cd31 std::complex<double>,3,1
#define cd13 std::complex<double>,1,3
#define cd33 std::complex<double>,3,3

%eigen_typemaps(Eigen::Matrix<dxx>);
%eigen_typemaps(Eigen::Matrix<d22>);
%eigen_typemaps(Eigen::Matrix<d33>);
%eigen_typemaps(Eigen::Matrix<d44>);

%eigen_typemaps(Eigen::Matrix<fxx>);
%eigen_typemaps(Eigen::Matrix<f22>);
%eigen_typemaps(Eigen::Matrix<f33>);
%eigen_typemaps(Eigen::Matrix<f44>);

%eigen_typemaps(Eigen::Matrix<dx1>);
%eigen_typemaps(Eigen::Matrix<d21>);
%eigen_typemaps(Eigen::Matrix<d31>);
%eigen_typemaps(Eigen::Matrix<d41>);
%eigen_typemaps(Eigen::Matrix<d61>);
%eigen_typemaps(Eigen::Matrix<d71>);

%eigen_typemaps(Eigen::Matrix<fx1>);
%eigen_typemaps(Eigen::Matrix<f21>);
%eigen_typemaps(Eigen::Matrix<f31>);
%eigen_typemaps(Eigen::Matrix<f41>);
%eigen_typemaps(Eigen::Matrix<f61>);
%eigen_typemaps(Eigen::Matrix<f71>);

%eigen_typemaps(Eigen::Matrix<f13>);
%eigen_typemaps(Eigen::Matrix<d13>);
%eigen_typemaps(Eigen::Matrix<cd31>);
%eigen_typemaps(Eigen::Matrix<cd13>);
%eigen_typemaps(Eigen::Matrix<cd33>);

#else

%template(EigenMatrixXf) Eigen::Matrix<float,-1,-1>;
%template(EigenMatrixXd) Eigen::Matrix<double,-1,-1>;
%template(EigenMatrix2f) Eigen::Matrix<float,2,2>;
%template(EigenMatrix2d) Eigen::Matrix<double,2,2>;
%template(EigenMatrix3f) Eigen::Matrix<float,3,3>;
%template(EigenMatrix3d) Eigen::Matrix<double,3,3>;
%template(EigenMatrix4f) Eigen::Matrix<float,4,4>;
%template(EigenMatrix4d) Eigen::Matrix<double,4,4>;


%template(EigenVectorXf) Eigen::Matrix<float,-1,1>;
%template(EigenVectorXd) Eigen::Matrix<double,-1,1>;
%template(EigenVector2f) Eigen::Matrix<float,2,1>;
%template(EigenVector2d) Eigen::Matrix<double,2,1>;
%template(EigenVector3f) Eigen::Matrix<float,3,1>;
%template(EigenVector3d) Eigen::Matrix<double,3,1>;
%template(EigenVector6f) Eigen::Matrix<float,6,1>;
%template(EigenVector6d) Eigen::Matrix<double,6,1>;
%template(EigenVector7f) Eigen::Matrix<float,7,1>;
%template(EigenVector7d) Eigen::Matrix<double,7,1>;

%template(EigenRowVector3f) Eigen::Matrix<float,1,3>;
%template(EigenRowVector3d) Eigen::Matrix<double,1,3>;
%template(EigenVector3id) Eigen::Matrix<std::complex<double>,3,1>;
%template(EigenRowVector3id) Eigen::Matrix<std::complex<double>,1,3>;
%template(EigenMatrix3id) Eigen::Matrix<std::complex<double>,3,3>;

#endif

#if ! SWIG_VERSION <= 0x030008
%template(EigenQuaterniond) Eigen::Quaternion<double>;
%template(EigenQuaternionf) Eigen::Quaternion<float>; 

%template(VectorEigenRowVector3f) std::vector<Eigen::Matrix<float,1,3>>;
%template(VectorEigenRowVector3d) std::vector<Eigen::Matrix<double,1,3>>;
%template(VectorEigenVector3f) std::vector<Eigen::Matrix<float,3,1>>;
%template(VectorEigenVector3d) std::vector<Eigen::Matrix<double,3,1>>;
%template(VectorEigenMatrix3f) std::vector<Eigen::Matrix<float,3,3>>;
%template(VectorEigenMatrix3d) std::vector<Eigen::Matrix<double,3,3>>;

%template(VectorEigenVector3id) std::vector<Eigen::Matrix<std::complex<double>,3,1>>;
%template(VectorEigenMatrix3id) std::vector<Eigen::Matrix<std::complex<double>,3,3>>;
#endif