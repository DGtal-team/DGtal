#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA__)
#define __DEC_EIGEN_LINEAR_ALGEBRA__

#define EIGEN_DENSEBASE_PLUGIN "DGtal/math/linalg/EigenDenseBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "DGtal/math/linalg/EigenSparseMatrixAddons.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseQR>
#include <Eigen/IterativeLinearSolvers>

struct EigenDenseLinearAlgebra
{
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
};

struct EigenSparseLinearAlgebra
{
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<Vector::Scalar, Eigen::ColMajor> Matrix;
};

#endif

