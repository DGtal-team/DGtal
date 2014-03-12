#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA__)
#define __DEC_EIGEN_LINEAR_ALGEBRA__

#define EIGEN_DENSEBASE_PLUGIN "DGtal/dec/eigen/DenseBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "DGtal/dec/eigen/SparseMatrixAddons.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

struct EigenDenseLinearAlgebra
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
};

struct EigenSparseLinearAlgebra
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<Scalar> Matrix;
};

#endif

