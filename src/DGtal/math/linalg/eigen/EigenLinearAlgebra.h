#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA__)
#define __DEC_EIGEN_LINEAR_ALGEBRA__

#define EIGEN_DENSEBASE_PLUGIN "DGtal/math/linalg/eigen/DenseBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "DGtal/math/linalg/eigen/SparseMatrixAddons.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseQR>
#include <Eigen/IterativeLinearSolvers>

struct EigenDenseLinearAlgebra
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::ConjugateGradient<Matrix> Solver;
};

struct EigenSparseLinearAlgebra
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor> Matrix;
    //typedef Eigen::SparseQR<Matrix, Eigen::COLAMDOrdering<Index> > Solver;
    typedef Eigen::ConjugateGradient<Matrix> Solver;
};

#endif

