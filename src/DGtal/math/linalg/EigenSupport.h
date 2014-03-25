#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA_SUPPORT__)
#define __DEC_EIGEN_LINEAR_ALGEBRA_SUPPORT__

#define EIGEN_DENSEBASE_PLUGIN "DGtal/math/linalg/EigenDenseBaseAddons.h"
#define EIGEN_SPARSEMATRIX_PLUGIN "DGtal/math/linalg/EigenSparseMatrixAddons.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

struct EigenDenseLinearAlgebraBackend
{
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
};

struct EigenSparseLinearAlgebraBackend
{
    typedef Eigen::VectorXd Vector;
    typedef Eigen::SparseMatrix<Vector::Scalar, Eigen::ColMajor> Matrix;
};

std::ostream& operator<<(std::ostream& os, const Eigen::ComputationInfo& info)
{
    os << "solve_info = ";

    switch (info)
    {
        case Eigen::Success:
            os << "success";
            break;
        case Eigen::NumericalIssue:
            os << "numerical_issue";
            break;
        case Eigen::NoConvergence:
            os << "no_convergence";
            break;
        case Eigen::InvalidInput:
            os << "invalid_input";
            break;
    }

    return os;
}

#endif

