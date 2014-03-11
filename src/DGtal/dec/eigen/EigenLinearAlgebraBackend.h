#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA_BACKEND_H__)
#define __DEC_EIGEN_LINEAR_ALGEBRA_BACKEND_H__

#include <Eigen/Dense>
#include <Eigen/Sparse>

struct EigenDenseLinearAlgebraBackend
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd KFormContainer;
    typedef Eigen::MatrixXd LinearOperatorContainer;
    typedef Eigen::VectorXd VectorFieldCoordinate;
};

struct EigenSparseLinearAlgebraBackend
{
    typedef Eigen::VectorXd::Index Index;
    typedef Eigen::VectorXd::Scalar Scalar;
    typedef Eigen::VectorXd KFormContainer;
    typedef Eigen::SparseMatrix<Scalar> LinearOperatorContainer;
    typedef Eigen::VectorXd VectorFieldCoordinate;
};

#endif

