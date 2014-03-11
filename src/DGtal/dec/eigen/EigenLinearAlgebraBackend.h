#if !defined(__DEC_EIGEN_LINEAR_ALGEBRA_BACKEND_H__)
#define __DEC_EIGEN_LINEAR_ALGEBRA_BACKEND_H__

#include <Eigen/Dense>
#include <Eigen/Sparse>

struct EigenLinearAlgebraBackend
{
    typedef size_t Index;
    typedef double Scalar;
    typedef Eigen::VectorXd KFormContainer;
    typedef Eigen::MatrixXd LinearOperatorContainer;
    typedef Eigen::VectorXd VectorFieldCoordinate;
};

#endif

