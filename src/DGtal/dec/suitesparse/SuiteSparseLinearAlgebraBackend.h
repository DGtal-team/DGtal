#if !defined(__DEC_SUITESPARSE_LINEAR_ALGEBRA_BACKEND_H__)
#define __DEC_SUITESPARSE_LINEAR_ALGEBRA_BACKEND_H__

#include "Real.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"

struct SuiteSparseLinearAlgebraBackend
{
    typedef size_t Index; // FIXME should be imported from linear algebra backend
    typedef DDG::Real Scalar;
    typedef DDG::DenseMatrix<DDG::Real> KFormContainer;
    typedef DDG::SparseMatrix<DDG::Real> LinearOperatorContainer;
    typedef DDG::DenseMatrix<DDG::Real> VectorFieldCoordinate;
};

#endif

