#if !defined(__DEC_SUITESPARSE_LINEAR_ALGEBRA__)
#define __DEC_SUITESPARSE_LINEAR_ALGEBRA__

#include "Real.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"

struct SuiteSparseLinearAlgebra
{
    typedef unsigned int Index;
    typedef DDG::Real Scalar;
    typedef DDG::DenseMatrix<DDG::Real> Vector;
    typedef DDG::SparseMatrix<DDG::Real> Matrix;
};

#endif

