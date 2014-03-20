#if !defined(__DEC_SUITESPARSE_LINEAR_ALGEBRA__)
#define __DEC_SUITESPARSE_LINEAR_ALGEBRA__

#include "Real.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"

struct SuiteSparseSolver
{
    public:
        typedef DDG::DenseMatrix<DDG::Real> Vector;
        typedef DDG::SparseMatrix<DDG::Real> Matrix;

        SuiteSparseSolver& compute(const Matrix& _matrix)
        {
            matrix = _matrix;
            return *this;
        }

        Vector solve(const Vector& input) const
        {
            Matrix matrix_copy(matrix);
            Vector input_copy(input);
            Vector solution;
            DDG::solve<DDG::Real>(matrix_copy, input_copy, solution);
            return solution;
        }
    protected:
        Matrix matrix;
};

struct SuiteSparseLinearAlgebra
{
    typedef unsigned int Index;
    typedef DDG::Real Scalar;
    typedef DDG::DenseMatrix<DDG::Real> Vector;
    typedef DDG::SparseMatrix<DDG::Real> Matrix;
    typedef SuiteSparseSolver Solver;
};

#endif

