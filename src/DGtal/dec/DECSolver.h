#if !defined(__DEC_SOLVER_H__)
#define __DEC_SOLVER_H__

#include "KForm.h"
#include "LinearOperator.h"
#include <DGtal/math/linalg/CLinearAlgebraSolver.h>

template <typename C, typename S, Order order_in, Duality duality_in, Order order_out, Duality duality_out>
struct DiscreteExteriorCalculusSolver
{
		typedef C Calculus;
		typedef S LinearAlgebraSolver;
		typedef typename Calculus::Vector Vector;
		typedef typename Calculus::Matrix Matrix;
		BOOST_CONCEPT_ASSERT(( CLinearAlgebraSolver<LinearAlgebraSolver, Vector, Matrix> ));

		typedef LinearOperator<Calculus, order_in, duality_in, order_out, duality_out> Operator;
		typedef KForm<Calculus, order_in, duality_in> SolutionKForm;
		typedef KForm<Calculus, order_out, duality_out> InputKForm;

		DiscreteExteriorCalculusSolver& compute(const Operator& linear_operator);
		SolutionKForm solve(const InputKForm& input_kform) const;

		LinearAlgebraSolver solver;
};

#include "DECSolver.ih"

#endif


