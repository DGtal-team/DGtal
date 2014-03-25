/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file CLinearAlgebraSolver.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/25
 *
 * Header file for concept CLinearAlgebraSolver.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLinearAlgebraSolver_RECURSES)
#error Recursive header files inclusion detected in CLinearAlgebraSolver.h
#else // defined(CLinearAlgebraSolver_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLinearAlgebraSolver_RECURSES

#if !defined CLinearAlgebraSolver_h
/** Prevents repeated inclusion of headers. */
#define CLinearAlgebraSolver_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/CLinearAlgebra.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CLinearAlgebraSolver
/**
Description of \b concept '\b CLinearAlgebraSolver' <p>
@ingroup Concepts
@brief Aim:
Describe a linear solver defined over a linear algebra.
Problems are of the form:

\e a * \e x = \e y

where \e a type is a model of CMatrix and \e x, \e y type is a model of CVector.
Matrix and vector types should be a model of CLinearAlgebra.

### Refinement of
 - boost::Assignable<T>

### Associated types

### Notation
 - \c Solver : A type that is a model of CLinearAlgebraSolver
 - \c Vector : A type that is a model of CVector
 - \c Matrix : A type that is a model of CMatrix
 - \e x, \e y : object of type \c Vector
 - \e a : object of type \c Matrix

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Problem factorization / matrix setter      | solver.compute(\e a)           |                   | \c Solver&              |              |           | return *this.               |            |
| Problem resolution / vector setter      | x = solver.solve(\e y)           |                   | \c Vector              |              |           |                |            |
|       |            |                   |               |              |           |                |            |

### Invariants

### Models

### Notes

@tparam T the type that should be a model of CLinearAlgebraSolver.
@tparam V the type that should be a model of CVector
@tparam M the type that should be a model of CMatrix
 */
template <typename T, typename V, typename M>
struct CLinearAlgebraSolver : boost::Assignable<T>, CLinearAlgebra<V, M>
{
    // ----------------------- Concept checks ------------------------------
public:
    BOOST_CONCEPT_USAGE( CLinearAlgebraSolver )
    {
        const_solver = solver.compute(a);
        x = const_solver.solve(y);
    }
    // ------------------------- Private Datas --------------------------------
private:
    const T const_solver;
    T solver;
    const M a;
    const V y;
    V x;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CLinearAlgebraSolver

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLinearAlgebraSolver_h

#undef CLinearAlgebraSolver_RECURSES
#endif // else defined(CLinearAlgebraSolver_RECURSES)
