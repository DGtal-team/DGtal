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
 * @file CLinearAlgebra.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/21
 *
 * Header file for concept CLinearAlgebra.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLinearAlgebra_RECURSES)
#error Recursive header files inclusion detected in CLinearAlgebra.h
#else // defined(CLinearAlgebra_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLinearAlgebra_RECURSES

#if !defined CLinearAlgebra_h
/** Prevents repeated inclusion of headers. */
#define CLinearAlgebra_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/CMatrix.h"
#include "DGtal/math/linalg/CVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CLinearAlgebra
/**
Description of \b concept '\b CLinearAlgebra' <p>
@ingroup Concepts
@brief Aim:
Check multiplication between static matrix and static vector

### Refinement of

### Associated types

### Notation
 - \c Vector : A type that is a model of CVector
 - \c Matrix : A type that is a model of CMatrix
 - \e x, \e y : object of type \c Vector
 - \e a, \e b, \e c : object of type \c Matrix

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Matrix vector multiplication      | x = a * y           |                   | \c Vector              |              |           |                |            |
| Matrix matrix multiplication      | c = a * b           |                   | \c Matrix              |              |           |                |            |

### Invariants

### Models

### Notes

@tparam M the type that should be a model of CMatrix
@tparam V the type that should be a model of CVector
 */
template <typename M, typename V>
struct CLinearAlgebra
{
    // ----------------------- Concept checks ------------------------------
public:
    BOOST_CONCEPT_ASSERT(( CVector<V> ));
    BOOST_CONCEPT_ASSERT(( CMatrix<M> ));

    BOOST_CONCEPT_USAGE( CLinearAlgebra )
    {
        x = a * y;
        c = a * b;
    }
    // ------------------------- Private Datas --------------------------------
private:
    M c;
    const M a, b;
    V x;
    const V y;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CLinearAlgebra

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLinearAlgebra_h

#undef CLinearAlgebra_RECURSES
#endif // else defined(CLinearAlgebra_RECURSES)
