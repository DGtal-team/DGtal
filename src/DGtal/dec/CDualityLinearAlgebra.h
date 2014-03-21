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
 * @file CDualityLinearAlgebra.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/20
 *
 * Header file for concept CDualityLinearAlgebra.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDualityLinearAlgebra_RECURSES)
#error Recursive header files inclusion detected in CDualityLinearAlgebra.h
#else // defined(CDualityLinearAlgebra_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDualityLinearAlgebra_RECURSES

#if !defined CDualityLinearAlgebra_h
/** Prevents repeated inclusion of headers. */
#define CDualityLinearAlgebra_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/CVectorSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CDualityLinearAlgebra
/**
Description of \b concept '\b CDualityLinearAlgebra' <p>
@ingroup Concepts
@brief Aim:
Lift linear algebra container concept into the dec package.

### Refinement of

### Associated types
  - \c Container: Associated container, a model of concept CLinearAlgebraContainer.
  - \c Calculus: Associated discrete exterior calculus.

### Notation

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

### Invariants

### Models

### Notes

@tparam T the type that should be a model of CDualityLinearAlgebra.
*/
template <typename T>
struct CDualityLinearAlgebra
{
    // ----------------------- Concept checks ------------------------------
public:
    typedef typename T::Container Container;
    typedef typename T::Calculus Calculus;
    typedef typename Calculus::Scalar Scalar;

    BOOST_CONCEPT_ASSERT(( CVectorSpace<Container> ));

    BOOST_CONCEPT_USAGE( CDualityLinearAlgebra )
    {
        T t0(calculus_const_ref);
        T t1(calculus_const_ref, container_const_ref);
        ConceptUtils::sameType(z, T(x + y));
        //ConceptUtils::sameType(z, T(x - y));
        ConceptUtils::sameType(z, T(a * x));
        checkConstConstraints();
    }
    void checkConstConstraints() const
    {
    }
    // ------------------------- Private Datas --------------------------------
private:
    const T x,y;
    T z;
    Scalar a;
    const Container& container_const_ref;
    const Calculus& calculus_const_ref;
    Scalar scalar;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CDualityLinearAlgebra

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDualityLinearAlgebra_h

#undef CDualityLinearAlgebra_RECURSES
#endif // else defined(CDualityLinearAlgebra_RECURSES)
