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
 * @file CLinearAlgebraContainer.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/20
 *
 * Header file for concept CLinearAlgebraContainer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLinearAlgebraContainer_RECURSES)
#error Recursive header files inclusion detected in CLinearAlgebraContainer.h
#else // defined(CLinearAlgebraContainer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLinearAlgebraContainer_RECURSES

#if !defined CLinearAlgebraContainer_h
/** Prevents repeated inclusion of headers. */
#define CLinearAlgebraContainer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CLinearAlgebraContainer
/**
Description of \b concept '\b CLinearAlgebraContainer' <p>
@ingroup Concepts
@brief Aim:
Base Concept for linear algebra container

### Refinement of
 - boost::Assignable
 - boost::DefaultConstructible

### Associated types :
 - \c Scalar: Scalar type used for external multiplication and internal type representation.
 - \c Index: Random access index type.

### Notation
 - \e LinearAlgebraContainer : A type that is a model of CLinearAlgebraContainer
 - \e x, \e y : const object of type LinearAlgebraContainer
 - \e z : object of type LinearAlgebraContainer
 - \e a : object of type LinearAlgebraContainer::Scalar
 - \e i : object of type LinearAlgebraContainer::Index

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Reset container content      |  \a z.clear()         |                   |               |              |           |                |            |
| Number of rows      | \a x.rows()           |                   |  \c Index            |              |           |                |            |
| Number of columns      |  \a x.cols()           |                   | \c Index              |              |           |                |            |
| Addition      | z = x + y           |                  | \c LinearAlgebraContainer              |              |           |                |            |
| Substraction      | z = x - y           |                   | \c LinearAlgebraContainer              |              |           |                |            |
| External multiplication      | z = a * x           |                   |  \c LinearAlgebraContainer               |              |           |                |            |

### Invariants

### Models

### Notes

@tparam T the type that should be a model of CLinearAlgebraContainer.
 */
template <typename T>
struct CLinearAlgebraContainer : boost::Assignable<T>, boost::DefaultConstructible<T>
{
    // ----------------------- Concept checks ------------------------------
public:
		typedef typename T::Scalar Scalar;
		typedef typename T::Index Index;

    BOOST_CONCEPT_USAGE( CLinearAlgebraContainer )
    {
				z.clear();
				ConceptUtils::sameType(z, T(x + y));
				ConceptUtils::sameType(z, T(x - y));
				ConceptUtils::sameType(z, T(a * x));
				ConceptUtils::sameType(ii, x.rows());
				ConceptUtils::sameType(ii, x.cols());
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
		Index ii;

    // ------------------------- Internals ------------------------------------
private:


}; // end of concept CLinearAlgebraContainer

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLinearAlgebraContainer_h

#undef CLinearAlgebraContainer_RECURSES
#endif // else defined(CLinearAlgebraContainer_RECURSES)
