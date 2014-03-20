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
 * @file CVector.h
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/20
 *
 * Header file for concept CVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CVector_RECURSES)
#error Recursive header files inclusion detected in CVector.h
#else // defined(CVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CVector_RECURSES

#if !defined CVector_h
/** Prevents repeated inclusion of headers. */
#define CVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/CLinearAlgebraContainer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CVector
/**
Description of \b concept '\b CVector' <p>
@ingroup Concepts
@brief Aim:

### Refinement of

### Associated types :

### Notation
 - \e X : A type that is a model of CVector
 - \e x, \e y : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

### Invariants

### Models

### Notes

@tparam T the type that should be a model of CVector.
 */
template <typename T>
struct CVector : CLinearAlgebraContainer<T>
{
    // ----------------------- Concept checks ------------------------------
public:
    typedef typename T::Scalar Scalar;
    typedef typename T::Index Index;

    BOOST_CONCEPT_USAGE( CVector )
    {
        ConceptUtils::sameType(a, x(ii));
        ConceptUtils::sameType(a_ref, z(ii));
        checkConstConstraints();
    }
    void checkConstConstraints() const
    {
    }
    // ------------------------- Private Datas --------------------------------
private:
    const T x;
    T z;
    Scalar a;
    Scalar& a_ref;
    Index ii;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CVector

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CVector_h

#undef CVector_RECURSES
#endif // else defined(CVector_RECURSES)
