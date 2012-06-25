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
 * @file CSegmentComputerFunctor.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/21
 *
 * Header file for concept CSegmentComputerFunctor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSegmentComputerFunctor_RECURSES)
#error Recursive header files inclusion detected in CSegmentComputerFunctor.h
#else // defined(CSegmentComputerFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSegmentComputerFunctor_RECURSES

#if !defined CSegmentComputerFunctor_h
/** Prevents repeated inclusion of headers. */
#define CSegmentComputerFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CQuantity.h"
#include "DGtal/geometry/curves/CSegment.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CSegmentComputerFunctor
/**
Description of \b concept '\b CSegmentComputerFunctor' <p>
@ingroup Concepts
@brief Aim: This concept gather all local geometrical estimators, 
which can return a quantity (at a given grid step @e h), computed 
at (the element pointed by) an iterator @e it from the model 
recognized by a segment computer @e sc. Moreover, in order to have
a better description of the local geometry, a boolean @e b1 
(resp. @e b2) indicates if the maximal segment detected by @a sc
intersects the previous (resp. next) maximal segment.  

### Refinement of 
- boost::DefaultConstructible
- boost::CopyConstructible
- boost::Assignable

### Associated types :
- SegmentComputer
- Quantity

### Notation
 - \e X : A type that is a model of CSegmentComputerFunctor
 - \e x : object of type X
 - \e h : double
 - \e sc : object of type SegmentComputer
 - \e it : object of type SegmentComputer::ConstIterator
 - \e b1, b2 : bool
 
### Definitions

### Valid expressions and semantics

| Name     | Expression          | Type requirements | Return type | Precondition | Semantics                       | Post condition | Complexity      |
|----------+---------------------+-------------------+-------------+--------------+---------------------------------+----------------+-----------------|
| init     | x.init( h )         |                   | void        | h > 0        | Initialization of the grid step |                | constant        |
| function | x( it, sc, b1, b2 ) |                   | Quantity    |              | Estimation of the quantity      |                | model dependant |
|          |                     |                   |             |              |                                 |                |                 |

### Invariants

### Models

   TangentFromDSSFunctor, TangentAngleFromDSSFunctor, CurvatureFromDSSFunctor.

### Notes

@tparam T the type that should be a model of CSegmentComputerFunctor.
 */
template <typename T>
struct CSegmentComputerFunctor: 
boost::DefaultConstructible<T>, boost::CopyConstructible<T>, boost::Assignable<T>
{
    // ----------------------- Concept checks ------------------------------
public:

    typedef typename T::SegmentComputer SegmentComputer;
    BOOST_CONCEPT_ASSERT(( CSegment< SegmentComputer > ));

    typedef typename T::Quantity Quantity;
    BOOST_CONCEPT_ASSERT(( CQuantity< Quantity > ));

    BOOST_CONCEPT_USAGE( CSegmentComputerFunctor )
    {

	myX.init(myH); 

        checkConstConstraints();
    }

    void checkConstConstraints() const
    {

        ConceptUtils::sameType( myQ, myX.operator()( myI, mySC, myB, myB ) );

    }
    // ------------------------- Private Datas --------------------------------
private:
    T myX;
    double myH; 
    Quantity myQ;
    SegmentComputer mySC;
    typename SegmentComputer::ConstIterator myI; 
    bool myB; 

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CSegmentComputerFunctor

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSegmentComputerFunctor_h

#undef CSegmentComputerFunctor_RECURSES
#endif // else defined(CSegmentComputerFunctor_RECURSES)
