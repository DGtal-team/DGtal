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
 * @file CLocalPremetric.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/01
 *
 * Header file for concept CLocalPremetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLocalPremetric_RECURSES)
#error Recursive header files inclusion detected in CLocalPremetric.h
#else // defined(CLocalPremetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLocalPremetric_RECURSES

#if !defined CLocalPremetric_h
/** Prevents repeated inclusion of headers. */
#define CLocalPremetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/base/CQuantity.h"
#include "DGtal/geometry/volumes/distance/CLocalPremetric.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CLocalPremetric
/**
Description of \b concept '\b CLocalPremetric' <p>
@ingroup Concepts
@brief Aim: Defines the concept of local premetric. Models of such
concept should implement a distance method which returns positive
values for the distance between  @a aPoint and  @a aPoint + @a
aDirection. 

Being a premetric, models should be such that the distance  at a point
in a given direction is equal to zero if and only if the direction is
a null vector.


### Refinement of 
  - boost::CopyConstructible
  - boost::Assignable

### Associated types :
 
 - @e Space: type of space on which the premetric is defined (model of CSpace)
 - @e Value: the value type of the metric (model of CQuantity) 

### Notation
 - \e X : A type that is a model of CLocalPremetric
 - \e x, \e y : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|-------------------------------------------|-------------------|---------------|--------------|-----------|----------------|------------|
| local distance computation      | v = x.local( aPoint,aDirection)  | aPoint of type @a Space::Point and aDirection of type @a Space::Vector |  v of type @a Value  |     | returns the distance at point @a aPoint in the direction @a aDirection. @a aDirection == null vector if and only if v==0      |                |    -         |

### Invariants

### Models

ExactPredicateLpSeparableMetric, InexactPredicateLpSeparableMetric

### Notes

@tparam T the type that should be a model of CLocalPremetric.
 */
template <typename T>
struct CLocalPremetric: boost::CopyConstructible<T>, boost::Assignable<T>
{
  // ----------------------- Concept checks ------------------------------
public:
  typedef typename T::Space Space;
  typedef typename T::Space::Point Point;
  typedef typename T::Space::Vector Vector;
  typedef typename T::Value Value;
    
  BOOST_CONCEPT_ASSERT(( CSpace< Space > ));
  BOOST_CONCEPT_ASSERT(( CQuantity< Value > ));

  BOOST_CONCEPT_USAGE( CLocalPremetric )
  {
    checkConstConstraints();
  }
  
  void checkConstConstraints() const
  {
    // const method dummyConst should take parameter myA of type A and return
    // something of type B
    ConceptUtils::sameType( myValue, myX.local( myPoint , myDirection ) );
  }
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  Point myPoint;
  Vector myDirection;
  Value myValue;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CLocalPremetric

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLocalPremetric_h

#undef CLocalPremetric_RECURSES
#endif // else defined(CLocalPremetric_RECURSES)
