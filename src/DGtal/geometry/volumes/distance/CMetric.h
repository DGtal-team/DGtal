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
 * @file CMetric.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/01
 *
 * Header file for concept CMetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CMetric_RECURSES)
#error Recursive header files inclusion detected in CMetric.h
#else // defined(CMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CMetric_RECURSES

#if !defined CMetric_h
/** Prevents repeated inclusion of headers. */
#define CMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/volumes/distance/CLocalPremetric.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CMetric
/**
Description of \b concept '\b CMetric' <p>
@ingroup Concepts
@brief Aim: defines the concept of metrics.

In addition to CLocalPremetric requirements (non-negativity and coincidence axiom), CMetric models should
implement a distance function on points satisfying the metric
conditions:
 - d(x,y) == d(y,x) (symmetry)
 - d(x,y) <= d(x,z) + d(z,y) (triangle inequality)

For performance purposes, we ask the model to implement a closest() method to decide given two points which one is closer to a third one. This method can simply be implemented as a test "d(aOrigin,aP)<d(aOrigin,aQ)" (see below) but fast implementation can be expected without computing the distances.  


### Refinement of CLocalPremetric

### Associated types :

Inherited from CLocalPremetric:
 - @e Point: type of points associated with the underlying metric space.
 - @e Vector: type of vectors associated with the underlying metric space.
 - @e Value: the value type of the metric (model of CQuantity)


### Notation
 - \e X : A type that is a model of CMetric
 - \e x, \e y : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| distance computation | x(aPoint,anotherPoint) | @a aPoint and @a anotherPoint of type @a Point  |  a value of type @a Value   |              |  compute the distance between two points  |                |    -        |
| closest point test | closest(aOrigin, aP, aQ) | @a aOrigin, @a aP,@a aQ of type @a aPoint |   a value of type Closest | | decide between @a aP and @a aQ which one is closer to the origin. This functions returns either DGtal::ClosestFIRST if @a aP is closer, DGtal::ClosestSECOND if @a aQ is closer  and DGtal::ClosestBOTH if both are equidistant.| | - |
  

### Invariants

### Models

ExactPredicateLpSeparableMetric, InexactPredicateLpSeparableMetric

### Notes

@tparam T the type that should be a model of CMetric.
 */
template <typename T>
struct CMetric: CLocalPremetric<T>
{
    // ----------------------- Concept checks ------------------------------
public:
  typedef typename T::Point Point;
  typedef typename T::Vector Vector;
  typedef typename T::Value Value;
  
  BOOST_CONCEPT_USAGE( CMetric )
  {
    checkConstConstraints();
  }
  
  void checkConstConstraints() const
  {
    // const method dummyConst should take parameter myA of type A and return
    // something of type B
    ConceptUtils::sameType( myValue, myX.operator()( myPoint , myPoint2 ) );
    ConceptUtils::sameType( myClosest, myX.closest( myPoint , myPoint2,myPoint3 ) );
 }
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  Point myPoint, myPoint2, myPoint3;
  Value myValue;
  DGtal::Closest myClosest;
    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CMetric

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CMetric_h

#undef CMetric_RECURSES
#endif // else defined(CMetric_RECURSES)
