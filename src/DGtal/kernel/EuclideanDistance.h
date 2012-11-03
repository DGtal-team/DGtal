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
 * @file EuclideanDistance.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/02
 *
 * Header file for module EuclideanDistance.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(EuclideanDistance_RECURSES)
#error Recursive header files inclusion detected in EuclideanDistance.h
#else // defined(EuclideanDistance_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EuclideanDistance_RECURSES

#if !defined EuclideanDistance_h
/** Prevents repeated inclusion of headers. */
#define EuclideanDistance_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanDistance
  /**
   * Description of template class 'EuclideanDistance' <p> \brief Aim:
   * A functor object that computes the Euclidean distance between two
   * points. It is a functor V x V -> S, where V is TRealPoint and S
   * is TRealPoint::Coordinate. This object does not hold any data.
   *
   * @tparam a model of CRealPoint, i.e. it has an inner type
   * Coordinate, a static dimension, iterators to get the coordinate
   * values, and the coordinate should support multiplication,
   * addition, and square root.
   */
  template <typename TRealPoint>
  struct EuclideanDistance
  {
    // ----------------------- Standard services ------------------------------
  public:
    typedef TRealPoint RealPoint;
    typedef typename RealPoint::Coordinate Coordinate;
    typedef RealPoint Argument;
    typedef RealPoint Argument1;
    typedef RealPoint Argument2;
    typedef Coordinate Value;

    /**
       @param arg1 any point.
       @param arg2 any point.
       @return the Euclidean distance between the two points.
    */
    Value operator()( const Argument1 & arg1, const Argument2 & arg2 ) const;

  }; // end of class EuclideanDistance


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/EuclideanDistance.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined EuclideanDistance_h

#undef EuclideanDistance_RECURSES
#endif // else defined(EuclideanDistance_RECURSES)
