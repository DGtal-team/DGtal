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
 * @file BoundedLatticePolytopeCounter.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/06/17
 *
 * Header file for module BoundedLatticePolytopeCounter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(BoundedLatticePolytopeCounter_RECURSES)
#error Recursive header files inclusion detected in BoundedLatticePolytopeCounter.h
#else // defined(BoundedLatticePolytopeCounter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BoundedLatticePolytopeCounter_RECURSES


#if !defined BoundedLatticePolytopeCounter_h
/** Prevents repeated inclusion of headers. */
#define BoundedLatticePolytopeCounter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class BoundedLatticePolytopeCounter
  /**
     Description of template class 'BoundedLatticePolytopeCounter' <p> \brief
     Aim: Represents an nD lattice polytope, i.e. a convex polyhedron

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable. 

     @tparam TSpace an arbitrary model of CSpace.
   */
  template < typename TSpace >
  class BoundedLatticePolytopeCounter 
  {
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));
  public:
    typedef BoundedLatticePolytopeCounter<TSpace>  Self;
    typedef TSpace                                 Space;
    typedef BoundedLatticePolytope<TSpace>         Polytope;
    using Integer          = typename Polytope::Integer;
    using Point            = typename Polytope::Point;
    using Vector           = typename Polytope::Vector;
    using InequalityMatrix = typename Polytope::InequalityMatrix;
    using InequalityVector = typename Polytope::InequalityVector;
    using Domain           = typename Polytope::Domain;
    using HalfSpace        = typename Polytope::HalfSpace;
    using BigInteger       = typename Polytope::BigInteger;
    using Interval         = std::pair<Integer,Integer>;
    static const Dimension dimension = Space::dimension;

    BoundedLatticePolytopeCounter() = default;
    BoundedLatticePolytopeCounter( const Polytope& P );
    void init( const Polytope* ptrP );

    /// Computes the intersection of the lattice points of the
    /// infinite line going through point \a p along axis \a a and the
    /// current polytope, returned as an interval `[b,e)`, where `b`is
    /// the \a a-th coordinate of the first lattice point in common,
    /// while `e` is the \a a-th coordinate after the last lattice
    /// point in common.
    ///
    /// @param p any point with the current domain
    /// @param a any axis between 0 (included) and `dimension` (excluded).
    ///
    /// @return the interval `[b,e)` of intersection, which is such
    /// that `b==e` when there is no intersection.
    Interval intersectionIntervalAlongAxis( Point p, Dimension a ) const;

    Integer countAlongAxis( Dimension a ) const;

    /// @return the most elongated axis of the bounding box of the
    /// current polytope.
    Dimension longestAxis() const;
    
    const Polytope* myPolytope;
    Point myLower;
    Point myUpper;
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "BoundedLatticePolytopeCounter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BoundedLatticePolytopeCounter_h

#undef BoundedLatticePolytopeCounter_RECURSES
#endif // else defined(BoundedLatticePolytopeCounter_RECURSES)
    
