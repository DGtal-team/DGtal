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
 * @file TangencyComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/07/16
 *
 * Header file for module TangencyComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TangencyComputer_RECURSES)
#error Recursive header files inclusion detected in TangencyComputer.h
#else // defined(TangencyComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TangencyComputer_RECURSES

#if !defined TangencyComputer_h
/** Prevents repeated inclusion of headers. */
#define TangencyComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <string>
#include <limits>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clone.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class TangencyComputer
  /**
     Description of template class 'TangencyComputer' <p> \brief Aim:
     A class that computes tangency to a given digital set. It
     provides services to compute all the cotangent points to a given
     point, or to compute shortest paths.

     @see moduleDigitalConvexityApplications

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   */
  template < typename TKSpace >
  class TangencyComputer
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

  public:
    typedef TangencyComputer< TKSpace > Self;
    typedef TKSpace                     KSpace;
    typedef typename KSpace::Space      Space;
    typedef typename KSpace::Point      Point;
    typedef typename KSpace::Vector     Vector;
    typedef HyperRectDomain< Space >    Domain;
    typedef std::size_t                 Index;
    typedef std::size_t                 Size;

    // ------------------------- Standard services --------------------------------
  public:
    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /// Constructor. The object is invalid.
    TangencyComputer() = default;

    /// Copy constructor.
    /// @param other the object to clone.
    TangencyComputer( const Self& other ) = default;

    /// Move constructor.
    /// @param other the object to move
    TangencyComputer( Self&& other ) = default;

    /// Assigment
    /// @param other the object to clone
    /// @return a reference to 'this'
    Self& operator=( const Self& other ) = default;

    /// Move assigment
    /// @param other the object to clone
    /// @return a reference to 'this'
    Self& operator=( Self&& other ) = default;
    
    /// Constructor from digital space.
    /// @param aK the input Khalimsky space, which is cloned.
    TangencyComputer( Clone< KSpace > aK );

    /// Init the object with the points of the range itB, itE
    /// Points within this range are indexed in the same order.
    ///
    /// @tparam PointIterator any model of ForwardIterator on Point.
    /// @param[in] itB an iterator pointing at the beginning of the range.
    /// @param[in] itE an iterator pointing after the end of the range.
    template < typename PointIterator >
    void init( PointIterator itB, PointIterator itE );

    /// @}

    // ------------------------- Accessors services --------------------------------
  public:
    /// @name Accessors services
    /// @{
    
    /// @return a const reference to the Khalimsky space.
    const KSpace& space() const
    { return myK; }

    /// @return a const reference to the points defining the digital set.
    const std::vector< Point >& points() const
    { return myX; }
      
    /// @return a const reference to the cell geometry of the current digital set.
    const CellGeometry< KSpace >& cellCover() const
    { return myCellCover; }

    /// @}

    // ------------------------- Tangency services --------------------------------
  public:
    /// @name Tangency services
    /// @{
    
    /// Tells is two points are cotangent with respect to the current digital set.
    /// @param[in] a any point
    /// @param[in] b any point
    /// @return 'true' if and only if \a a and \a b are cotangent in this set.
    bool arePointsCotangent( const Point& a, const Point& b ) const;

    /// Extracts cotangent points by a breadth-first traversal.
    /// @param[in] a any point
    /// @return the indices of the other points of the shape that are cotangent to \a a.
    std::vector< Index >
    getCotangentPoints( const Point& a ) const;

    /// Extracts a subset of cotangent points by a breadth-first traversal.
    ///
    /// @param[in] a any point
    /// @param[in] to_avoid if 'to_avoid[ i ]' is true, then the point
    /// of index \a i is not visited by the bft.
    ///
    /// @return the indices of the other points of the shape that are cotangent to \a a.
    std::vector< Index >
    getCotangentPoints( const Point& a,
                        const std::vector< bool > & to_avoid ) const;

    /// Extracts a subset of cotangent points by a breadth-first
    /// traversal. Used for computing shortest paths.
    ///
    /// @param[in] a any point
    /// @param[in] to_avoid if 'to_avoid[ i ]' is true, then the point
    /// of index \a i is not visited by the bft.
    ///
    /// @param[in] distance an array storing for each point index its
    /// current distance to some source (used to prune points that
    /// won't induce shortest paths)
    ///
    /// @param[in] dmax this value is used to prune vertices in the
    /// bft. If it is greater or equal to \f$ \sqrt{d} \f$ where \a d
    /// is the dimension, the shortest path algorithm is guaranteed to
    /// output the correct result. If the value is smaller (down to
    /// 0.0), the algorithm is much faster but a few shortest path may
    /// be missed.
    ///
    /// @return the indices of the other points of the shape that are cotangent to \a a.
    std::vector< Index >
    getCotangentPoints( const Point& a,
                        const std::vector< bool > & to_avoid,
                        const std::vector< double >& distance,
                        double dmax = std::numeric_limits<double>::infinity() ) const;
  
    /// @}

    // ------------------------- Shortest paths services --------------------------------
  public:
    /// @name Shortest paths services
    /// @{

    double
    shortestPaths( std::vector< Index >&  ancestor,
                   std::vector< double >& distance,
                   Index source,
                   double max_distance = std::numeric_limits<double>::infinity(),
                   double secure = sqrt( KSpace::dimension ),
                   bool verbose = false ) const;
    
    /// @}
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// The cellular grid space where computations are done.
    KSpace myK;
    /// The digital convexity object used to check full convexity.
    DigitalConvexity< KSpace > myDConv;
    /// The vector of all vectors to neighbors (8 in 2D, 26 in 3D, etc).
    std::vector< Vector > myN;
    /// The vector of all distances to neighbors (8 in 2D, 26 in 3D,
    /// etc), that is the norm of each value of \ref myN.
    std::vector< double > myDN;
    /// The vector of points defining the digital shape under study.
    std::vector< Point >  myX;
    /// The cell geometry representing all the cells touching the digital shape. 
    CellGeometry< KSpace > myCellCover;
    /// A map giving for each point its index.
    std::unordered_map< Point, Index > myPt2Index;
    
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

    /// Precomputes some neighborhood tables at construction.
    void setUp();
    
  }; // end of class TangencyComputer

  /// @name Functions related to TangencyComputer (output)
  /// @{

  /**
   * Overloads 'operator<<' for displaying objects of class 'TangencyComputer'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'TangencyComputer' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out,
               const TangencyComputer<TKSpace> & object );

  /// @}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "TangencyComputer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TangencyComputer_h

#undef TangencyComputer_RECURSES
#endif // else defined(TangencyComputer_RECURSES)

  
