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
 * @file DigitalConvexity.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/01/31
 *
 * Header file for module DigitalConvexity.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalConvexity_RECURSES)
#error Recursive header files inclusion detected in DigitalConvexity.h
#else // defined(DigitalConvexity_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalConvexity_RECURSES

#if !defined DigitalConvexity_h
/** Prevents repeated inclusion of headers. */
#define DigitalConvexity_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clone.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalConvexity
  /**
     Description of template class 'DigitalConvexity' <p> \brief Aim:
     A helper class to build polytopes from digital sets and to check
     digital k-convexity.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable. 

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   */
  template < typename TKSpace >
  class DigitalConvexity 
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

  public:
    typedef DigitalConvexity<TKSpace>       Self;
    typedef TKSpace                         KSpace;
    typedef typename KSpace::Integer        Integer;
    typedef typename KSpace::Point          Point;
    typedef typename KSpace::Vector         Vector;
    typedef typename KSpace::Cell           Cell;
    typedef typename KSpace::Space          Space;
    typedef DGtal::BoundedLatticePolytope< Space > Polytope;
    typedef DGtal::CellGeometry< KSpace >   CellGeometry;
    typedef std::vector<Point>              PointRange;
    
    static const Dimension dimension = KSpace::dimension;


    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /**
     * Destructor.
     */
    ~DigitalConvexity() = default;

    /**
     * Constructor. 
     */
    DigitalConvexity() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalConvexity ( const Self & other ) = default;

    /**
     * Constructor from cellular space.
     * @param K any cellular grid space.
     */
    DigitalConvexity( Clone<KSpace> K );

    /**
     * Constructor from lower and upper points.
     * @param lo the lowest point of the domain (bounding box for computations).
     * @param hi the highest point of the domain (bounding box for computations).
     */
    DigitalConvexity( Point lo, Point hi );
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// @}

    // ----------------------- Polytope services --------------------------------------
  public:
    /// @name Polytope services
    /// @{

    /**
     * Constructs the polytope from a simplex given as a range
     * [itB,itE) of lattice points.  Note that the range must contain
     * Space::dimension+1 points in general position.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param itB the start of the range of n+1 points defining the simplex.
     * @param itE past the end the range of n+1 points defining the simplex.
     */
    template <typename PointIterator>
    static
    Polytope makeSimplex( PointIterator itB, PointIterator itE );

    /**
     * Constructs the polytope from a simplex given as an initializer_list.
     *
     * @param l any list of d+1 points in general positions.
     * @pre Note that the list must contain Space::dimension+1 points
     * in general position.
     */
    static
    Polytope makeSimplex( std::initializer_list<Point> l );

    /// @param polytope any polytope.
    /// @return the range of digital points that belongs to the polytope.
    static
    PointRange insidePoints( const Polytope& polytope );

    /// @param polytope any polytope.
    /// @return the range of digital points that belongs to the interior of the polytope.
    static
    PointRange interiorPoints( const Polytope& polytope );

    /// @}


    // ----------------------- Cell geometry services -----------------------------------
  public:
    /// @name Cell geometry services
    /// @{

    /// Builds the cell geometry containing all the j-cells touching a
    /// point of [itB,itE), for i <= j <= k.
    ///
    /// @tparam PointIterator any model of input iterator on Points.
    /// @param itB start of a range of arbitrary points.
    /// @param itE past the end of a range of arbitrary points.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    template <typename PointIterator>
    CellGeometry makeCellCover( PointIterator itB, PointIterator itE,
				Dimension i = 0, Dimension k = KSpace::dimension  );

    /// Builds the cell geometry containing all the j-cells touching
    /// the polytope P, for i <= j <= k. It conbains thus all the
    /// j-cells intersecting the convex enveloppe of P.
    ///
    /// @param P any polytope such that `P.canBeSummed() == true`.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    CellGeometry makeCellCover( const Polytope& P,
				Dimension i = 0, Dimension k = KSpace::dimension  );
    
    
    /// @}
    
    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface services
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object. If the polytope
     * has been default constructed, it is invalid.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /// @}

    // ------------------------- Protected Datas ------------------------------
  protected:
    /// The cellular grid space where computations are done.
    KSpace myK;
    
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalConvexity

  /// @name Functions related to DigitalConvexity (output)
  /// @{
  
  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalConvexity'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalConvexity' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out, 
               const DigitalConvexity<TKSpace> & object );

  /// @}
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DigitalConvexity.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalConvexity_h

#undef DigitalConvexity_RECURSES
#endif // else defined(DigitalConvexity_RECURSES)
