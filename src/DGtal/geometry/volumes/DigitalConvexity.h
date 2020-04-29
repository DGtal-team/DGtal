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
#include "DGtal/geometry/volumes/BoundedRationalPolytope.h"
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
    typedef DGtal::BoundedLatticePolytope < Space > Polytope;
    typedef DGtal::BoundedLatticePolytope < Space > LatticePolytope;
    typedef DGtal::BoundedRationalPolytope< Space > RationalPolytope;
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

    // ----------------------- Simplex services --------------------------------------
  public:
    /// @name Simplex services
    /// @{

    /**
     * Constructs a lattice polytope from a simplex given as a range
     * [itB,itE) of lattice points.  Note that the range must contain
     * Space::dimension+1 points or less in general position.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param itB the start of the range of no more than n+1 points defining the simplex.
     * @param itE past the end the range of no more than n+1 points defining the simplex.
     */
    template <typename PointIterator>
    static
    LatticePolytope makeSimplex( PointIterator itB, PointIterator itE );

    /**
     * Constructs a lattice polytope from a simplex given as an initializer_list.
     *
     * @param l any list of no more than d+1 points in general positions.
     * @pre Note that the list must contain no more than Space::dimension+1 points
     * in general position.
     */
    static
    LatticePolytope makeSimplex( std::initializer_list<Point> l );

    /**
     * Constructs a rational polytope from a rational simplex given as a range
     * [itB,itE) of lattice points.  Note that the range must contain
     * Space::dimension+1 points or less in general position.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param d the common denominator of all given lattice point coordinates.
     * @param itB the start of the range of no more than n+1 points defining the simplex.
     * @param itE past the end the range of no more than n+1 points defining the simplex.
     *
     * @note If your range is `[itB,itE) = { (3,2), (1,7), (6,6) }` and the
     * denominator `d = 4`, then your polytope has vertices `{
     * (3/4,2/4), (1/4,7/4), (6/4,6/4) }`.
     */
    template <typename PointIterator>
    static
    RationalPolytope makeRationalSimplex( Integer d,
					  PointIterator itB, PointIterator itE );

    /**
     * Constructs a rational polytope from a simplex given as an initializer_list.
     *
     * @param l any list where the first point give the denominator
     * and then no more than d+1 points in general positions.
     *
     * @note If your list is `l = { (4,x), (3,2), (1,7), (6,6) }`, then the
     * denominator is `d = 4` and your polytope has vertices `{
     * (3/4,2/4), (1/4,7/4), (6/4,6/4) }`.
     */
    static
    RationalPolytope makeRationalSimplex( std::initializer_list<Point> l );

    /**
     * Checks if the given range [itB,itE) of lattice points form a
     * full dimensional simplex, i.e. it must contain
     * Space::dimension+1 points in general position.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param itB the start of the range of n+1 points defining the simplex.
     * @param itE past the end the range of n+1 points defining the simplex.
     */
    template <typename PointIterator>
    static
    bool isSimplexFullDimensional( PointIterator itB, PointIterator itE );

    /**
     * Checks if the given list of lattice points \a l form a
     * full dimensional simplex, i.e. it must contain
     * Space::dimension+1 points in general position.
     *
     * @param l any list of d+1 points in general positions.
     */
    static
    bool isSimplexFullDimensional( std::initializer_list<Point> l );

    /// The possible types for simplices.
    enum class SimplexType{
      INVALID,     ///< When there are not the right number of vertices
      DEGENERATED, ///< When the points of the simplex are not in general position
      UNITARY,     ///< When its edges form a unit parallelotope (det = +/- 1)
      COMMON       ///< Common simplex
    };
    
    /**
     * Returns the type of simplex formed by the given range [itB,itE)
     * of lattice points.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param itB the start of the range of n+1 points defining the simplex.
     * @param itE past the end the range of n+1 points defining the simplex.
     * @return the type of simplex formed by the given range [itB,itE)
     * of lattice points.
     */
    template <typename PointIterator>
    static
    SimplexType simplexType( PointIterator itB, PointIterator itE );

    /**
     * Returns the type of simplex formed by the given list \a l
     * of lattice points.
     *
     * @param l any list of lattice points.
     * @return the type of simplex formed by the given list of lattice points.
     */
    static
    SimplexType simplexType( std::initializer_list<Point> l );

    /**
     * Displays information about the simplex formed by the given range [itB,itE)
     * of lattice points.
     *
     * @tparam PointIterator any model of forward iterator on Point.
     * @param[inout] out the output stream where information is outputed.
     * @param itB the start of the range of n+1 points defining the simplex.
     * @param itE past the end the range of n+1 points defining the simplex.
     */
    template <typename PointIterator>
    static
    void displaySimplex( std::ostream& out, PointIterator itB, PointIterator itE );

    /**
     * Displays information about simplex formed by the given list \a l
     * of lattice points.
     *
     * @param[inout] out the output stream where information is outputed.
     * @param l any list of lattice points.
     */
    static
    void displaySimplex( std::ostream& out, std::initializer_list<Point> l );
    
    /// @}

    // ----------------------- Polytope services --------------------------------------
  public:
    /// @name Lattice and rational polytope services
    /// @{
    
    /// @param polytope any lattice polytope.
    /// @return the range of digital points that belongs to the polytope.
    static
    PointRange insidePoints( const LatticePolytope& polytope );

    /// @param polytope any lattice polytope.
    /// @return the range of digital points that belongs to the interior of the polytope.
    static
    PointRange interiorPoints( const LatticePolytope& polytope );

    /// @param polytope any rational polytope.
    /// @return the range of digital points that belongs to the polytope.
    static
    PointRange insidePoints( const RationalPolytope& polytope );

    /// @param polytope any rational polytope.
    /// @return the range of digital points that belongs to the interior of the polytope.
    static
    PointRange interiorPoints( const RationalPolytope& polytope );

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
				Dimension i = 0,
				Dimension k = KSpace::dimension ) const;

    /// Builds the cell geometry containing all the j-cells touching
    /// the lattice polytope P, for i <= j <= k. It conbains thus all the
    /// j-cells intersecting the convex enveloppe of P.
    ///
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    CellGeometry makeCellCover( const LatticePolytope& P,
				Dimension i = 0,
				Dimension k = KSpace::dimension ) const;

    /// Builds the cell geometry containing all the j-cells touching
    /// the rational polytope P, for i <= j <= k. It conbains thus all the
    /// j-cells intersecting the convex enveloppe of P.
    ///
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    CellGeometry makeCellCover( const RationalPolytope& P,
				Dimension i = 0,
				Dimension k = KSpace::dimension ) const;
    /// @}


    // ----------------------- Convexity services -----------------------------------
  public:
    /// @name Convexity services for lattice polytopes 
    /// @{

    /// Tells if a given polytope \a P is digitally k-convex. The digital
    /// 0-convexity is the usual property \f$ Conv( P \cap Z^d ) = P
    /// \cap Z^d) \f$. Otherwise the property asks that the points
    /// inside P touch as many k-cells that the convex hull of P.
    
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param k the dimension for which the digital k-convexity is checked, 0 <= k <= KSpace::dimension.
    /// @return 'true' iff the polytope \a P is digitally \a k-convex.
    ///
    /// @note A polytope is always digitally 0-convex. Furthermore, if
    /// it is not digitally d-1-convex then it is digitally not d-convex
    /// (d := KSpace::dimension).
    bool isKConvex( const LatticePolytope& P, const Dimension k ) const;

    /// Tells if a given polytope \a P is fully digitally convex. The
    /// digital 0-convexity is the usual property \f$ Conv( P \cap Z^d
    /// ) = P \cap Z^d) \f$. Otherwise the property asks that the
    /// points inside P touch as many k-cells that the convex hull of
    /// P, for any valid dimension k.
    
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @return 'true' iff the polytope \a P is fully digitally convex.
    ///
    /// @note A polytope is always digitally 0-convex. Furthermore, if
    /// it is not digitally d-1-convex then it is digitally d-convex
    /// (d := KSpace::dimension). Hence, we only check k-convexity for
    /// 1 <= k <= d-1.
    bool isFullyConvex( const LatticePolytope& P ) const;

    /// Tells if a given polytope \a P is digitally k-subconvex of some
    /// cell cover \a C. The digital 0-subconvexity is the usual
    /// property \f$ Conv( P \cap Z^d ) \subset C \cap Z^d)
    /// \f$. Otherwise the property asks that the k-cells intersected
    /// by the convex hull of P is a subset of the k-cells of C.
    
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param C any cell cover geometry (i.e. a cubical complex).
    /// @param k the dimension for which the digital k-convexity is checked, 0 <= k <= KSpace::dimension.
    /// @return 'true' iff the polytope \a P is a digitally \a k-subconvex of C.
    bool isKSubconvex( const LatticePolytope& P, const CellGeometry& C, const Dimension k ) const;

    /// Tells if a given polytope \a P is digitally fully subconvex to some
    /// cell cover \a C. The digital 0-subconvexity is the usual
    /// property \f$ Conv( P \cap Z^d ) \subset C \cap Z^d)
    /// \f$. Otherwise the property asks that the k-cells intersected
    /// by the convex hull of P is a subset of the k-cells of C.
    
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param C any cell cover geometry (i.e. a cubical complex).
    /// @return 'true' iff the polytope \a P is digitally fully subconvex to C.
    ///
    /// @note This method only checks the k-subconvexity for valid
    /// dimensions stored in \a C.
    bool isFullySubconvex( const LatticePolytope& P, const CellGeometry& C ) const;
    
    /// @}

    // ----------------------- Convexity services -----------------------------------
  public:
    /// @name Convexity services for rational polytopes 
    /// @{

    /// Tells if a given polytope \a P is digitally k-convex. The digital
    /// 0-convexity is the usual property \f$ Conv( P \cap Z^d ) = P
    /// \cap Z^d) \f$. Otherwise the property asks that the points
    /// inside P touch as many k-cells that the convex hull of P.
    
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @param k the dimension for which the digital k-convexity is checked, 0 <= k <= KSpace::dimension.
    /// @return 'true' iff the polytope \a P is digitally \a k-convex.
    ///
    /// @note A polytope is always digitally 0-convex. Furthermore, if
    /// it is not digitally d-1-convex then it is digitally not d-convex
    /// (d := KSpace::dimension).
    bool isKConvex( const RationalPolytope& P, const Dimension k ) const;

    /// Tells if a given polytope \a P is fully digitally convex. The
    /// digital 0-convexity is the usual property \f$ Conv( P \cap Z^d
    /// ) = P \cap Z^d) \f$. Otherwise the property asks that the
    /// points inside P touch as many k-cells that the convex hull of
    /// P, for any valid dimension k.
    
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @return 'true' iff the polytope \a P is fully digitally convex.
    ///
    /// @note A polytope is always digitally 0-convex. Furthermore, if
    /// it is not digitally d-1-convex then it is digitally d-convex
    /// (d := KSpace::dimension). Hence, we only check k-convexity for
    /// 1 <= k <= d-1.
    bool isFullyConvex( const RationalPolytope& P ) const;

    /// Tells if a given polytope \a P is digitally k-subconvex of some
    /// cell cover \a C. The digital 0-subconvexity is the usual
    /// property \f$ Conv( P \cap Z^d ) \subset C \cap Z^d)
    /// \f$. Otherwise the property asks that the k-cells intersected
    /// by the convex hull of P is a subset of the k-cells of C.
    
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @param C any cell cover geometry (i.e. a cubical complex).
    /// @param k the dimension for which the digital k-convexity is checked, 0 <= k <= KSpace::dimension.
    /// @return 'true' iff the polytope \a P is a digitally \a k-subconvex of C.
    bool isKSubconvex( const RationalPolytope& P, const CellGeometry& C, const Dimension k ) const;

    /// Tells if a given polytope \a P is digitally fully subconvex to some
    /// cell cover \a C. The digital 0-subconvexity is the usual
    /// property \f$ Conv( P \cap Z^d ) \subset C \cap Z^d)
    /// \f$. Otherwise the property asks that the k-cells intersected
    /// by the convex hull of P is a subset of the k-cells of C.
    
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @param C any cell cover geometry (i.e. a cubical complex).
    /// @return 'true' iff the polytope \a P is digitally fully subconvex to C.
    ///
    /// @note This method only checks the k-subconvexity for valid
    /// dimensions stored in \a C.
    bool isFullySubconvex( const RationalPolytope& P, const CellGeometry& C ) const;
    
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
