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
#include "DGtal/kernel/LatticeSetByIntervals.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytopeCounter.h"
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
     digital k-convexity and full convexity.

     @see moduleDigitalConvexity

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
    typedef std::size_t                     Size;
    typedef DGtal::BoundedLatticePolytope < Space > Polytope;
    typedef DGtal::BoundedLatticePolytope < Space > LatticePolytope;
    typedef DGtal::BoundedRationalPolytope< Space > RationalPolytope;
    typedef DGtal::CellGeometry< KSpace >   CellGeometry;
    typedef std::vector<Point>              PointRange;
    typedef std::unordered_set<Point>       PointSet;
    typedef DGtal::BoundedLatticePolytopeCounter< Space > Counter;
    typedef typename Counter::Interval      Interval;
    typedef DGtal::LatticeSetByIntervals< Space > LatticeSet;

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
     * @param safe when 'true' performs convex hull computations with arbitrary
     * precision integer (if available), otherwise chooses a
     * compromise between speed and precision (int64_t).
     */
    DigitalConvexity( Clone<KSpace> K, bool safe = false );

    /**
     * Constructor from lower and upper points.
     * @param lo the lowest point of the domain (bounding box for computations).
     * @param hi the highest point of the domain (bounding box for computations).
     * @param safe when 'true' performs convex hull computations with arbitrary
     * precision integer (if available), otherwise chooses a
     * compromise between speed and precision (int64_t).
     */
    DigitalConvexity( Point lo, Point hi, bool safe = false );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// @return a const reference to the cellular grid space used by this object.
    const KSpace& space() const;

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
     * @param[in, out] out the output stream where information is outputted.
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
     * @param[in, out] out the output stream where information is outputted.
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
    /// j-cells intersecting the convex hull of P.
    ///
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    CellGeometry makeCellCover( const LatticePolytope& P,
                                Dimension i = 0,
                                Dimension k = KSpace::dimension ) const;

    /// Builds the cell geometry containing all the j-cells touching
    /// the rational polytope P, for i <= j <= k. It conbains thus all the
    /// j-cells intersecting the convex hull of P.
    ///
    /// @param P any rational polytope such that `P.canBeSummed() == true`.
    /// @param i the first dimension for which the cell cover is computed.
    /// @param k the last dimension for which the cell cover is computed.
    CellGeometry makeCellCover( const RationalPolytope& P,
                                Dimension i = 0,
                                Dimension k = KSpace::dimension ) const;
    /// @}

    // ----------------------- Morphological services --------------------------------
  public:
    /// @name Morphological services
    /// @{

    /// Given a range of distinct points \a X, computes the tightiest
    /// polytope that enclosed it. Note that this polytope may contain
    /// more lattice points than the given input points.
    ///
    /// @param X any range of \b pairwise \b distinct points
    ///
    /// @param[in] make_minkowski_summable Other constraints are added
    /// so that we can perform axis aligned Minkowski sums on this
    /// polytope. Useful in 2D/3D for checking digital k-convexity (see
    /// moduleDigitalConvexity).
    ///
    /// @return the corresponding lattice polytope.
    LatticePolytope makePolytope( const PointRange& X,
                                  bool make_minkowski_summable = false ) const;

    /// Performs the digital Minkowski sum of \a X along direction \a i
    /// @param i any valid dimension
    /// @param X any \b sorted range of digital points
    ///
    /// @return the \b sorted range of digital points X union the
    /// translation of X of one along direction \a i.
    PointRange U( Dimension i, const PointRange& X ) const;

    /// Tells if a given point range \a X is digitally 0-convex,
    /// i.e. \f$ \mathrm{Cvxh}(X) \cap \mathbb{Z}^d = X \f$. It works
    /// for arbitrary set of points in arbitrary dimenion.
    ///
    /// @param X any range of \b pairwise \b distinct points
    ///
    /// @return 'true' iff \a X is fully digitally convex.
    bool is0Convex( const PointRange& X ) const;

    /// Tells if a given point range \a X is fully digitally
    /// convex. The test uses the morphological characterization of
    /// full convexity. It is slightly slower than testing full
    /// convexity on simplices, but it works for arbitrary set of
    /// points in arbitrary dimenion.
    ///
    /// @param X any range of \b pairwise \b distinct points
    ///
    /// @param convex0 when 'true' indicates that \a X is known to be
    /// digitally 0-convex, otherwise the method will check it also.
    ///
    /// @return 'true' iff \a X is fully digitally convex.
    bool isFullyConvex( const PointRange& X, bool convex0 = false ) const;

    /// Tells if a given point range \a X is fully digitally
    /// convex. The test uses the morphological characterization of
    /// full convexity and a fast way to compute lattice points within
    /// a polytope. It works for arbitrary set of points in arbitrary
    /// dimenion.
    ///
    /// @param X any range of \b pairwise \b distinct points
    /// @return 'true' iff \a X is fully digitally convex.
    ///
    /// @note This method is generally faster than
    /// DigitalConvexity::isFullyConvex if (1) the set is indeed is
    /// fully convex, (2) the dimension is high (>= 3 or 4).
    bool isFullyConvexFast( const PointRange& X ) const;

    /// Tells if a given set of points Y is digitally fully subconvex to
    /// some lattice set \a Star_X, i.e. the cell cover of some set X
    /// represented by lattice points.
    ///
    /// @param Y any set of points
    /// @param StarX any lattice set representing an open cubical complex.
    /// @return 'true' iff  Y is digitally fully subconvex to X.
    ///
    /// @note This method is slower than the two others, since it
    /// builds the polytope embracing \a Y. However it is much more
    /// generic since the two other methods require a Minkowski
    /// summable polytope, i.e. `P.canBeSummed() == true`.
    bool isFullySubconvex( const PointRange& Y, const LatticeSet& StarX ) const;


    /// Tells if the non-degenerated 3D triangle a,b,c is digitally
    /// fully subconvex to some lattice set \a Star_X, i.e. the cell
    /// cover of some set X represented by lattice points.
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param StarX any lattice set representing an open cubical complex.
    /// @return 'true' iff  Y is digitally fully subconvex to X.
    ///
    /// @note This method is supposed to be faster than the others,
    /// but is limited to 3D triangles.
    bool isFullySubconvex( const Point& a, const Point& b, const Point& c,
			   const LatticeSet& StarX ) const;

    /// Tells if the non-degenerated 3D triangle a,b,c is
    /// fully covered by some lattice set of cells \a cells.
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param cells any lattice set representing a set of cells.
    ///
    /// @return 'true' iff `Cvxh({a,b,c})` is fully covered by \a cells,
    /// i.e. all the cells intersected by the triangle belong to \a
    /// cells.
    ///
    /// @note This method is limited to 3D triangles.
    bool isFullyCovered( const Point& a, const Point& b, const Point& c,
			 const LatticeSet& cells ) const;

    /// Tells if the non-degenerated 3D segment a,b is
    /// fully covered by some lattice set of cells \a cells.
    ///
    /// @param a any 3D point (distinct from the other)
    /// @param b any 3D point (distinct from the other)
    ///
    /// @param cells any lattice set representing a set of cells.
    ///
    /// @return 'true' iff `Cvxh({a,b})` is fully covered by \a cells,
    /// i.e. all the cells intersected by the triangle belong to \a
    /// cells.
    ///
    /// @note This method is limited to 3D segments.
    bool isFullyCovered( const Point& a, const Point& b,
			 const LatticeSet& cells ) const;


    /// Tells if the non-degenerated 3D open triangle a,b,c (ie
    /// without its edges and vertices) is digitally fully subconvex
    /// to some lattice set \a Star_X, i.e. the cell cover of some set
    /// X represented by lattice points.
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param StarX any lattice set representing an open cubical complex.
    /// @return 'true' iff  Y is digitally fully subconvex to X.
    ///
    /// @note This method is supposed to be faster than the others,
    /// but is limited to 3D open triangles.
    bool isOpenTriangleFullySubconvex( const Point& a, const Point& b, const Point& c,
				       const LatticeSet& StarX ) const;


    /// Tells if a given segment from \a a to \a b is digitally
    /// k-subconvex (i.e. k-tangent) to some cell cover \a C. The
    /// digital 0-subconvexity is the usual property \f$ Conv( P \cap
    /// Z^d ) \subset C \cap Z^d) \f$. Otherwise the property asks
    /// that the k-cells intersected by the convex hull of the segment
    /// is a subset of the k-cells of C.
    ///
    /// @param a any point
    /// @param b any point
    /// @param C any cell cover geometry (i.e. a cubical complex).
    /// @param k the dimension for which the digital k-convexity is checked, 0 <= k <= KSpace::dimension.
    ///
    /// @return 'true' iff the segment is a digitally \a k-subconvex
    /// of C, i.e. the two points are k-cotangent.
    ///
    /// @note Three times faster than building a (degenerated) lattice
    /// polytope and then checking if it subconvex.
    bool isKSubconvex( const Point& a, const Point& b,
                       const CellGeometry& C, const Dimension k ) const;

    /// Tells if a given segment from \a a to \a b is digitally fully
    /// subconvex (i.e. tangent) to some cell cover \a C. The digital
    /// 0-subconvexity is the usual property \f$ Conv( P \cap Z^d )
    /// \subset C \cap Z^d) \f$. Otherwise the property asks that the
    /// k-cells intersected by the convex hull of the segment is a
    /// subset of the k-cells of C.
    ///
    /// @param a any point
    /// @param b any point
    /// @param C any cell cover geometry (i.e. a cubical complex).
    ///
    /// @return 'true' iff the segment is a digitally fully subconvex
    /// of C, i.e. the two points are cotangent.
    ///
    /// @note Three times faster than building a (degenerated) lattice
    /// polytope and then checking if it subconvex.
    bool isFullySubconvex( const Point& a, const Point& b,
                           const CellGeometry& C ) const;

    /// Tells if a given segment from \a a to \a b is digitally fully
    /// subconvex (i.e. tangent) to some open complex \a StarX.
    ///
    /// @param a any point
    /// @param b any point
    /// @param StarX any lattice set representing an open cubical complex.
    ///
    /// @return 'true' iff the segment is a digitally fully subconvex
    /// of C, i.e. the two points are cotangent.
    ///
    /// @note Three times faster than building a (degenerated) lattice
    /// polytope and then checking if it subconvex.
    bool isFullySubconvex( const Point& a, const Point& b,
                           const LatticeSet& StarX ) const;

    /// Given a range of distinct points \a X, computes the tightiest
    /// polytope that enclosed it. Note that this polytope may contain
    /// more lattice points than the given input points.
    ///
    /// @param X any range of \b pairwise \b distinct points
    ///
    /// @return the corresponding lattice polytope.
    ///
    /// @note alias for DigitalConvexity::makePolytope
    LatticePolytope CvxH( const PointRange& X ) const
    {
      return makePolytope( X );
    }

    /// Given a range of distinct points \a X, computes the vertices
    /// of the tightiest polytope that enclosed it.
    ///
    /// @param X any range of \b pairwise \b distinct points
    ///
    /// @return the vertices or extrema of `CvxH(X)`.
    ///
    /// @note The method works in nD for full dimensional convex
    /// hulls. It can handle not full dimensional convex hull up to
    /// dimension 3 included.
    PointRange ExtrCvxH( const PointRange& X ) const;

    /// Builds the cell complex Star(CvxH(X)) for X a digital set,
    /// represented as a lattice set (stacked row representation).
    ///
    /// @param X any range of lattice points
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells touching the convex hull of X,
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates).
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet StarCvxH( const PointRange& X,
                         Dimension axis = dimension ) const;

    /// Builds the cell complex `Star(CvxH({a,b,c}))` for `a,b,c` a
    /// non-degenerate 3D triangle, represented as a lattice set (stacked
    /// row representation).
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells touching the triangle `abc`,
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates). If the triangle is degenerate (a,b,c
    /// not distinct or aligned), then it returns an empty range of
    /// cells.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet StarCvxH( const Point& a, const Point& b, const Point& c,
                         Dimension axis = dimension ) const;

    /// Builds the cell complex `Star(OpenTriangle({a,b,c}))` for
    /// `a,b,c` the vertices of a non-degenerate open 3D triangle (ie
    /// without edges or vertices), represented as a lattice set
    /// (stacked row representation).
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells touching the open triangle `abc`,
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates). If the triangle is degenerate (a,b,c
    /// not distinct or aligned), then it returns an empty range of
    /// cells.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet StarOpenTriangle( const Point& a, const Point& b, const Point& c,
				 Dimension axis = dimension ) const;


    /// Computes the number of cells in Star(CvxH(X)) for X a digital set.
    ///
    /// @param X any range of lattice points
    ///
    /// @return the number of cells touching the convex hull of X,
    /// represented as lattice points with Khalimsky coordinates.
    Integer sizeStarCvxH( const PointRange& X ) const;

    /// Builds the cell complex Star(X) for X a digital set,
    /// represented as a lattice set (stacked row representation).
    ///
    /// @param X any range of lattice points
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the set of cells, represented as a lattice set, that
    /// touches points of \a X, i.e. `Star(X)`.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet Star( const PointRange& X,
                     Dimension axis = dimension ) const;

    /// Builds the cell complex Star(C) for C a range of cells,
    /// represented as a lattice set (stacked row representation).
    ///
    /// @param C a range of cells represented with points in Khalimsky coordinates.
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the set of cells, represented as a lattice set, that
    /// touches cells of \a C, i.e. `Star(C)`.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet StarCells( const PointRange& C,
                          Dimension axis = dimension ) const;

    /// @param C a range of cells represented with points in Khalimsky coordinates.
    ///
    /// @return the range of digital points that are the extremal
    /// vertices to the cells in \a C.
    PointRange Extr( const PointRange& C ) const;

    /// @param C a range of cells represented as a lattice set.
    ///
    /// @return the range of digital points that are the extremal
    /// vertices to the cells in \a C.
    PointRange Extr( const LatticeSet& C ) const;

    /// @param C a range of cells represented as a lattice set.
    ///
    /// @return the set of cells, represented as a lattice set, that
    /// form the skeleton of the given range of cells \a C.
    LatticeSet Skel( const LatticeSet& C ) const;

    /// @param C a range of cells represented as a lattice set.
    ///
    /// @return the range of digital points that are the extremal
    /// vertices to the skeleton of the cells in \a C.
    PointRange ExtrSkel( const LatticeSet& C ) const;

    /// Builds the cell complex `Cover(CvxH({a,b}))` for `a,b` a
    /// non-degenerate 3D segment, represented as a lattice set
    /// (stacked row representation). The cover of a Euclidean set X
    /// is the set of grid cells that have a non-empty intersection
    /// with X. It is always included in the star of X, which is the
    /// of cells whose closure has a non-empty intersection with
    /// X. However the cover of X is not necessarily closed or open.
    ///
    /// @param a any 3D point (distinct from the other)
    /// @param b any 3D point (distinct from the other)
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells intersecting the segment `ab`,
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates). If the segment is degenerate (a,b
    /// not distinct), then it returns an empty range of
    /// cells.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet CoverCvxH( const Point& a, const Point& b,
			  Dimension axis = dimension ) const;

    /// Builds the cell complex `Cover(CvxH({a,b,c}))` for `a,b,c` a
    /// non-degenerate 3D triangle, represented as a lattice set
    /// (stacked row representation). The cover of a Euclidean set X
    /// is the set of grid cells that have a non-empty intersection
    /// with X. It is always included in the star of X, which is the
    /// of cells whose closure has a non-empty intersection with
    /// X. However the cover of X is not necessarily closed or open.
    ///
    /// @param a any 3D point (distinct from the two others)
    /// @param b any 3D point (distinct from the two others)
    /// @param c any 3D point (distinct from the two others)
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells intersecting the triangle `abc`,
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates). If the triangle is degenerate (a,b,c
    /// not distinct or aligned), then it returns an empty range of
    /// cells.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet CoverCvxH( const Point& a, const Point& b, const Point& c,
			  Dimension axis = dimension ) const;


    /// Builds the lattice set (stacked row representation) associated
    /// to the given range of points.
    ///
    /// @param X any range of lattice points
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the lattice set that represents the exact same points as \a X
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    LatticeSet toLatticeSet( const PointRange& X,
                             Dimension axis = dimension ) const;

    /// Builds the range of lattice points  associated
    /// to the given lattice set.
    ///
    /// @param L any lattice set.
    ///
    /// @return the point range that represents the exact same points as \a L
    PointRange toPointRange( const LatticeSet& L ) const;

    /// @}

    // ----------------------- Convex envelope services -----------------------------
  public:
    /// @name Convex envelope services
    /// @{

    /// Choice of algorithm for computing the fully convex envelope of
    /// a digital set.
    enum class EnvelopeAlgorithm
      { DIRECT /**< Slightly faster but quite ugly big function */,
        LATTICE_SET /**< Slightly slower function but decomposes well the algorithm */
      };

    /// Computes `FC(Z):=Extr(Skel(Star(CvxH(Z))))`, for \a Z a range of points
    /// @param Z any range of points (must be sorted).
    /// @param algo the chosen method of computation.
    /// @return  FC( Z )
    PointRange
    FC( const PointRange& Z,
        EnvelopeAlgorithm algo = EnvelopeAlgorithm::DIRECT ) const;

    /// Computes the fully convex envelope of \a Z,
    /// i.e. \f$ FC^*(Z):=FC(FC( \ldots FC(Z) \ldots )) \f$, for \a Z a range of
    /// points, until stabilization of the iterative process.
    ///
    /// @param Z any range of points (must be sorted).
    /// @param algo the chosen method of computation.
    /// @return \f$ FC^*( Z ) \f$
    ///
    /// @note If \a Z is fully convex, then the output is \a Z
    /// itself. Otherwise, the returned set of points includes \a Z
    /// and is fully convex.
    PointRange
    envelope( const PointRange& Z,
              EnvelopeAlgorithm algo = EnvelopeAlgorithm::DIRECT ) const;

    /// Computes the fully convex envelope of \a Z relative to fully
    /// convex digital set \a Y, i.e. \f$ FC^*_Y(Z):=FC_Y(FC_Y( \ldots
    /// FC_Y(Z) \ldots )) \f$ for \a Z a range of points, until
    /// stabilization of the iterative process.
    ///
    /// @param Z any range of points (must be sorted).
    /// @param Y any range of points (must be sorted) that is fully convex.
    /// @param algo the chosen method of computation.
    /// @return \f$ FC^*_Y( Z ) \f$
    ///
    /// @note If \a Z is fully convex, then the output is \a Z
    /// itself. Otherwise, the returned set of points includes \a Z
    /// and is fully convex.
    PointRange
    relativeEnvelope( const PointRange& Z, const PointRange& Y,
                      EnvelopeAlgorithm algo = EnvelopeAlgorithm::DIRECT ) const;

    /// Computes the fully convex envelope of \a Z relative to fully
    /// convex digital set \a Y defined by a corresponding predicate
    /// \a PredY. It computes \f$ FC^*_Y(Z):=FC_Y(FC_Y( \ldots FC_Y(Z) \ldots
    /// )) \f$ for \a Z a range of points, until stabilization of the
    /// iterative process.
    ///
    /// @tparam Predicate the type of a predicate Point -> boolean
    /// @param Z any range of points (must be sorted).
    /// @param PredY a Point predicate such that `PredY(p)==true` iff \a p belongs to \a Y.
    /// @param algo the chosen method of computation.
    /// @return \f$ FC^*_Y( Z ) \f$
    ///
    /// @note If \a Z is fully convex, then the output is \a Z
    /// itself. Otherwise, the returned set of points includes \a Z
    /// and is fully convex.
    template <typename Predicate>
    PointRange
    relativeEnvelope( const PointRange& Z, const Predicate& PredY,
                      EnvelopeAlgorithm algo = EnvelopeAlgorithm::DIRECT ) const;

    /// @return the number of iterations of the last process
    /// `FC^*(Z):=FC(FC(....FC(Z)...))`, i.e. the last call to
    /// DigitalConvexity::envelope or DigitalConvexity::relativeEnvelope .
    Size depthLastEnvelope() const;

    /// @}

    // ----------------------- Convexity services for lattice polytopes --------------
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

    /// Tells if a given polytope \a P is digitally fully subconvex to
    /// some lattice set \a Star_X, i.e. the cell cover of some set X
    /// represented by lattice points.
    ///
    /// @param P any lattice polytope such that `P.canBeSummed() == true`.
    /// @param StarX any lattice set representing an open cubical complex.
    /// @return 'true' iff  Y is digitally fully subconvex to X.
    bool isFullySubconvex( const LatticePolytope& P, const LatticeSet& StarX ) const;


    /// @}

    // ----------------------- Convexity services for rational polytopes ----------------
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

    // ------------------------- Protected Data ------------------------------
  protected:
    /// The cellular grid space where computations are done.
    KSpace myK;

    /// when 'true' performs convex hull computations with arbitrary
    /// precision integer (if available), otherwise chooses a
    /// compromise between speed and precision (int64_t).
    bool mySafe;

    /// The number of iterations of the last FullyConvexEnvelope operation.
    mutable Size myDepthLastFCE;

    // ------------------------- Private Data --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

    /// Computes `FC(Z):=Extr(Skel(Star(CvxH(Z))))`, for \a Z a range of points
    /// @param Z any range of points (must be sorted).
    /// @return  FC( Z )
    PointRange FC_direct( const PointRange& Z ) const;

    /// Computes `FC(Z):=Extr(Skel(Star(CvxH(Z))))`, for \a Z a range of points
    /// @param Z any range of points (must be sorted).
    /// @return  FC( Z )
    PointRange FC_LatticeSet( const PointRange& Z ) const;

    /// Erase the interval I from the intervals in V such that the integer
    /// in I are not part of V anymore.
    ///
    /// @param[in] I is a closed interval
    /// @param[inout] V is a sorted list of closed intervals
    static void eraseInterval( Interval I, std::vector< Interval > & V );

    /// Filters the points of \a E and outputs only the ones that
    /// satisfies the given predicate \a Pred.
    ///
    /// @tparam Predicate the type of a predicate Point -> boolean
    /// @param[in] E any range of point
    /// @param[in] Pred the predicate Point -> boolean
    /// @return the subset of E whose elements satisfy the predicate \a Pred.
    template <typename Predicate>
    static PointRange filter( const PointRange& E, const Predicate& Pred );

    /// Builds the cell complex `Cover(P))` for `P` a Minkowski
    /// summable polytope, represented as a lattice set (stacked row
    /// representation). The cover of a Euclidean set X is the set of
    /// grid cells that have a non-empty intersection with X. It is
    /// always included in the star of X, which is the of cells whose
    /// closure has a non-empty intersection with X. However the cover
    /// of X is not necessarily closed or open.
    ///
    /// @tparam TPolytope an instance of LatticePolytope.
    /// @param P any Minkowski summable lattice polytope (see LatticePolytope).
    ///
    /// @param axis specifies the projection axis for the row
    /// representation if below space dimension, otherwise chooses the
    /// axis that minimizes memory/computations.
    ///
    /// @return the range of cells intersecting the polytope
    /// represented as a lattice set (cells are represented with
    /// Khalimsky coordinates).
    ///
    /// @note Used by CoverCvxH.
    ///
    /// @note It is useful to specify an axis if you wish later to
    /// compare or make operations with several lattice sets. They
    /// must indeed have the same axis.
    template <typename TPolytope>
    LatticeSet CoverPolytope( const TPolytope& P,
			      Dimension axis = dimension ) const;



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
