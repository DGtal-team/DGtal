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
 * @file CellGeometry.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/01/27
 *
 * Header file for module CellGeometry.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CellGeometry_RECURSES)
#error Recursive header files inclusion detected in CellGeometry.h
#else // defined(CellGeometry_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CellGeometry_RECURSES

#if !defined CellGeometry_h
/** Prevents repeated inclusion of headers. */
#define CellGeometry_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/UnorderedSetByBlock.h"
#include "DGtal/kernel/PointHashFunctions.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/KhalimskyCellHashFunctions.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/geometry/volumes/BoundedRationalPolytope.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CellGeometry
  /**
     Description of template class 'CellGeometry' <p> \brief Aim:
     Computes and stores sets of cells and provides methods to compute
     intersections of lattice and rational polytopes with cells.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   */
  template < typename TKSpace >
  class CellGeometry
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

  public:
    typedef CellGeometry<TKSpace>           Self;
    typedef TKSpace                         KSpace;
    typedef typename KSpace::Integer        Integer;
    typedef typename KSpace::Point          Point;
    typedef typename KSpace::Vector         Vector;
    typedef typename KSpace::Cell           Cell;
    typedef typename KSpace::Space          Space;
    typedef typename KSpace::Size           Size;
    typedef DGtal::BigInteger               BigInteger;
    typedef DGtal::BoundedLatticePolytope< Space >  Polytope;
    typedef DGtal::BoundedLatticePolytope< Space >  LatticePolytope;
    typedef DGtal::BoundedRationalPolytope< Space > RationalPolytope;

    static const Dimension dimension = KSpace::dimension;


    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /**
     * Destructor.
     */
    ~CellGeometry() = default;

    /**
     * Constructor.
     */
    CellGeometry();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CellGeometry ( const Self & other ) = default;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    CellGeometry ( Self && other ) = default;

    /**
     * Constructor from cellular space.
     * @param K any cellular grid space.
     * @param min_cell_dim the minimum cell dimension that is used for processing.
     * @param max_cell_dim the maximal cell dimension that is used for processing (K::dimension - 1 is sufficient to check convexity).
     * @param verbose tells if verbose mode.
     */
    CellGeometry ( const KSpace & K,
                   Dimension min_cell_dim = 0,
                   Dimension max_cell_dim = KSpace::dimension,
                   bool verbose = false );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// Initialization from cellular space. If there was cells stored
    /// in this object, they are removed.
    ///
    /// @param K any cellular grid space.
    /// @param min_cell_dim the minimum cell dimension that is used for processing.
    /// @param max_cell_dim the maximal cell dimension that is used for processing (K::dimension - 1 is sufficient to check convexity)
    /// @param verbose tells if verbose mode.
    void init( const KSpace & K,
               Dimension min_cell_dim = 0,
               Dimension max_cell_dim = KSpace::dimension,
               bool verbose = false );
    /// @}

    // ----------------------- Cells services ------------------------------
  public:
    /// @name Cells services
    /// @{

    /// Updates the cell cover with the cells touching a point \a p.
    /// @param p any point
    void addCellsTouchingPoint( const Point& p );

    /// Updates the cell cover with the cells touching a pointel \a pointel.
    /// @param pointel any pointel
    void addCellsTouchingPointel( const Cell& pointel );

    /// Updates the cell cover with the cells whose boundary covers
    /// cell \a c (so \a c itself and its up-incident cells).
    ///
    /// @param c any any cell
    void addCellsTouchingCell( const Cell& c );

    /// Given two points \a a and \a b, updates the cell cover with
    /// all the cells intersected by the Euclidean straight segment
    /// from \a a to \a b.
    ///
    /// @param a any point
    /// @param b any point
    ///
    /// @note Three times faster than building a (degenerated) lattice
    /// polytope and then calling addCellsTouchingPolytope.
    void addCellsTouchingSegment( const Point& a, const Point& b );
    
    /// Updates the cell cover with the cells touching a range of
    /// digital points [itB, itE).
    template <typename PointIterator>
    void addCellsTouchingPoints( PointIterator itB, PointIterator itE );

    /// Updates the cell cover with the cells touching a range of
    /// digital pointels [itB, itE).
    template <typename PointelIterator>
    void addCellsTouchingPointels( PointelIterator itB, PointelIterator itE );

    /// Updates the cell cover with the cells touching the lattice
    /// points of a polytope.
    /// @param polytope the lattice polytope
    void addCellsTouchingPolytopePoints( const LatticePolytope& polytope );

    /// Updates the cell cover with the cells touching the lattice
    /// points of a rational polytope.
    /// @param polytope the rational polytope
    void addCellsTouchingPolytopePoints( const RationalPolytope& polytope );

    /// Updates the cell cover with all the cells touching the
    /// lattice polytope (all cells whose closure have a non empty
    /// intersection with the polytope).
    /// @param polytope the lattice polytope
    void addCellsTouchingPolytope( const LatticePolytope& polytope );

    /// Updates the cell cover with all the cells touching the
    /// rational polytope (all cells whose closure have a non empty
    /// intersection with the polytope).
    /// @param polytope the rational polytope
    void addCellsTouchingPolytope( const RationalPolytope& polytope );

    /// Adds the cells of dimension k of object \a other, for
    /// `minCellDim() <= k <= maxCellDim()`, to this cell geometry.
    ///
    /// @param other any cell geometry.
    /// @return a reference to this object.
    CellGeometry& operator+=( const CellGeometry& other );

    /// @}

    // ----------------------- Accessor services ------------------------------
  public:
    /// @name Accessor services
    /// @{

    /// @return the total number of cells in this cell geometry.
    Size nbCells() const;
    /// @param k any non negative integer
    /// @return computes and returns the number of k-dimensional cells
    /// in this cell geometry.
    Size computeNbCells(const Dimension k ) const;
    /// @return computes and return the Euler chracteristic of this set of cells.
    Integer computeEuler() const;
    /// @return the smallest dimension for which cells are stored in this object.
    Dimension minCellDim() const;
    /// @return the highest dimension for which cells are stored in this object.
    Dimension maxCellDim() const;

    /// @param k any non negative integer
    /// @return the vector of k-cells, represented as points with Khalimsky coordinates.
    std::vector< Point > getKPoints( const Dimension k ) const;
    
    /// @}

    // ----------------------- Cell services ------------------------------
  public:
    /// @name Cell services
    /// @{

    /// Tells if the cells of 'this' are subset of the cells of \a
    /// other, for all valid dimensions of 'this' (i.e. from
    /// myMinCellDim till myMaxCellDim).
    ///
    /// @param other any cell geometry object
    /// @return 'true' iff the cells of 'this' are subset of the cells of \a other.
    ///
    /// @note if `other.maxCellDim() < k` or `k < other.minCellDim()`
    /// then it means that \a other contains no cell of dimension k.
    bool subset( const CellGeometry& other ) const;

    /// Tells if the k-cells of 'this' are subset of the k-cells of \a
    /// other.
    ///
    /// @param other any cell geometry object
    /// @param k any valid dimension for cells (`0 <= k <= KSpace::dimension`)
    /// @return 'true' iff the k-cells of 'this' are subset of the k-cells of \a other.
    ///
    /// @note if `other.maxCellDim() < k` or `k < other.minCellDim()`
    /// then it means that \a other contains no cell of dimension k.
    bool subset( const CellGeometry& other, const Dimension k ) const;

    /// @}

    // ----------------------- helper services ------------------------------
  public:
    /// @name Helper services
    /// @{

    /// Given a lattice \a polytope, such that `polytope.canBeSummed()==true`,
    /// return the \a i-kpoints that intersect it.
    ///
    /// @param polytope any lattice polytope such that `polytope.canBeSummed() == true`
    /// @param i any integer between 0 and KSpace::dimension
    /// @return the \a i-kpoints that intersect this polytope.
    std::vector< Point >
    getIntersectedKPoints( const LatticePolytope& polytope, const Dimension i ) const;

    /// Given a rational \a polytope, such that `polytope.canBeSummed()==true`,
    /// return the \a i-kpoints that intersect it.
    ///
    /// @param polytope any rational polytope such that `polytope.canBeSummed() == true`
    /// @param i any integer between 0 and KSpace::dimension
    /// @return the \a i-kpoints that intersect this polytope.
    std::vector< Point >
    getIntersectedKPoints( const RationalPolytope& polytope, const Dimension i ) const;

    /// Given a vector of points, return the \a i-kpoints that touch it.
    ///
    /// @param points any vector of points
    /// @param i any integer between 0 and KSpace::dimension (and limited to 5-D at most).
    /// @return the \a i-kpoints that intersect this polytope.
    std::vector< Point >
    getTouchedKPoints( const std::vector< Point >& points, const Dimension i ) const;


    /// Given a lattice \a polytope, such that `polytope.canBeSummed()==true`,
    /// return the \a i-cells that intersect it.
    ///
    /// @param polytope any lattice polytope such that `polytope.canBeSummed() == true`
    /// @param i any integer between 0 and KSpace::dimension
    /// @return the \a i-cells that intersect this polytope.
    std::vector< Cell >
    getIntersectedCells( const LatticePolytope& polytope, const Dimension i ) const;

    /// Given a rational \a polytope, such that `polytope.canBeSummed()==true`,
    /// return the \a i-cells that intersect it.
    ///
    /// @param polytope any rational polytope such that `polytope.canBeSummed() == true`
    /// @param i any integer between 0 and KSpace::dimension
    /// @return the \a i-cells that intersect this polytope.
    std::vector< Cell >
    getIntersectedCells( const RationalPolytope& polytope, const Dimension i ) const;

    /// Given a vector of points, return the \a i-cells that touch it.
    ///
    /// @param points any vector of points
    /// @param i any integer between 0 and KSpace::dimension (and limited to 5-D at most).
    /// @return the \a i-cells that intersect this polytope.
    std::vector< Cell >
    getTouchedCells( const std::vector< Point >& points, const Dimension i ) const;

    /// @}

    // ----------------------- Khalimsky point services -------------------------------
  public:
    /// @name Khalimsky point services
    /// @{

    /// @param kp a Khalimsky point (i.e. an integer point whose coordinates
    /// parities correspond to cells).
    /// @return the dimension of the cell associated to this Khalimsky point.
    static Dimension dim( const Point& kp );

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
     * Checks the validity/consistency of the object.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * @return the class name. It is notably used for drawing this object.
     */
    std::string className() const;

    /// @}

    // ------------------------- Protected Datas ------------------------------
  protected:

    /// The cellular space for cells.
    KSpace myK;
    /// The unordered set that stores cells.
    UnorderedSetByBlock< Point,
                         Splitter< Point, uint64_t> > myKPoints;
    /// The minimum cell dimension
    Dimension myMinCellDim;
    /// The maximal cell dimension
    Dimension myMaxCellDim;
    /// Tells if verbose mode.
    bool myVerbose;

    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:
    // Internal method for cheking if sorted range [it1,itE1] is a
    // subset of sorted range [it2, itE2].  Different from
    // std::includes since it performs exponential march and dichotomy
    // to walk faster along range [it1,itE1].
    template <typename RandomIterator>
    static
    bool includes( RandomIterator it2, RandomIterator itE2,
                   RandomIterator it1, RandomIterator itE1 );

  }; // end of class CellGeometry

  /// @name Functions related to CellGeometry (output)
  /// @{

  /**
   * Overloads 'operator<<' for displaying objects of class 'CellGeometry'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CellGeometry' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out,
               const CellGeometry<TKSpace> & object );

  /// @}

  /// Utility class gathering some useful functions related to cell
  /// geometry and digital or cell convexity. It is meant to be
  /// specialized for low dimensions.
  ///
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  /// @tparam i the integer specifying the dimension of cells.
  /// @tparam N the integer specifying the dimension of the digital space.
  template <typename TKSpace, int i, int N>
  struct CellGeometryFunctions
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;
    typedef typename KSpace::Point Point;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident i-cells to the given range of pointels [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      if ( i == 0 )
        for ( auto it = itB; it != itE; ++it )
          cells.insert( *it );
      else
        for ( auto it = itB; it != itE; ++it )
          {
            auto pointel = *it;
            auto cofaces = K.uCoFaces( pointel );
            for ( auto&& f : cofaces )
              if ( K.uDim( f ) == i ) cells.insert( f );
          }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident i-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      if ( i == 0 )
        for ( auto it = itB; it != itE; ++it )
          cells.insert( K.uPointel( *it ) );
      else
        for ( auto it = itB; it != itE; ++it )
          {
            auto pointel = K.uPointel( *it );
            auto cofaces = K.uCoFaces( pointel );
            for ( auto&& f : cofaces )
              if ( K.uDim( f ) == i ) cells.insert( f );
          }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident i-kpoints to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t > >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t > > kpoints;
      if ( i == 0 )
        for ( auto it = itB; it != itE; ++it )
          kpoints.insert( K.uKCoords( K.uPointel( *it ) ) );
      else
        for ( auto it = itB; it != itE; ++it )
          {
            auto pointel = K.uPointel( *it );
            auto cofaces = K.uCoFaces( pointel );
            for ( auto&& f : cofaces )
              if ( K.uDim( f ) == i ) kpoints.insert( K.uKCoords( f ) );
          }
      return kpoints;
    }


  }; // end struct CellGeometryFunctions

  /// Specialization for 1-cells in 2D.
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace>
  struct CellGeometryFunctions< TKSpace, 1, 2 >
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT( TKSpace::dimension == 2 );
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident 1-cells to the given range of pointels [itB, itE).
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = *it;
          cells.insert( K.uIncident( pointel, 0, true ) );
          cells.insert( K.uIncident( pointel, 0, false ) );
          cells.insert( K.uIncident( pointel, 1, true ) );
          cells.insert( K.uIncident( pointel, 1, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 1-cells to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = K.uPointel( *it );
          cells.insert( K.uIncident( pointel, 0, true ) );
          cells.insert( K.uIncident( pointel, 0, false ) );
          cells.insert( K.uIncident( pointel, 1, true ) );
          cells.insert( K.uIncident( pointel, 1, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 1-kpoints to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t > >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t > > kpoints;
      for ( auto it = itB; it != itE; ++it )
        {
          auto kp = K.uKCoords( K.uPointel( *it ) );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] - 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ]     );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ]     );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] + 1 );
        }
      return kpoints;
    }

  }; // end struct CellGeometryFunctions

  /// Specialization for 1-cells in 3D.
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace>
  struct CellGeometryFunctions< TKSpace, 1, 3 >
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT( TKSpace::dimension == 3 );
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident 1-cells to the given range of pointels [itB, itE).
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = *it;
          cells.insert( K.uIncident( pointel, 0, true ) );
          cells.insert( K.uIncident( pointel, 0, false ) );
          cells.insert( K.uIncident( pointel, 1, true ) );
          cells.insert( K.uIncident( pointel, 1, false ) );
          cells.insert( K.uIncident( pointel, 2, true ) );
          cells.insert( K.uIncident( pointel, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 1-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::cout << "<1,3> specialization" << std::endl;
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = K.uPointel( *it );
          cells.insert( K.uIncident( pointel, 0, true ) );
          cells.insert( K.uIncident( pointel, 0, false ) );
          cells.insert( K.uIncident( pointel, 1, true ) );
          cells.insert( K.uIncident( pointel, 1, false ) );
          cells.insert( K.uIncident( pointel, 2, true ) );
          cells.insert( K.uIncident( pointel, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 1-kpoints to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t > >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t > > kpoints;
      for ( auto it = itB; it != itE; ++it )
        {
          auto kp = K.uKCoords( K.uPointel( *it ) );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ]    , kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ]    , kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] - 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] + 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ]    , kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ]    , kp[ 2 ] + 1 );
        }
      return kpoints;
    }

  }; // end struct CellGeometryFunctions

  /// Specialization for 2-cells in 2D.
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace>
  struct CellGeometryFunctions< TKSpace, 2, 2 >
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT( TKSpace::dimension == 2 );
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident 2-cells to the given range of pointels [itB, itE).
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = *it;
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          cells.insert( K.uIncident( linelxp, 1, true ) );
          cells.insert( K.uIncident( linelxp, 1, false ) );
          cells.insert( K.uIncident( linelxm, 1, true ) );
          cells.insert( K.uIncident( linelxm, 1, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 2-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::cout << "<2,2> specialization" << std::endl;
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = K.uPointel( *it );
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          cells.insert( K.uIncident( linelxp, 1, true ) );
          cells.insert( K.uIncident( linelxp, 1, false ) );
          cells.insert( K.uIncident( linelxm, 1, true ) );
          cells.insert( K.uIncident( linelxm, 1, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 2-kpoints to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t> >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t> > kpoints;
      for ( auto it = itB; it != itE; ++it )
        {
          auto kp = K.uKCoords( K.uPointel( *it ) );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] - 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] - 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] + 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] + 1 );
        }
      return kpoints;
    }

  }; // end struct CellGeometryFunctions

  /// Specialization for 2-cells in 3D.
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace>
  struct CellGeometryFunctions< TKSpace, 2, 3 >
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT( TKSpace::dimension == 2 );
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident 2-cells to the given range of pointels [itB, itE).
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = *it;
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          auto linelyp = K.uIncident( pointel, 1, true );
          auto linelym = K.uIncident( pointel, 1, false );
          cells.insert( K.uIncident( linelxp, 1, true ) );
          cells.insert( K.uIncident( linelxp, 1, false ) );
          cells.insert( K.uIncident( linelxp, 2, true ) );
          cells.insert( K.uIncident( linelxp, 2, false ) );
          cells.insert( K.uIncident( linelxm, 1, true ) );
          cells.insert( K.uIncident( linelxm, 1, false ) );
          cells.insert( K.uIncident( linelxm, 2, true ) );
          cells.insert( K.uIncident( linelxm, 2, false ) );
          cells.insert( K.uIncident( linelyp, 2, true ) );
          cells.insert( K.uIncident( linelyp, 2, false ) );
          cells.insert( K.uIncident( linelym, 2, true ) );
          cells.insert( K.uIncident( linelym, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 2-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::cout << "<2,3> specialization" << std::endl;
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = K.uPointel( *it );
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          auto linelyp = K.uIncident( pointel, 1, true );
          auto linelym = K.uIncident( pointel, 1, false );
          cells.insert( K.uIncident( linelxp, 1, true ) );
          cells.insert( K.uIncident( linelxp, 1, false ) );
          cells.insert( K.uIncident( linelxp, 2, true ) );
          cells.insert( K.uIncident( linelxp, 2, false ) );
          cells.insert( K.uIncident( linelxm, 1, true ) );
          cells.insert( K.uIncident( linelxm, 1, false ) );
          cells.insert( K.uIncident( linelxm, 2, true ) );
          cells.insert( K.uIncident( linelxm, 2, false ) );
          cells.insert( K.uIncident( linelyp, 2, true ) );
          cells.insert( K.uIncident( linelyp, 2, false ) );
          cells.insert( K.uIncident( linelym, 2, true ) );
          cells.insert( K.uIncident( linelym, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 2-kpoints to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t> >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t> > kpoints;
      for ( auto it = itB; it != itE; ++it )
        {
          auto kp = K.uKCoords( K.uPointel( *it ) );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] - 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] - 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] + 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] + 1, kp[ 2 ]     );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ]    , kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ]    , kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ]    , kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ]    , kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] - 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] + 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] - 1, kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ]    , kp[ 1 ] + 1, kp[ 2 ] + 1 );
        }
      return kpoints;
    }

  }; // end struct CellGeometryFunctions

  /// Specialization for 3-cells in 3D.
  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace>
  struct CellGeometryFunctions< TKSpace, 3, 3 >
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    BOOST_STATIC_ASSERT( TKSpace::dimension == 3 );
    typedef TKSpace                KSpace;
    typedef typename KSpace::Space Space;
    typedef typename KSpace::Cell  Cell;

    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident 3-cells to the given range of pointels [itB, itE).
    template <typename PointelIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
                                PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = *it;
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          auto surfxpyp = K.uIncident( linelxp, 1, true );
          auto surfxpym = K.uIncident( linelxp, 1, false );
          auto surfxmyp = K.uIncident( linelxm, 1, true );
          auto surfxmym = K.uIncident( linelxm, 1, false );
          cells.insert( K.uIncident( surfxpyp, 2, true ) );
          cells.insert( K.uIncident( surfxpyp, 2, false ) );
          cells.insert( K.uIncident( surfxpym, 2, true ) );
          cells.insert( K.uIncident( surfxpym, 2, false ) );
          cells.insert( K.uIncident( surfxmyp, 2, true ) );
          cells.insert( K.uIncident( surfxmyp, 2, false ) );
          cells.insert( K.uIncident( surfxmym, 2, true ) );
          cells.insert( K.uIncident( surfxmym, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 3-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointIterator>
    static
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
                              PointIterator itB, PointIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
        {
          auto pointel = K.uPointel( *it );
          auto linelxp = K.uIncident( pointel, 0, true );
          auto linelxm = K.uIncident( pointel, 0, false );
          auto surfxpyp = K.uIncident( linelxp, 1, true );
          auto surfxpym = K.uIncident( linelxp, 1, false );
          auto surfxmyp = K.uIncident( linelxm, 1, true );
          auto surfxmym = K.uIncident( linelxm, 1, false );
          cells.insert( K.uIncident( surfxpyp, 2, true ) );
          cells.insert( K.uIncident( surfxpyp, 2, false ) );
          cells.insert( K.uIncident( surfxpym, 2, true ) );
          cells.insert( K.uIncident( surfxpym, 2, false ) );
          cells.insert( K.uIncident( surfxmyp, 2, true ) );
          cells.insert( K.uIncident( surfxmyp, 2, false ) );
          cells.insert( K.uIncident( surfxmym, 2, true ) );
          cells.insert( K.uIncident( surfxmym, 2, false ) );
        }
      return cells;
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident 3-kpoints to the given range of points [itB, itE).
    template <typename PointIterator>
    static
    UnorderedSetByBlock< typename KSpace::Point,
                         Splitter< typename KSpace::Point, uint64_t> >
    getIncidentKPointsToPoints( const KSpace& K,
                                PointIterator itB, PointIterator itE )
    {
      UnorderedSetByBlock< typename KSpace::Point,
                           Splitter< typename KSpace::Point, uint64_t> > kpoints;
      for ( auto it = itB; it != itE; ++it )
        {
          auto kp = K.uKCoords( K.uPointel( *it ) );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] - 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] - 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] + 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] + 1, kp[ 2 ] - 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] - 1, kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] - 1, kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ] - 1, kp[ 1 ] + 1, kp[ 2 ] + 1 );
          kpoints.emplace( kp[ 0 ] + 1, kp[ 1 ] + 1, kp[ 2 ] + 1 );
        }
      return kpoints;
    }

  }; // end struct CellGeometryFunctions


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "CellGeometry.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CellGeometry_h

#undef CellGeometry_RECURSES
#endif // else defined(CellGeometry_RECURSES)
