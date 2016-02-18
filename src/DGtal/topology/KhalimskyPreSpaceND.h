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
 * @file KhalimskyPreSpaceND.h
 * @author Roland Denis ( \c roland.denis@univ-smb.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2016/02/18
 *
 * Header file for module KhalimskyPreSpaceND.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(KhalimskyPreSpaceND_RECURSES)
#error Recursive header files inclusion detected in KhalimskyPreSpaceND.h
#else // defined(KhalimskyPreSpaceND_RECURSES)
/** Prevents recursive inclusion of headers. */
#define KhalimskyPreSpaceND_RECURSES

#if !defined KhalimskyPreSpaceND_h
/** Prevents repeated inclusion of headers. */
#define KhalimskyPreSpaceND_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <set>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SpaceND.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /** Represents an unsigned cell in an unbounded cellular grid space by its
   * Khalimsky coordinates.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int32).
   */
  template < Dimension dim,
             typename TInteger = DGtal::int32_t >
  struct KhalimskyPreCell
  {

    // Integer must be a model of the concept CInteger.
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ) );

    // Aliases
  public:
    using Integer = TInteger;
    using UnsignedInteger = typename NumberTraits<Integer>::UnsignedVersion;
    using Point   = PointVector< dim, Integer >;
    using Self    = KhalimskyPreCell< dim, Integer >;

    // Public members
  public:
    Point myCoordinates; ///< Khalimsky coordinates of the cell. Public to allow easy coordinate manipulations.

    // Standard interface
  public:

    /// Default constructor.
    explicit KhalimskyPreCell( Integer dummy = 0 );


    /** Explicit constructor from its Khalimsky coordinates.
     * @param aPoint Its Khalimsky coordinates as a point.
     */
    explicit KhalimskyPreCell( Point const& point );

    /** Copy constructor.
     * @param aCell any other cell.
     */
    KhalimskyPreCell( KhalimskyPreCell const& aCell ) = default;
    
    /** Copy operator
     * @param aCell any other cell.
     */
    KhalimskyPreCell & operator=( KhalimskyPreCell const& aCell ) = default;

    /** Move constructor.
     * @param aCell any other cell.
     */
    KhalimskyPreCell( KhalimskyPreCell && aCell ) = default;
    
    /** Move operator
     * @param aCell any other cell.
     */
    KhalimskyPreCell & operator=( KhalimskyPreCell && aCell ) = default;

  }; // KhalimskyPreCell


  /** Represents a signed cell in an unbounded cellular grid space by its
   * Khalimsky coordinates and a boolean value.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int32).
   */
  template < Dimension dim,
             typename TInteger = DGtal::int32_t >
  struct SignedKhalimskyPreCell
  {

    // Integer must be a model of the concept CInteger.
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ) );

    // Aliases
  public:
    using Integer = TInteger;
    using UnsignedInteger = typename NumberTraits<Integer>::UnsignedVersion;
    using Point = PointVector< dim, Integer >;

    // Public members
  public:
    Point myCoordinates;  ///< Khalimsky coordinates of the cell.
    bool  myPositive;     ///< Cell sign.

    // Standard interface
  public:

    /// Default constructor.
    explicit SignedKhalimskyPreCell( Integer dummy = 0 );

    /** Explicit constructor from its Khalimsky coordinates.
     *
     * @param aPoint Its Khalimsky coordinates as a point.
     */
    explicit SignedKhalimskyPreCell( Point const& point, bool positive );

    /** Copy constructor.
     * @param aCell any other cell.
     */
    SignedKhalimskyPreCell( SignedKhalimskyPreCell const& aCell ) = default;
    
    /** Copy operator
     * @param aCell any other cell.
     */
    SignedKhalimskyPreCell & operator=( SignedKhalimskyPreCell const& aCell ) = default;

    /** Move constructor.
     * @param aCell any other cell.
     */
    SignedKhalimskyPreCell( SignedKhalimskyPreCell && aCell ) = default;
    
    /** Move operator
     * @param aCell any other cell.
     */
    SignedKhalimskyPreCell & operator=( SignedKhalimskyPreCell && aCell ) = default;

  }; // SignedKhalimskyPreCell

  /**
     @brief This class is useful for looping on all "interesting" coordinates of a
     pre-cell. For instance, surfels in Z3 have two interesting coordinates (the
     ones spanned by the surfel).
     @code
     KSpace::PreCell p;
     KnSpace::PreDirIterator q;
     for ( q = KSpace::uDirs( p ); q != 0; ++q )
     {
     KSpace::Dimension dir = *q;
     ...
     }
     @endcode
  */
  template < Dimension dim,
             typename TInteger = DGtal::int32_t >
  class PreCellDirectionIterator
  {
  public:
    typedef TInteger Integer;
    // Cells
    typedef KhalimskyPreCell< dim, Integer > PreCell;
    typedef SignedKhalimskyPreCell< dim, Integer > SPreCell;

  public:
    /**
     * Constructor from a pre-cell.
     * @param cell any unsigned pre-cell
     */
    explicit PreCellDirectionIterator( PreCell cell, bool open = true );

    /**
     * Constructor from a signed pre-cell.
     * @param scell any signed pre-cell
     */
    explicit PreCellDirectionIterator( SPreCell scell, bool open = true );

    /**
     * @return the current direction.
     */
    Dimension operator*() const;

    /**
     * Pre-increment. Go to next direction.
     */
    PreCellDirectionIterator & operator++();

    /**
     * Fast comparison with unsigned integer (unused
     * parameter). Comparison is 'false' at the end of the iteration.
     *
     * @return 'true' if the iterator is finished.
     */
    bool operator!=( const Integer ) const;

    /**
     * @return 'true' if the iteration is ended.
     */
    bool end() const;

    /**
     * Slow comparison with other iterator. Useful to check for end of loop.
     * @param other any direction iterator.
     */
    bool operator!=( const PreCellDirectionIterator & other ) const;

    /**
     * Slow comparison with other iterator.
     * @param other any direction iterator.
     */
    bool operator==( const PreCellDirectionIterator & other ) const;

  private:
    /** the current direction. */
    Dimension myDir;
    /** the cell. */
    PreCell myCell;
    /** If 'true', returns open coordinates, otherwise returns closed
        coordinates. */
    bool myOpen;

  private:
    /** Look for next valid coordinate. */
    void find();
  };

  /////////////////////////////////////////////////////////////////////////////
  // template class KhalimskyPreSpaceND
  /**
   * Description of template class 'KhalimskyPreSpaceND' <p>
   *
   * \brief Aim: This class is a model of CCellularGridSpaceND. It
   * represents the cubical grid as a cell complex, whose cells are
   * defined as an array of integers. The topology of the cells is
   * defined by the parity of the coordinates (even: closed, odd:
   * open).
   *
   * When \b initializing the space using init(),
   * the user should choose, for each dimension spanned by the space,
   * between a closed and non-periodic (default) cell dimension,
   * an open cell dimension or a periodic cell dimension.
   * The space is generally finite, except for arbitrary size
   * integers and when the space has a periodic dimension.
   *
   * @anchor KhalimskyPreSpaceNDBounds
   * Supposing that the space has been initialized with digital bounds \c lower and \c upper,
   * the methods lowerBound() and upperBound() will always return, respectively, \c lower and \c upper.
   * It as also true for periodic dimension, in order to span over the unique digital points of the space.
   *
   * In the same way, lowerCell() and upperCell() respect the following rules:
   * - the k-th Khalimsky coordinate of lowerCell() is equal to:
   *    - `2*lower[k]` if the k-th dimension is closed or periodic,
   *    - `2*lower[k]+1` if the k-th dimension is open;
   * - the k-th Khalimsky coordinate of upperCell() is equal to:
   *    - `2*upper[k]+2` if the k-th dimension is closed,
   *    - `2*upper[k]+1` if the k-th dimension is open or periodic.
   *    .
   * .
   * The special behavior for __periodic dimensions__ guarantees that each cell has unique
   * Khalimsky coordinates in this range.
   * It is useful to span the space and also for cell-based containers (see e.g. CubicalComplex).
   * Uniqueness also gives meaning to equality tests between cells.
   *
   * Following this concept, the related methods size(), min(), max(),
   * uFirst(), uLast(), uGetMin(), uGetMax(), uDistanceToMin(), uDistanceToMax(),
   * sFirst(), sLast(), sGetMin(), sGetMax(), sDistanceToMin() and sDistanceToMax()
   * behave for periodic dimensions like for finite dimensions, using the bounds described above.
   *
   * Thus, if a cell needs to be __compared to the bounds__, prefer using dedicated tests like
   * uIsMin(), uIsMax(), sIsMin() and sIsMax() that return always \c false for a periodic dimension,
   * and uIsInside() and sIsInside() that return always \c true for a periodic dimension.
   *
   * To be consistent with those choices, each cell returned or modified by a KhalimskyPreSpaceND method
   * will have his Khalimsky coordinates along periodic dimensions between the corresponding
   * coordinates of lowerCell() and upperCell().
   * But, in order to keep low computational cost, each cell passed by parameter to a KhalimskyPreSpaceND
   * method must follow the same conditions.
   * This validity can be tested with the dedicated methods uIsValid() and sIsValid().
   *
   * Exceptions exist for uCell(const Cell &) const and sCell(const SCell &) const that are specially featured
   * to correct Khalimsky coordinates of a given cell.
   * In addition, methods returning digital or Khalimsky coordinate of a cell have a flag to control if this
   * coordinate must be corrected.
   * However, when a method accepts a coordinate as parameter, it is always corrected along periodic dimensions.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int32).
   * @note Essentially a backport from [ImaGene](https://gforge.liris.cnrs.fr/projects/imagene).
   *
   * @warning Periodic Khalimsky space and per-dimension closure specification are new features.
   * Therefore, there is no guarantee that it is compatible with the whole DGtal library.
   *
  */
  template < 
    Dimension dim,
    typename TInteger = DGtal::int32_t
  >
  class KhalimskyPreSpaceND
  {
    //Integer must be signed to characterize a ring.
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ) );

  public:
    ///Arithmetic ring induced by (+,-,*) and Integer numbers.
    using Integer = TInteger;

    ///Type used to represent sizes in the digital space.
    using Size = typename NumberTraits<Integer>::UnsignedVersion;

    // Cells
    using PreCell   = KhalimskyPreCell< dim, Integer > PreCell;
    using SPreCell  = SignedKhalimskyPreCell< dim, Integer > SPreCell;

    using Sign = bool;
    using PreDirIterator = PreCellDirectionIterator< dim, Integer >;

    // Points and Vectors
    using Point   = PointVector< dim, Integer >;
    using Vector  = PointVector< dim, Integer >;

    using Space = SpaceND<dim, Integer>;
    using KhalimskyPreSpace = KhalimskyPreSpaceND<dim, Integer>;

#if defined ( WIN32 )
    // static constants
    static const Dimension dimension = dim;
    static const Dimension DIM = dim;
    static const Sign POS = true;
    static const Sign NEG = false;
#else
    // static constants
    static const Dimension dimension = dim;
    static const Dimension DIM;
    static const Sign POS;
    static const Sign NEG;
#endif //WIN32

    template <typename CellType>
    struct AnyCellCollection : public std::deque<CellType> {
      using Value         = CellType;
      using Container     = typename std::deque<CellType>;
      using Iterator      = typename std::deque<CellType>::iterator;
      using ConstIterator = typename std::deque<CellType>::const_iterator;
    };

    // Neighborhoods, Incident cells, Faces and Cofaces
    using PreCells  = AnyCellCollection<PreCell>;
    using SPreCells = AnyCellCollection<SPreCell>;

    // Sets, Maps
    /// Preferred type for defining a set of Cell(s).
    using PreCellSet    = std::set<PreCell>;

    /// Preferred type for defining a set of SCell(s).
    using SPreCellSet   = std::set<SPreCell>;

    /// Preferred type for defining a set of surfels (always signed cells).
    using PreSurfelSet  = std::set<SPreCell>;

    /// Template rebinding for defining the type that is a mapping
    /// Cell -> Value.
    template < typename Value >
    using PreCellMap = std::map< PreCell, Value >;

    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template < typename Value > 
    using SPreCellMap = std::map< SPreCell, Value >;

    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template < typename Value >
    using PreSurfelMap = std::map< SPreCell, Value >;

    // ----------------------- Standard services ------------------------------
    /** @name Standard services
     * @{
     */
  protected:

    /**
     * Protected destructor to avoid undefined behavior with derived classes.
     */
    ~KhalimskyPreSpaceND() = default;

    /// @}

    // ----------------------- Pre-cell creation services --------------------------
    /** @name Pre-cell creation services (static methods)
     * @{
     */
  public:

    /** From the Khalimsky coordinates of a cell,
     * builds the corresponding unsigned pre-cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @return the unsigned pre-cell.
     */
    static PreCell uPreCell( const Point & kp );

    /** From the digital coordinates of a point in Zn and a cell type,
     * builds the corresponding unsigned pre-cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology.
     * @return the pre-cell having the topology of [c] and the given
     * digital coordinates [p].
     */
    static PreCell uPreCell( Point p, const PreCell & c );

    /** From the Khalimsky coordinates of a cell and a sign,
     * builds the corresponding signed pre-cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-cell.
     */
    static SPreCell sPreCell( const Point & kp, Sign sign = POS );

    /** From the digital coordinates of a point in Zn and a signed cell type,
     * builds the corresponding signed pre-cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology and sign.
     * @return the pre-cell having the topology and sign of [c] and the given
     * digital coordinates [p].
     */
    static SPreCell sPreCell( Point p, const SPreCell & c );

    /** From the digital coordinates of a point in Zn,
     * builds the corresponding pre-spel (pre-cell of maximal dimension).
     *
     * @param p an integer point (digital coordinates of cell).
     * @return the pre-spel having the given digital coordinates [p].
     */
    static PreCell uPreSpel( Point p );

    /** From the digital coordinates of a point in Zn,
     * builds the corresponding pre-spel (pre-cell of maximal dimension).
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-spel having the given digital coordinates [p].
     */
    static SPreCell sPreSpel( Point p, Sign sign = POS );

    /** From the digital coordinates of a point in Zn,
     * builds the corresponding pre-pointel (pre-cell of dimension 0).
     *
     * @param p an integer point (digital coordinates of cell).
     * @return the pre-pointel having the given digital coordinates [p].
     */
    static PreCell uPrePointel( Point p );

    /** From the digital coordinates of a point in Zn,
     * builds the corresponding pre-pointel (pre-cell of dimension 0).
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-pointel having the given digital coordinates [p].
     */
    static SPreCell sPrePointel( Point p, Sign sign = POS );

    ///@}

    // ----------------------- Read accessors to pre-cells ------------------------
    /** @name Read accessors to pre-cells
     * @{
     */
  public:
    /**
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    static Integer uKCoord( const PreCell & c, Dimension k );

    /**
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @return its digital coordinate along [k].
     */
    static Integer uCoord( const PreCell & c, Dimension k );

    /**
     * @param c any unsigned pre-cell.
     * @return its Khalimsky coordinates.
     */
    static Point uKCoords( const PreCell & c );

    /**
     * @param c any unsigned pre-cell.
     * @return its digital coordinates.
     */
    static Point uCoords( const PreCell & c );

    /**
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    static Integer sKCoord( const SPreCell & c, Dimension k );

    /**
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @return its digital coordinate along [k].
     */
    static Integer sCoord( const SPreCell & c, Dimension k );

    /**
     * @param c any signed pre-cell.
     * @return its Khalimsky coordinates.
     */
    static Point sKCoords( const SPreCell & c );

    /**
     * @param c any signed pre-cell.
     * @return its digital coordinates.
     */
    static Point sCoords( const SPreCell & c );

    /**
     * @param c any signed pre-cell.
     * @return its sign.
     */
    static Sign sSign( const SPreCell & c );

    /// @}

    // ----------------------- Write accessors to pre-cells ------------------------
    /** @name Write accessors to pre-cells
     * @{
     */
  public:

    /** Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void uPreSetKCoord( PreCell & c, Dimension k, Integer i );

    /** Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void sPreSetKCoord( SPreCell & c, Dimension k, Integer i );

    /** Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void uPreSetCoord( PreCell & c, Dimension k, Integer i );

    /** Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void sPreSetCoord( SPreCell & c, Dimension k, Integer i );

    /** Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any unsigned pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void uPreSetKCoords( PreCell & c, const Point & kp );

    /** Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any signed pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void sPreSetKCoords( SPreCell & c, const Point & kp );

    /** Sets the digital coordinates of [c] to [kp].
     * @param c any unsigned pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void uPreSetCoords( PreCell & c, const Point & kp );

    /** Sets the digital coordinates of [c] to [kp].
     * @param c any signed pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void sPreSetCoords( SPreCell & c, const Point & kp );

    /** Sets the sign of the pre-cell.
     * @param c (modified) any signed pre-cell.
     * @param s any sign.
     */
    static void sPreSetSign( SPreCell & c, Sign s );

    /// @}

    // -------------------- Conversion signed/unsigned ------------------------
    /** @name Conversion signed/unsigned
     * @{
     */
  public:
    /** Creates a signed pre-cell from an unsigned one and a given sign.
     * @param p any unsigned pre-cell.
     * @param s a sign.
     * @return the signed version of the pre-cell [p] with sign [s].
     */
    static SPreCell signs( const PreCell & p, Sign s );

    /** Creates an unsigned pre-cell from a signed one.
     * @param p any signed pre-cell.
     * @return the unsigned version of the pre-cell [p].
     */
    static PreCell unsigns( const SPreCell & p );

    /**
     * Creates the signed pre-cell with the inverse sign of [p].
     * @param p any signed pre-cell.
     * @return the pre-cell [p] with opposite sign.
     */
    static SPreCell sOpp( const SPreCell & p );

    /// @}

    // ------------------------- Pre-cell topology services -----------------------
    /** @name Pre-cell topology services
     * @{
     */
  public:
    /**
     * @param p any unsigned pre-cell.
     * @return the topology word of [p].
     */
    static Integer uTopology( const PreCell & p );

    /**
     * @param p any signed pre-cell.
     * @return the topology word of [p].
     */
    static Integer sTopology( const SPreCell & p );

    /**
     * @param p any unsigned pre-cell.
     * @return the dimension of the pre-cell [p].
     */
    static Dimension uDim( const PreCell & p );

    /**
     * @param p any signed pre-cell.
     * @return the dimension of the pre-cell [p].
     */
    static Dimension sDim( const SPreCell & p );

    /**
     * @param b any unsigned pre-cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    static bool uIsSurfel( const PreCell & b );

    /**
     * @param b any signed pre-cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    static bool sIsSurfel( const SPreCell & b );

    /**
     * @param p any pre-cell.
     * @param k any direction.
     * @return 'true' if [p] is open along the direction [k].
     */
    static bool uIsOpen( const PreCell & p, Dimension k );

    /**
     * @param p any signed pre-cell.
     * @param k any direction.
     * @return 'true' if [p] is open along the direction [k].
     */
    static bool sIsOpen( const SPreCell & p, Dimension k );

    /// @}

    // -------------------- Iterator services for pre-cells ------------------------
    /** @name Iterator services for cells
     * @{
     */
  public:

    /** Given an unsigned pre-cell [p], returns an iterator to iterate over
     * each coordinate the cell spans. (A spel spans all coordinates; a
     * surfel all but one, etc). Example:
     *
     * @code
     * KSpace::PreCell p;
     * ...
     * for ( auto q = KSpace::uDirs( p ); q != 0; ++q )
     * {
     *   Dimension dir = *q;
     * ...
     * }
     * @endcode
     *
     * @param p any unsigned pre-cell.
     * @return an iterator that points on the first coordinate spanned
     * by the pre-cell.
     */
    static PreDirIterator uDirs( const PreCell & p );

    /** Given a signed pre-cell [p], returns an iterator to iterate over
     * each coordinate the cell spans. (A spel spans all coordinates; a
     * surfel all but one, etc). Example:
     *
     * @code
     * KSpace::SPreCell p;
     * ...
     * for ( auto q = KSpace::uDirs( p ); q != 0; ++q )
     * {
     *   Dimension dir = *q;
     * ...
     * }
     *  @endcode
     *
     * @param p any signed pre-cell.
     * @return an iterator that points on the first coordinate spanned
     * by the pre-cell.
     */
    static PreDirIterator sDirs( const SPreCell & p );

    /** Given an unsigned pre-cell [p], returns an iterator to iterate over each
     * coordinate the cell does not span. (A spel spans all coordinates;
     * a surfel all but one, etc). Example:
     *
     * @code
     * KSpace::PreCell p;
     * ...
     * for ( auto q = KSpace::uOrthDirs( p ); q != 0; ++q )
     * {
     *   Dimension dir = *q;
     * ...
     * }
     * @endcode
     *
     * @param p any unsigned pre-cell.
     * @return an iterator that points on the first coordinate spanned
     * by the cell.
     */
    static PreDirIterator uOrthDirs( const PreCell & p );

    /** Given a signed pre-cell [p], returns an iterator to iterate over each
     * coordinate the cell does not span. (A spel spans all coordinates;
     * a surfel all but one, etc). Example:
     *
     * @code
     * KSpace::SPreCell p;
     * ...
     * for ( auto q = KSpace::uOrthDirs( p ); q != 0; ++q )
     * {
     *   Dimension dir = *q;
     * ...
     * }
     * @endcode
     *
     * @param p any signed pre-cell.
     * @return an iterator that points on the first coordinate spanned
     * by the cell.
     */
    static PreDirIterator sOrthDirs( const SPreCell & p );

    /** Given an unsigned pre-surfel [s], returns its orthogonal direction (ie,
     * the coordinate where the surfel is closed).
     *
     * @param s an unsigned pre-surfel
     * @return the orthogonal direction of [s]
     */
    static Dimension uOrthDir( const PreCell & s );

    /** Given a signed pre-surfel [s], returns its orthogonal direction (ie,
     * the coordinate where the surfel is closed).
     *
     * @param s a signed pre-surfel
     * @return the orthogonal direction of [s]
     */
    static Dimension sOrthDir( const SPreCell & s );

    /// @}

    // -------------------- Unsigned pre-cell geometry services --------------------
    /** @name Unsigned pre-cell geometry services
     * @{
     */
  public:

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for the incremented
     * coordinate [k].
     */
    static PreCell uGetIncr( PreCell p, Dimension k );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for an decremented
     * coordinate [k].
     */
    static PreCell uGetDecr( PreCell p, Dimension k );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the increment.
     * @return the same element as [p] except for a coordinate [k]
     * incremented with x.
     */
    static PreCell uGetAdd( PreCell p, Dimension k, Integer x );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the decrement.
     * @return the same element as [p] except for a coordinate [k]
     * decremented with x.
     */
    static PreCell uGetSub( PreCell p, Dimension k, Integer x );

    /** Add the vector [vec] to [p].
     *
     * @param p any pre-cell.
     * @param vec any pointel.
     * @return the unsigned copy of the pre-cell [p] translated by [coord].
     */
    static PreCell uTranslation( PreCell p, const Vector & vec );

    /** Return the projection of [p] along the [k]th direction toward
     *  [bound]. Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @return the projection.
     * @pre  `uIsOpen(p, k) == uIsOpen(bound, k)`
     * @post `uTopology(p) == uTopology(uProjection(p, bound, k))`.
     */
    static PreCell uProjection( PreCell p, const PreCell & bound, Dimension k );

    /** Projects [p] along the [k]th direction toward
     *  [bound]. Otherwise said, p[ k ] == bound[ k ] afterwards
     *
     * @param [in,out] p any pre-cell.
     * @param [in] bound the element acting as bound (same topology as p).
     * @param [in] k the concerned coordinate.
     * @pre  `uIsOpen(p, k) == uIsOpen(bound, k)`
     */
    static void uProject( PreCell & p, const PreCell & bound, Dimension k );

    /** Increment the pre-cell [p] to its next position (as classically done in
     *  a scanning). Example:
     *
     * \code
     * PreCell first, last; // lower and upper bounds
     * PreCell p = first;
     * do
     * { // ... whatever [p] is the current cell
     * }
     * while ( KSpace::uNext( p, first, last ) );
     * \endcode
     *
     * @param p any pre-cell.
     * @param lower the lower bound.
     * @param upper the upper bound.
     * @return true if p is still within the bounds, false if the
     * scanning is finished.
     * @pre `uTopology(p) == uTopology(lower) == uTopology(upper)`.
     */
    static bool uNext( PreCell & p, const PreCell & lower, const PreCell & upper );

    /// @}

    // -------------------- Signed pre-cell geometry services --------------------
    /** @name Signed pre-cell geometry services
     * @{
     */
  public:

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for the incremented
     * coordinate [k].
     */
    static SPreCell sGetIncr( SPreCell p, Dimension k );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the increment.
     * @return the same element as [p] except for a coordinate [k]
     * incremented with x.
     */
    static SPreCell sGetAdd( SPreCell p, Dimension k, Integer x );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the decrement.
     * @return the same element as [p] except for a coordinate [k]
     * decremented with x.
     */
    static SPreCell sGetSub( SPreCell p, Dimension k, Integer x );

    /** Add the vector [vec] to [p].
     *
     * @param p any pre-cell.
     * @param vec any pointel.
     * @return the signed code of the cell [p] translated by [coord].
     */
    static SPreCell sTranslation( SPreCell p, const Vector & vec );

    /** Return the projection of [p] along the [k]th direction toward
     *  [bound]. Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @return the projection.
     * @pre  `sIsOpen(p, k) == sIsOpen(bound, k)`
     * @post `sTopology(p) == sTopology(sProjection(p, bound, k))`.
     */
    static SPreCell sProjection( SPreCell p, const SPreCell & bound, Dimension k );

    /** Projects [p] along the [k]th direction toward
     *  [bound]. Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @pre  `sIsOpen(p, k) == sIsOpen(bound, k)`
     */
    static void sProject( SPreCell & p, const SPreCell & bound, Dimension k );

    /** Increment the pre-cell [p] to its next position (as classically done in
     *  a scanning). Example:

     * \code
     * PreCell first, last; // lower and upper bounds
     * PreCell p = first;
     * do
     * { // ... whatever [p] is the current cell
     * }
     * while ( KSpace::uNext( p, first, last ) );
     * \endcode
     *
     * @param p any pre-cell.
     * @param lower the lower bound.
     * @param upper the upper bound.
     * @return true if p is still within the bounds, false if the
     * scanning is finished.
     * @pre `sTopology(p) == sTopology(lower) == sTopology(upper)`.
     */
    static bool sNext( SPreCell & p, const SPreCell & lower, const SPreCell & upper );

    /// @}

    // ----------------------- Neighborhood services --------------------------
    /** @name Neighborhood services
     * @{
     */
  public:

    /** Computes the 1-neighborhood of the pre-cell [c] and returns
     *  it. It is the set of cells with same topology that are adjacent
     *  to [c].
     *
     * @param cell the unsigned pre-cell of interest.
     * @return the pre-cells of the 1-neighborhood of [cell].
     */
    static PreCells uNeighborhood( const PreCell & cell );

    /** Computes the 1-neighborhood of the pre-cell [c] and returns
     *  it. It is the set of pre-cells with same topology that are adjacent
     *  to [c].
     *
     * @param cell the signed pre-cell of interest.
     * @return the pre-cells of the 1-neighborhood of [cell].
     */
    static SPreCells sNeighborhood( const SPreCell & cell );

    /** Computes the proper 1-neighborhood of the pre-cell [c] and returns
     *  it. It is the set of pre-cells with same topology that are adjacent
     *  to [c] and different from [c].
     *
     * @param cell the unsigned pre-cell of interest.
     * @return the pre-cells of the proper 1-neighborhood of [cell].
     */
    static PreCells uProperNeighborhood( const PreCell & cell );

    /** Computes the proper 1-neighborhood of the pre-cell [c] and returns
     *  it. It is the set of pre-cells with same topology that are adjacent
     *  to [c] and different from [c].
     *
     * @param cell the signed pre-cell of interest.
     * @return the pre-cells of the proper 1-neighborhood of [cell].
     */
    statuc SPreCells sProperNeighborhood( const SPreCell & cell );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     * @note It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.
     */
    static PreCell uAdjacent( const PreCell & p, Dimension k, bool up );

    /**
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     * @note It is an alias to 'up ? sGetIncr( p, k ) : sGetDecr( p, k )'.
     */
    static SPreCell sAdjacent( const SPreCell & p, Dimension k, bool up );

    /// @}

    // ----------------------- Incidence services --------------------------
    /** @name Incidence services
     * @{
     */
  public:

    /**
     * @param c any unsigned pre-cell.
     * @param k any coordinate.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the forward or backward unsigned pre-cell incident to [c]
     * along axis [k], depending on [up].
     * @note It may be a lower incident pre-cell if [c] is open along axis
     * [k], else an upper incident pre-cell.
     */
    static PreCell uIncident( PreCell c, Dimension k, bool up );

    /**
     * @param c any signed pre-cell.
     * @param k any coordinate.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the forward or backward signed pre-cell incident to [c]
     * along axis [k], depending on [up]. It is worthy to note
     * that the forward and backward pre-cell have opposite
     * sign. Furthermore, the sign of these pre-cells is defined so as to
     * satisfy a boundary operator.
     * @note It may be a lower incident pre-cell if [c] is open along axis
     * [k], else an upper incident pre-cell.
     */
    static SPreCell sIncident( SPreCell c, Dimension k, bool up );

    /**
     * @param c any unsigned pre-cell.
     * @return the pre-cells directly low incident to c.
     */
    static PreCells uLowerIncident( const PreCell & c );

    /**
     * @param c any unsigned pre-cell.
     * @return the pre-cells directly up incident to c.
     */
    static PreCells uUpperIncident( const PreCell & c );

    /**
     *  @param c any signed pre-cell.
     *  @return the signed pre-cells directly low incident to c.
     *  @note it is the lower boundary of c expressed as a list of signed pre-cells.
     */
    static SPreCells sLowerIncident( const SPreCell & c );

    /**
     * @param c any signed pre-cell.
     * @return the signed pre-cells directly up incident to c.
     * @note it is the upper boundary of c expressed as a list of signed pre-cells.
     */
    static SPreCells sUpperIncident( const SPreCell & c );

    /**
     * @param c any unsigned pre-cell.
     * @return the proper faces of [c] (chain of lower incidence).
     */
    static PreCells uFaces( const PreCell & c );

    /**
     * @param c any unsigned ipre-cell.
     * @return the proper cofaces of [c] (chain of upper incidence).
     */
    static PreCells uCoFaces( const PreCell & c );

    /** Return 'true' if the direct orientation of [p] along [k] is in
     *  the positive coordinate direction. The direct orientation in a
     *  direction allows to go from positive incident pre-cells to positive
     *  incident pre-cells.  This means that
     *  @code
     *  KSpace::sSign( KSpace::sIncident( p, k, KSpace::sDirect( p, k ) ) ) == K.POS
     *  @endcode
     *  is always true.
     *
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the direct orientation of [p] along [k] (true is
     * upward, false is backward).
     */
    static bool sDirect( const SPreCell & p, Dimension k );

    /**
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the direct incident pre-cell of [p] along [k] (the incident
     * pre-cell along [k])
     */
    static SPreCell sDirectIncident( SPreCell p, Dimension k );

    /**
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the indirect incident pre-cell of [p] along [k] (the incident
     * cell along [k] whose sign is negative).
    */
    static SPreCell sIndirectIncident( SPreCell p, Dimension k );

    /// @}


    // ------------------------- Internals ------------------------------------
    /** @name Internals
     * @{
     */
  private:
    /**
     * Used by uFaces for computing incident faces.
     */
    static void uAddFaces( PreCells & faces, const PreCell & c, Dimension axis );

    /**
     * Used by uCoFaces for computing incident cofaces.
     */
    static void uAddCoFaces( PreCells & cofaces, const PreCell & c, Dimension axis );

    /// @}
    
    // ----------------------- Interface --------------------------------------
    /** @name DGtal interface
     * @{
     */
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    static void selfDisplay ( std::ostream & out );

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    static constexpr bool isValid();

    /// @}

  }; // end of class KhalimskyPreSpaceND

  /**
   * Overloads 'operator<<' for displaying objects of class 'KhalimskyPreSpaceND'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'KhalimskyPreSpaceND' to write.
   * @return the output stream after the writing.
   */
  template < Dimension dim,
             typename TInteger >
  std::ostream&
  operator<< ( std::ostream & out,
               const KhalimskyPreSpaceND<dim, TInteger > & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/KhalimskyPreSpaceND.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined KhalimskyPreSpaceND_h

#undef KhalimskyPreSpaceND_RECURSES
#endif // else defined(KhalimskyPreSpaceND_RECURSES)
