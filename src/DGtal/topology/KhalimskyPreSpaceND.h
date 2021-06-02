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
#include <DGtal/base/Common.h>
#include <DGtal/kernel/CInteger.h>
#include <DGtal/kernel/PointVector.h>
#include <DGtal/kernel/SpaceND.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // Pre-declaration
  template <
      Dimension dim,
      typename TInteger = DGtal::int32_t
  >
  class KhalimskyPreSpaceND;

  /////////////////////////////////////////////////////////////////////////////
  /** @brief Represents an unsigned cell in an unbounded cellular grid space by its
   * Khalimsky coordinates.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int32).
   */
  template < Dimension dim,
             typename TInteger = DGtal::int32_t >
  struct KhalimskyPreCell
  {

    /// Integer must be a model of the concept CInteger.
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ) );

    // Aliases
  public:
    using Integer = TInteger;
    using UnsignedInteger = typename NumberTraits<Integer>::UnsignedVersion;
    using Point   = PointVector< dim, Integer >;
    using Self    = KhalimskyPreCell< dim, Integer >;
    using PreCell = Self;
    using PreCellularGridSpace = KhalimskyPreSpaceND< dim, TInteger >;
    using CellularGridSpace    = PreCellularGridSpace;

    // Public members
  public:
    Point coordinates; ///< Khalimsky coordinates of the cell. Public to allow easy coordinate manipulations.

    // Standard interface
  public:

    /// Returns the underlying constant pre-cell, itself in fact.
    PreCell const& preCell() const;

    /// Default constructor.
    explicit KhalimskyPreCell( Integer dummy = 0 );


    /** @brief Implicit constructor from its Khalimsky coordinates.
     * @param aPoint Its Khalimsky coordinates as a point.
     */
    KhalimskyPreCell( Point const& aPoint );

    /** @brief Copy constructor.
     * @param aCell any other pre-cell.
     */
    KhalimskyPreCell( KhalimskyPreCell const& aCell ) = default;

    /** @brief Copy operator
     * @param aCell any other pre-cell.
     */
    KhalimskyPreCell & operator=( KhalimskyPreCell const& aCell ) = default;

    /** @brief Move constructor.
     * @param aCell any other pre-cell.
     */
    KhalimskyPreCell( KhalimskyPreCell && aCell ) = default;

    /** @brief Move operator
     * @param aCell any other pre-cell.
     */
    KhalimskyPreCell & operator=( KhalimskyPreCell && aCell ) = default;

    /** @brief
     * Equality operator.
     * @param other any other pre-cell.
     */
    bool operator==( const KhalimskyPreCell & other ) const;

    /** @brief
     * Difference operator.
     * @param other any other pre-cell.
    */
    bool operator!=( const KhalimskyPreCell & other ) const;

    /** @brief
     * Inferior operator. (lexicographic order).
     * @param other any other pre-cell.
    */
    bool operator<( const KhalimskyPreCell & other ) const;

    // --------------- CDrawableWithBoard2D realization -------------------
  public:

    /** @brief Return the style name used for drawing this object.
     *
     * @return the style name used for drawing this object.
     */
    std::string className() const;

  }; // KhalimskyPreCell

  template < Dimension dim,
             typename TInteger >
  std::ostream &
  operator<<( std::ostream & out,
              const KhalimskyPreCell< dim, TInteger > & object );

  /** @brief Represents a signed cell in an unbounded cellular grid space by its
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
    using Point   = PointVector< dim, Integer >;
    using Self    = SignedKhalimskyPreCell< dim, Integer >;
    using SPreCell = Self;
    using PreCellularGridSpace = KhalimskyPreSpaceND< dim, TInteger >;
    using CellularGridSpace    = PreCellularGridSpace;

    // Public members
  public:
    Point coordinates;  ///< Khalimsky coordinates of the cell.
    bool  positive;     ///< Cell sign.

    // Standard interface
  public:

    /// Returns the underlying constant pre-cell, itself in fact.
    SPreCell const& preCell() const;

    /// Default constructor.
    explicit SignedKhalimskyPreCell( Integer dummy = 0 );

    /** @brief Implicit constructor from its Khalimsky coordinates.
     *
     * @param aPoint Its Khalimsky coordinates as a point.
     * @param positive if cell has positive sign.
     */
    SignedKhalimskyPreCell( Point const& aPoint, bool positive );

    /** @brief Copy constructor.
     * @param aCell any other pre-cell.
     */
    SignedKhalimskyPreCell( SignedKhalimskyPreCell const& aCell ) = default;

    /** @brief Copy operator
     * @param aCell any other pre-cell.
     */
    SignedKhalimskyPreCell & operator=( SignedKhalimskyPreCell const& aCell ) = default;

    /** @brief Move constructor.
     * @param aCell any other pre-cell.
     */
    SignedKhalimskyPreCell( SignedKhalimskyPreCell && aCell ) = default;

    /** @brief Move operator
     * @param aCell any other pre-cell.
     */
    SignedKhalimskyPreCell & operator=( SignedKhalimskyPreCell && aCell ) = default;

    /** @brief
     * Equality operator.
     * @param other any other pre-cell.
     */
    bool operator==( const SignedKhalimskyPreCell & other ) const;

    /** @brief
     * Difference operator.
     * @param other any other pre-cell.
     */
    bool operator!=( const SignedKhalimskyPreCell & other ) const;

    /** @brief
     * Inferior operator. (lexicographic order).
     * @param other any other pre-cell.
     */
    bool operator<( const SignedKhalimskyPreCell & other ) const;

    // --------------- CDrawableWithBoard2D realization -------------------
  public:

    /** @brief Return the style name used for drawing this object.
     *
     * @return the style name used for drawing this object.
     */
    std::string className() const;

  }; // SignedKhalimskyPreCell

  template < Dimension dim,
             typename TInteger >
  std::ostream &
  operator<<( std::ostream & out,
              const SignedKhalimskyPreCell< dim, TInteger > & object );

  /**
     @brief This class is useful for looping on all "interesting" coordinates of a
     pre-cell.

     For instance, surfels in Z3 have two interesting coordinates (the
     ones spanned by the surfel).

     @code
     KPreSpace::Cell p;
     KPreSpace::DirIterator q;
     for ( q = KPreSpace::uDirs( p ); q != 0; ++q )
     {
     KPreSpace::Dimension dir = *q;
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
    typedef KhalimskyPreCell< dim, Integer > Cell;
    typedef SignedKhalimskyPreCell< dim, Integer > SCell;

  public:
    /** @brief
     * Constructor from a pre-cell.
     * @param cell any unsigned pre-cell
     * @param open 'true' if the cell if open.
     */
    explicit PreCellDirectionIterator( Cell cell, bool open = true );

    /** @brief
     * Constructor from a signed pre-cell.
     * @param scell any signed pre-cell
     * @param open 'true' if the cell if open.
     */
    explicit PreCellDirectionIterator( SCell scell, bool open = true );

    /** @brief Return the current direction.
     *
     * @return the current direction.
     */
    Dimension operator*() const;

    /// Pre-increment. Go to next direction.
    PreCellDirectionIterator & operator++();

    /** @brief Fast comparison with unsigned integer (unused
     * parameter).
     *
     * Comparison is 'false' at the end of the iteration.
     *
     * @return 'true' if the iterator is finished.
     */
    bool operator!=( const Integer ) const;

    /** @brief Return 'true' if the iteration is ended.
     *
     * @return 'true' if the iteration is ended.
     */
    bool end() const;

    /** @brief
     * Slow comparison with other iterator. Useful to check for end of loop.
     * @param other any direction iterator.
     */
    bool operator!=( const PreCellDirectionIterator & other ) const;

    /** @brief
     * Slow comparison with other iterator.
     * @param other any direction iterator.
     */
    bool operator==( const PreCellDirectionIterator & other ) const;

  private:
    /// the current direction.
    Dimension myDir;
    /// the cell.
    Cell myCell;
    /** @brief If 'true', returns open coordinates, otherwise returns closed
     *  coordinates.
     */
    bool myOpen;

  private:
    /// Look for next valid coordinate.
    void find();
  };

  /////////////////////////////////////////////////////////////////////////////
  // template class KhalimskyPreSpaceND
  /**
   * Description of template class 'KhalimskyPreSpaceND' <p>
   *
   * \brief Aim: This class is a model of CPreCellularGridSpaceND. It
   * represents the cubical grid as a cell complex, whose cells are
   * defined as an array of integers. The topology of the cells is
   * defined by the parity of the coordinates (even: closed, odd:
   * open).
   *
   * This cellular grid space has no bounds (depending on the Integer type)
   * and thus only provides static methods.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int32).
   * @note Essentially a backport from ImaGene.
   * @see KhalimskySpaceND
   *
   */
  template <
    Dimension dim,
    typename TInteger
  >
  class KhalimskyPreSpaceND
  {
    /// Integer must be signed to characterize a ring.
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ) );

  public:
    /// Arithmetic ring induced by (+,-,*) and Integer numbers.
    using Integer = TInteger;

    // Cells
    using Cell   = KhalimskyPreCell< dim, Integer >;
    using SCell  = SignedKhalimskyPreCell< dim, Integer >;

    using Surfel = SCell;
    using Sign = bool;
    using DirIterator = PreCellDirectionIterator< dim, Integer >;

    // Points and Vectors
    using Point   = PointVector< dim, Integer >;
    using Vector  = PointVector< dim, Integer >;

    using Space = SpaceND<dim, Integer>;
    using PreCellularGridSpace = KhalimskyPreSpaceND<dim, Integer>;

    // static constants
    static constexpr const Dimension dimension = dim;
    static constexpr const Dimension DIM = dim;
    static constexpr const Sign POS = true;
    static constexpr const Sign NEG = false;

    template <typename CellType>
    struct AnyCellCollection : public std::deque<CellType> {
      using Value         = CellType;
      using Container     = typename std::deque<CellType>;
      using Iterator      = typename std::deque<CellType>::iterator;
      using ConstIterator = typename std::deque<CellType>::const_iterator;
    };

    // Neighborhoods, Incident cells, Faces and Cofaces
    using Cells  = AnyCellCollection<Cell>;
    using SCells = AnyCellCollection<SCell>;

    // Sets, Maps
    /// Preferred type for defining a set of Cell(s).
    using CellSet    = std::set<Cell>;

    /// Preferred type for defining a set of SCell(s).
    using SCellSet   = std::set<SCell>;

    /// Preferred type for defining a set of surfels (always signed cells).
    using SurfelSet  = std::set<SCell>;

    /// Template rebinding for defining the type that is a mapping
    /// Cell -> Value.
    template <typename Value> struct CellMap {
        typedef std::map<Cell,Value> Type;
    };

    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template <typename Value> struct SCellMap {
        typedef std::map<SCell,Value> Type;
    };

    /// Template rebinding for defining the type that is a mapping
    /// SCell -> Value.
    template <typename Value> struct SurfelMap {
        typedef std::map<SCell,Value> Type;
    };

    // ----------------------- Pre-cell creation services --------------------------
    /** @name Pre-cell creation services (static methods)
     * @{
     */
  public:

    /** @brief From the Khalimsky coordinates of a cell,
     * builds the corresponding unsigned pre-cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @return the unsigned pre-cell.
     */
    static Cell uCell( const Point & kp );

    /** @brief From the digital coordinates of a point in Zn and a cell type,
     * builds the corresponding unsigned pre-cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology.
     * @return the pre-cell having the topology of [c] and the given
     * digital coordinates [p].
     */
    static Cell uCell( Point p, const Cell & c );

    /** @brief From the Khalimsky coordinates of a cell and a sign,
     * builds the corresponding signed pre-cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-cell.
     */
    static SCell sCell( const Point & kp, Sign sign = POS );

    /** @brief From the digital coordinates of a point in Zn and a signed cell type,
     * builds the corresponding signed pre-cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology and sign.
     * @return the pre-cell having the topology and sign of [c] and the given
     * digital coordinates [p].
     */
    static SCell sCell( Point p, const SCell & c );

    /** @brief From the digital coordinates of a point in Zn,
     * builds the corresponding pre-spel (pre-cell of maximal dimension).
     *
     * @param p an integer point (digital coordinates of cell).
     * @return the pre-spel having the given digital coordinates [p].
     */
    static Cell uSpel( Point p );

    /** @brief From the digital coordinates of a point in Zn,
     * builds the corresponding pre-spel (pre-cell of maximal dimension).
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-spel having the given digital coordinates [p].
     */
    static SCell sSpel( Point p, Sign sign = POS );

    /** @brief From the digital coordinates of a point in Zn,
     * builds the corresponding pre-pointel (pre-cell of dimension 0).
     *
     * @param p an integer point (digital coordinates of cell).
     * @return the pre-pointel having the given digital coordinates [p].
     */
    static Cell uPointel( Point p );

    /** @brief From the digital coordinates of a point in Zn,
     * builds the corresponding pre-pointel (pre-cell of dimension 0).
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the signed pre-pointel having the given digital coordinates [p].
     */
    static SCell sPointel( Point p, Sign sign = POS );

    ///@}

    // ----------------------- Read accessors to pre-cells ------------------------
    /** @name Read accessors to pre-cells
     * @{
     */
  public:
    /** @brief Return its Khalimsky coordinate along [k].

     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    static Integer uKCoord( const Cell & c, Dimension k );

    /** @brief Return its digital coordinate along [k].

     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @return its digital coordinate along [k].
     */
    static Integer uCoord( const Cell & c, Dimension k );

    /** @brief Return its Khalimsky coordinates.

     * @param c any unsigned pre-cell.
     * @return its Khalimsky coordinates.
     */
    static const Point & uKCoords( const Cell & c );

    /** @brief Return its digital coordinates.

     * @param c any unsigned pre-cell.
     * @return its digital coordinates.
     */
    static Point uCoords( const Cell & c );

    /** @brief Return its Khalimsky coordinate along [k].

     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    static Integer sKCoord( const SCell & c, Dimension k );

    /** @brief Return its digital coordinate along [k].

     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @return its digital coordinate along [k].
     */
    static Integer sCoord( const SCell & c, Dimension k );

    /** @brief Return its Khalimsky coordinates.

     * @param c any signed pre-cell.
     * @return its Khalimsky coordinates.
     */
    static const Point & sKCoords( const SCell & c );

    /** @brief Return its digital coordinates.

     * @param c any signed pre-cell.
     * @return its digital coordinates.
     */
    static Point sCoords( const SCell & c );

    /** @brief Return its sign.

     * @param c any signed pre-cell.
     * @return its sign.
     */
    static Sign sSign( const SCell & c );

    /// @}

    // ----------------------- Write accessors to pre-cells ------------------------
    /** @name Write accessors to pre-cells
     * @{
     */
  public:

    /** @brief Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void uSetKCoord( Cell & c, Dimension k, Integer i );

    /** @brief Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void sSetKCoord( SCell & c, Dimension k, Integer i );

    /** @brief Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any unsigned pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void uSetCoord( Cell & c, Dimension k, Integer i );

    /** @brief Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any signed pre-cell.
     * @param k any valid dimension.
     * @param i an integer coordinate.
     */
    static void sSetCoord( SCell & c, Dimension k, Integer i );

    /** @brief Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any unsigned pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void uSetKCoords( Cell & c, const Point & kp );

    /** @brief Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any signed pre-cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    static void sSetKCoords( SCell & c, const Point & kp );

    /** @brief Sets the digital coordinates of [c] to [kp].
     * @param c any unsigned pre-cell.
     * @param kp the new digital coordinates for [c].
     */
    static void uSetCoords( Cell & c, const Point & kp );

    /** @brief Sets the digital coordinates of [c] to [kp].
     * @param c any signed pre-cell.
     * @param kp the new digital coordinates for [c].
     */
    static void sSetCoords( SCell & c, const Point & kp );

    /** @brief Sets the sign of the pre-cell.
     * @param c (modified) any signed pre-cell.
     * @param s any sign.
     */
    static void sSetSign( SCell & c, Sign s );

    /// @}

    // -------------------- Conversion signed/unsigned ------------------------
    /** @name Conversion signed/unsigned
     * @{
     */
  public:
    /** @brief Creates a signed pre-cell from an unsigned one and a given sign.
     * @param p any unsigned pre-cell.
     * @param s a sign.
     * @return the signed version of the pre-cell [p] with sign [s].
     */
    static SCell signs( const Cell & p, Sign s );

    /** @brief Creates an unsigned pre-cell from a signed one.
     * @param p any signed pre-cell.
     * @return the unsigned version of the pre-cell [p].
     */
    static Cell unsigns( const SCell & p );

    /** @brief Creates the signed pre-cell with the inverse sign of [p].
     * @param p any signed pre-cell.
     * @return the pre-cell [p] with opposite sign.
     */
    static SCell sOpp( const SCell & p );

    /// @}

    // ------------------------- Pre-cell topology services -----------------------
    /** @name Pre-cell topology services
     * @{
     */
  public:
    /** @brief Return the topology word of [p].
     * @param p any unsigned pre-cell.
     * @return the topology word of [p].
     */
    static Integer uTopology( const Cell & p );

    /** @brief Return the topology word of [p].

     * @param p any signed pre-cell.
     * @return the topology word of [p].
     */
    static Integer sTopology( const SCell & p );

    /** @brief Return the dimension of the pre-cell [p].

     * @param p any unsigned pre-cell.
     * @return the dimension of the pre-cell [p].
     */
    static Dimension uDim( const Cell & p );

    /** @brief Return the dimension of the pre-cell [p].

     * @param p any signed pre-cell.
     * @return the dimension of the pre-cell [p].
     */
    static Dimension sDim( const SCell & p );

    /** @brief Return 'true' if [b] is a surfel (spans all but one coordinate).

     * @param b any unsigned pre-cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    static bool uIsSurfel( const Cell & b );

    /** @brief Return 'true' if [b] is a surfel (spans all but one coordinate).

     * @param b any signed pre-cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    static bool sIsSurfel( const SCell & b );

    /** @brief Return 'true' if [p] is open along the direction [k].

     * @param p any pre-cell.
     * @param k any direction.
     * @return 'true' if [p] is open along the direction [k].
     */
    static bool uIsOpen( const Cell & p, Dimension k );

    /** @brief Return 'true' if [p] is open along the direction [k].

     * @param p any signed pre-cell.
     * @param k any direction.
     * @return 'true' if [p] is open along the direction [k].
     */
    static bool sIsOpen( const SCell & p, Dimension k );

    /// @}

    // -------------------- Iterator services for pre-cells ------------------------
    /** @name Iterator services for cells
     * @{
     */
  public:

    /** @brief Given an unsigned pre-cell [p], returns an iterator to iterate over
     * each coordinate the cell spans. (A spel spans all coordinates; a
     * surfel all but one, etc).
     *
     * Example:
     *
     * @code
     * KPreSpace::Cell p;
     * ...
     * for ( auto q = KPreSpace::uDirs( p ); q != 0; ++q )
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
    static DirIterator uDirs( const Cell & p );

    /** @brief Given a signed pre-cell [p], returns an iterator to iterate over
     * each coordinate the cell spans. (A spel spans all coordinates; a
     * surfel all but one, etc).
     *
     * Example:
     *
     * @code
     * KPreSpace::SCell p;
     * ...
     * for ( auto q = KPreSpace::uDirs( p ); q != 0; ++q )
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
    static DirIterator sDirs( const SCell & p );

    /** @brief Given an unsigned pre-cell [p], returns an iterator to iterate over each
     * coordinate the cell does not span. (A spel spans all coordinates;
     * a surfel all but one, etc).
     *
     * Example:
     *
     * @code
     * KPreSpace::Cell p;
     * ...
     * for ( auto q = KPreSpace::uOrthDirs( p ); q != 0; ++q )
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
    static DirIterator uOrthDirs( const Cell & p );

    /** @brief Given a signed pre-cell [p], returns an iterator to iterate over each
     * coordinate the cell does not span. (A spel spans all coordinates;
     * a surfel all but one, etc).
     *
     * Example:
     *
     * @code
     * KPreSpace::SCell p;
     * ...
     * for ( auto q = KPreSpace::uOrthDirs( p ); q != 0; ++q )
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
    static DirIterator sOrthDirs( const SCell & p );

    /** @brief Given an unsigned pre-surfel [s], returns its orthogonal direction (ie,
     * the coordinate where the surfel is closed).
     *
     * @param s an unsigned pre-surfel
     * @return the orthogonal direction of [s]
     */
    static Dimension uOrthDir( const Cell & s );

    /** @brief Given a signed pre-surfel [s], returns its orthogonal direction (ie,
     * the coordinate where the surfel is closed).
     *
     * @param s a signed pre-surfel
     * @return the orthogonal direction of [s]
     */
    static Dimension sOrthDir( const SCell & s );

    /// @}

    // -------------------- Unsigned pre-cell geometry services --------------------
    /** @name Unsigned pre-cell geometry services
     * @{
     */
  public:

    /** @brief Return the same element as [p] except for the incremented
     * coordinate [k].

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for the incremented
     * coordinate [k].
     */
    static Cell uGetIncr( Cell p, Dimension k );

    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @param k the tested coordinate.
     * @return always false.
     */
    static bool uIsMax( const Cell & p, Dimension k );


    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     *  @param p any pre-cell.
     *  @param k the tested coordinate.
     *  @return always true.
     */
    static bool uIsInside( const Cell & p, Dimension k );


    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @return always true.
     */
    static bool uIsInside( const Cell & p );

    /** @brief Return the same element as [p] except for an decremented
     * coordinate [k].

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for an decremented
     * coordinate [k].
     */
    static Cell uGetDecr( Cell p, Dimension k );

    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @param k the tested coordinate.
     * @return always false.
     */
    static bool uIsMin( const Cell & p, Dimension k );

    /** @brief Return the same element as [p] except for a coordinate [k]
     * incremented with x.

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the increment.
     * @return the same element as [p] except for a coordinate [k]
     * incremented with x.
     */
    static Cell uGetAdd( Cell p, Dimension k, Integer x );

    /** @brief Return the same element as [p] except for a coordinate [k]
     * decremented with x.

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the decrement.
     * @return the same element as [p] except for a coordinate [k]
     * decremented with x.
     */
    static Cell uGetSub( Cell p, Dimension k, Integer x );

    /** @brief Add the vector [vec] to [p].
     *
     * @param p any pre-cell.
     * @param vec any pointel.
     * @return the unsigned copy of the pre-cell [p] translated by [coord].
     */
    static Cell uTranslation( Cell p, const Vector & vec );

    /** @brief Return the projection of [p] along the [k]th direction toward
     *  [bound].
     *
     * Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @return the projection.
     * @pre  `uIsOpen(p, k) == uIsOpen(bound, k)`
     * @post `uTopology(p) == uTopology(uProjection(p, bound, k))`.
     */
    static Cell uProjection( Cell p, const Cell & bound, Dimension k );

    /** @brief Projects [p] along the [k]th direction toward [bound].
     *
     * Otherwise said, p[ k ] == bound[ k ] afterwards
     *
     * @param [in,out] p any pre-cell.
     * @param [in] bound the element acting as bound (same topology as p).
     * @param [in] k the concerned coordinate.
     * @pre  `uIsOpen(p, k) == uIsOpen(bound, k)`
     */
    static void uProject( Cell & p, const Cell & bound, Dimension k );

    /** @brief Increment the pre-cell [p] to its next position (as classically done in
     *  a scanning).
     *
     * Example:
     *
     * \code
     * Cell first, last; // lower and upper bounds
     * Cell p = first;
     * do
     * { // ... whatever [p] is the current cell
     * }
     * while ( KPreSpace::uNext( p, first, last ) );
     * \endcode
     *
     * @param p any pre-cell.
     * @param lower the lower bound.
     * @param upper the upper bound.
     * @return true if p is still within the bounds, false if the
     * scanning is finished.
     * @pre `uTopology(p) == uTopology(lower) == uTopology(upper)`.
     */
    static bool uNext( Cell & p, const Cell & lower, const Cell & upper );

    /// @}

    // -------------------- Signed pre-cell geometry services --------------------
    /** @name Signed pre-cell geometry services
     * @{
     */
  public:

    /** @brief Return the same element as [p] except for the incremented
     * coordinate [k].

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for the incremented
     * coordinate [k].
     */
    static SCell sGetIncr( SCell p, Dimension k );

    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @param k the tested coordinate.
     * @return always false.
     */
    static bool sIsMax( const SCell & p, Dimension k );


    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     *  @param p any pre-cell.
     *  @param k the tested coordinate.
     *  @return always true.
     */
    static bool sIsInside( const SCell & p, Dimension k );


    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @return always true.
     */
    static bool sIsInside( const SCell & p );

    /** @brief Return the same element as [p] except for an decremented
     * coordinate [k].

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @return the same element as [p] except for an decremented
     * coordinate [k].
     */
    static SCell sGetDecr( SCell p, Dimension k );

    /** @brief Useful to check if you are going out of the space.
     *
     * Only for compatibility reasons.
     *
     * @param p any pre-cell.
     * @param k the tested coordinate.
     * @return always false.
     */
    static bool sIsMin( const SCell & p, Dimension k );

    /** @brief Return the same element as [p] except for a coordinate [k]
     * incremented with x.

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the increment.
     * @return the same element as [p] except for a coordinate [k]
     * incremented with x.
     */
    static SCell sGetAdd( SCell p, Dimension k, Integer x );

    /** @brief Return the same element as [p] except for a coordinate [k]
     * decremented with x.

     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param x the decrement.
     * @return the same element as [p] except for a coordinate [k]
     * decremented with x.
     */
    static SCell sGetSub( SCell p, Dimension k, Integer x );

    /** @brief Add the vector [vec] to [p].
     *
     * @param p any pre-cell.
     * @param vec any pointel.
     * @return the signed code of the cell [p] translated by [coord].
     */
    static SCell sTranslation( SCell p, const Vector & vec );

    /** @brief Return the projection of [p] along the [k]th direction toward
     *  [bound].
     *
     *  Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @return the projection.
     * @pre  `sIsOpen(p, k) == sIsOpen(bound, k)`
     * @post `sTopology(p) == sTopology(sProjection(p, bound, k))`.
     */
    static SCell sProjection( SCell p, const SCell & bound, Dimension k );

    /** @brief Projects [p] along the [k]th direction toward [bound].
     *
     *  Otherwise said, p[ k ] == bound[ k ] afterwards.
     *
     * @param p any pre-cell.
     * @param bound the element acting as bound (same topology as p).
     * @param k the concerned coordinate.
     * @pre  `sIsOpen(p, k) == sIsOpen(bound, k)`
     */
    static void sProject( SCell & p, const SCell & bound, Dimension k );

    /** @brief Increment the pre-cell [p] to its next position (as classically done in
     *  a scanning).
     *
     * Example:
     *
     * \code
     * Cell first, last; // lower and upper bounds
     * Cell p = first;
     * do
     * { // ... whatever [p] is the current cell
     * }
     * while ( KPreSpace::uNext( p, first, last ) );
     * \endcode
     *
     * @param p any pre-cell.
     * @param lower the lower bound.
     * @param upper the upper bound.
     * @return true if p is still within the bounds, false if the
     * scanning is finished.
     * @pre `sTopology(p) == sTopology(lower) == sTopology(upper)`.
     */
    static bool sNext( SCell & p, const SCell & lower, const SCell & upper );

    /// @}

    // ----------------------- Neighborhood services --------------------------
    /** @name Neighborhood services
     * @{
     */
  public:

    /** @brief Computes the 1-neighborhood of the pre-cell [c] and returns it.
     *
     * It is the set of cells with same topology that are adjacent
     * to [c].
     *
     * @param cell the unsigned pre-cell of interest.
     * @return the pre-cells of the 1-neighborhood of [cell].
     */
    static Cells uNeighborhood( const Cell & cell );

    /** @brief Computes the 1-neighborhood of the pre-cell [c] and returns it.
     *
     * It is the set of pre-cells with same topology that are adjacent
     * to [c].
     *
     * @param cell the signed pre-cell of interest.
     * @return the pre-cells of the 1-neighborhood of [cell].
     */
    static SCells sNeighborhood( const SCell & cell );

    /** @brief Computes the proper 1-neighborhood of the pre-cell [c] and
     *  returns it.
     *
     * It is the set of pre-cells with same topology that are adjacent
     * to [c] and different from [c].
     *
     * @param cell the unsigned pre-cell of interest.
     * @return the pre-cells of the proper 1-neighborhood of [cell].
     */
    static Cells uProperNeighborhood( const Cell & cell );

    /** @brief Computes the proper 1-neighborhood of the pre-cell [c] and
     *  returns it.
     *
     * It is the set of pre-cells with same topology that are adjacent
     * to [c] and different from [c].
     *
     * @param cell the signed pre-cell of interest.
     * @return the pre-cells of the proper 1-neighborhood of [cell].
     */
    static SCells sProperNeighborhood( const SCell & cell );

    /** @brief Return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     *
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     * @note It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.
     */
    static Cell uAdjacent( const Cell & p, Dimension k, bool up );

    /** @brief Return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     *
     * @param p any pre-cell.
     * @param k the coordinate that is changed.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the adjacent element to [p] along axis [k] in the given
     * direction and orientation.
     * @note It is an alias to 'up ? sGetIncr( p, k ) : sGetDecr( p, k )'.
     */
    static SCell sAdjacent( const SCell & p, Dimension k, bool up );

    /// @}

    // ----------------------- Incidence services --------------------------
    /** @name Incidence services
     * @{
     */
  public:

    /** @brief Return the forward or backward unsigned pre-cell incident to [c]
     * along axis [k], depending on [up].
     * @param c any unsigned pre-cell.
     * @param k any coordinate.
     * @param up if 'true' the orientation is forward along axis
     * [k], otherwise backward.
     * @return the forward or backward unsigned pre-cell incident to [c]
     * along axis [k], depending on [up].
     * @note It may be a lower incident pre-cell if [c] is open along axis
     * [k], else an upper incident pre-cell.
     */
    static Cell uIncident( Cell c, Dimension k, bool up );

    /** @brief Return the forward or backward signed pre-cell incident to [c]
     * along axis [k], depending on [up].
     *
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
    static SCell sIncident( SCell c, Dimension k, bool up );

    /** @brief Return the pre-cells directly low incident to c.
     *
     * @param c any unsigned pre-cell.
     * @return the pre-cells directly low incident to c.
     */
    static Cells uLowerIncident( const Cell & c );

    /** @brief Return the pre-cells directly up incident to c.
     *
     * @param c any unsigned pre-cell.
     * @return the pre-cells directly up incident to c.
     */
    static Cells uUpperIncident( const Cell & c );

    /** @brief Return the signed pre-cells directly low incident to c.
     *
     *  @param c any signed pre-cell.
     *  @return the signed pre-cells directly low incident to c.
     *  @note it is the lower boundary of c expressed as a list of signed pre-cells.
     */
    static SCells sLowerIncident( const SCell & c );

    /** @brief Return the signed pre-cells directly up incident to c.
     *
     * @param c any signed pre-cell.
     * @return the signed pre-cells directly up incident to c.
     * @note it is the upper boundary of c expressed as a list of signed pre-cells.
     */
    static SCells sUpperIncident( const SCell & c );

    /** @brief Return the proper faces of [c] (chain of lower incidence).
     *
     * @param c any unsigned pre-cell.
     * @return the proper faces of [c] (chain of lower incidence).
     */
    static Cells uFaces( const Cell & c );

    /** @brief Return the proper cofaces of [c] (chain of upper incidence).
     *
     * @param c any unsigned ipre-cell.
     * @return the proper cofaces of [c] (chain of upper incidence).
     */
    static Cells uCoFaces( const Cell & c );

    /** @brief Return 'true' if the direct orientation of [p] along [k] is in
     *  the positive coordinate direction.
     *
     *  The direct orientation in a
     *  direction allows to go from positive incident pre-cells to positive
     *  incident pre-cells.  This means that
     *  @code
     *  KPreSpace::sSign( KPreSpace::sIncident( p, k, KPreSpace::sDirect( p, k ) ) ) == KPreSpace::POS
     *  @endcode
     *  is always true.
     *
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the direct orientation of [p] along [k] (true is
     * upward, false is backward).
     */
    static bool sDirect( const SCell & p, Dimension k );

    /** @brief Return the direct incident pre-cell of [p] along [k] (the incident
     * pre-cell along [k])
     *
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the direct incident pre-cell of [p] along [k] (the incident
     * pre-cell along [k])
     */
    static SCell sDirectIncident( SCell p, Dimension k );

    /** @brief Return the indirect incident pre-cell of [p] along [k] (the incident
     * cell along [k] whose sign is negative).
     *
     * @param p any signed pre-cell.
     * @param k any coordinate.
     * @return the indirect incident pre-cell of [p] along [k] (the incident
     * cell along [k] whose sign is negative).
    */
    static SCell sIndirectIncident( SCell p, Dimension k );

    /// @}


    // ------------------------- Internals ------------------------------------
    /** @name Internals
     * @{
     */
  private:
    /** @brief
     * Used by uFaces for computing incident faces.
     */
    static void uAddFaces( Cells & faces, const Cell & c, Dimension axis );

    /** @brief
     * Used by uCoFaces for computing incident cofaces.
     */
    static void uAddCoFaces( Cells & cofaces, const Cell & c, Dimension axis );

    /// @}

    // ----------------------- Interface --------------------------------------
    /** @name DGtal interface
     * @{
     */
  public:

    /** @brief Writes/Displays the object on an output stream.
     *
     * @param out the output stream where the object is written.
     */
    static void selfDisplay ( std::ostream & out );

    /** @brief Checks the validity/consistency of the object.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    static constexpr bool isValid();

    /// @}

  }; // end of class KhalimskyPreSpaceND

  /** @brief
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
