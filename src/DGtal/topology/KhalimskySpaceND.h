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
 * @file KhalimskySpaceND.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/02/08
 *
 * Header file for module KhalimskySpaceND.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(KhalimskySpaceND_RECURSES)
#error Recursive header files inclusion detected in KhalimskySpaceND.h
#else // defined(KhalimskySpaceND_RECURSES)
/** Prevents recursive inclusion of headers. */
#define KhalimskySpaceND_RECURSES

#if !defined KhalimskySpaceND_h
/** Prevents repeated inclusion of headers. */
#define KhalimskySpaceND_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CUnsignedInteger.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SpaceND.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
     Represents an (unsigned) cell in a cellular grid space by its
     Khalimsky coordinates.
   */
  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t >
  struct KhalimskyCell
  {
  public:
    typedef TInteger Integer;
    typedef typename IntegerTraits<Integer>::UnsignedVersion UnsignedInteger;
    typedef PointVector< dim, Integer > Point;

    PointVector< dim, TInteger > myCoordinates;

    /**
     * Constructor.
     */
    KhalimskyCell();
    
    /**
     * Copy constructor.
     *
     * @param other any other cell.
     */
    KhalimskyCell( const KhalimskyCell & other );

    /**
     * constructor from point.
     *
     * @param other any point.
     */
    KhalimskyCell( const Point & point );

    /**
     * Copy constructor.
     *
     * @param other any other cell.
     */
    KhalimskyCell & operator=( const KhalimskyCell & other );

  }; 

  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t >
  std::ostream & 
  operator<<( std::ostream & out, 
	      const KhalimskyCell< dim, TInteger > & object );

  /**
     Represents a signed cell in a cellular grid space by its
     Khalimsky coordinates and a boolean value.
   */
  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t >
  struct SignedKhalimskyCell
  {
  public:
    typedef TInteger Integer;
    typedef typename IntegerTraits<Integer>::UnsignedVersion UnsignedInteger;
    typedef PointVector< dim, Integer > Point;

    PointVector< dim, TInteger > myCoordinates;
    bool myPositive;

    /**
     * Constructor.
     */
    SignedKhalimskyCell();
    
    /**
     * Copy constructor.
     *
     * @param other any other cell.
     */
    SignedKhalimskyCell( const SignedKhalimskyCell & other );

    /**
     * constructor from point.
     *
     * @param other any point.
     * @param 'true' if cell has positive sign.
     */
    SignedKhalimskyCell( const Point & point, bool positive );

    /**
     * Copy constructor.
     *
     * @param other any other cell.
     */
    SignedKhalimskyCell & operator=( const SignedKhalimskyCell & other );

  }; 

  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t >
  std::ostream & 
  operator<<( std::ostream & out, 
	      const SignedKhalimskyCell< dim, TInteger > & object );

  /**
     This class is useful for looping on all "interesting" coordinates of a
     cell. For instance, surfels in Z3 have two interesting coordinates (the
     ones spanned by the surfel).
     <pre>
     KSpace::Cell p;
     KnSpace::DirIterator q;
     for ( q = ks.uDirs( p ); q != 0; ++q ) ...
     </pre>
   */
  template < typename TInteger = DGtal::int32_t,
	     typename TDimension = DGtal::uint32_t >
  class CellDirectionIterator 
  {
  public:
    typedef TInteger Integer;
    typedef TDimension Dimension;

  public:
    /**
     * Constructor from directions of cell.
     * @param dirs the directions of a cell.
     */
    CellDirectionIterator( Integer dirs ) 
      : m_k( 0 ), m_dirs( dirs )
    {
      if ( m_dirs ) find();
    }

    /**
     * @return the current direction.
     */
    Dimension operator*() const
    {
      return m_k;
    }

    /**
     * @return the current coded directions.
     */
    Integer codedDirs() const
    {
      return m_dirs;
    }

    /**
     * Pre-increment. Go to next direction.
     */
    CellDirectionIterator & operator++()
    {
      m_dirs >>= 1;
      ++m_k;
      if ( m_dirs ) find();
      return *this;
    }
    
    /** 
     * Fast comparison with unsigned integer. Comparison == with 0 is 'true' at
     * the end of the iteration.
     * @param coded_dirs any coded directions.
     * @return 'true' if the iterator dirs are different from the coded dirs.
     */
    bool operator!=( Integer coded_dirs ) const
    {
      return m_dirs != coded_dirs;
    }

    /** 
     * @return 'true' if the iteration is ended.
     */
    bool end() const
    {
      return m_dirs == 0;
    }

    /** 
     * Slow comparison with other iterator. Useful to check for end of loop.
     * @param other any direction iterator.
     */
    bool operator!=( const CellDirectionIterator & other ) const
    {
      return ( m_dirs != other.m_dirs )
	|| ( m_k != other.m_k );
    }

    /** 
     * Slow comparison with other iterator.
     * @param other any direction iterator.
     */
    bool operator==( const CellDirectionIterator & other ) const
    {
      return ( m_dirs == other.m_dirs )
	&& ( m_k == other.m_k );
    }
    
  private:
    /**
     * the current direction.
     */
    Dimension m_k;

    /**
     * the directions to iterate (topology word).
     */
    Integer m_dirs;

  private:
    /**
     * Look for valid coordinate (m_dirs must be != 0).
     */
    void find()
    {
      while ( ( m_dirs & 1 ) == 0 )
	{
	  m_dirs >>= 1;
	  ++m_k;
	}
    }
  };


  /////////////////////////////////////////////////////////////////////////////
  // template class KhalimskySpaceND
  /**
   * Description of template class 'KhalimskySpaceND' <p> \brief Aim:
   * This class is a model of CCellularGridSpaceND. It represents the
   * cubical grid as a cell complex, whose cells are defined as an
   * array of integers. The topology of the cells is defined by the
   * parity of the coordinates (even: closed, odd: open). 
   *
   * The space is generally finite (except for arbitrary size
   * integers). The user should choose between a closed (default) cell
   * space or an open cell space.
   *
   * @tparam dim the dimension of the digital space.
   * @tparam TInteger the Integer class used to specify the arithmetic computations (default type = int).
   * @tparam TSize the Integer class used to represent the sizes in the space (default type = unsigned int).
   * @tparam TDimension the type used to represent indices of coordinates.
   */
  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t,
	     typename TSize = DGtal::uint32_t,
	     typename TDimension = DGtal::uint32_t >
  class KhalimskySpaceND
  {
    /// \todo fixer des concept check sur Integer
    BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
    BOOST_CONCEPT_ASSERT(( CUnsignedInteger<TSize> ) );
    BOOST_CONCEPT_ASSERT(( CUnsignedInteger<TDimension> ) );

    // ----------------------- Public types ------------------------------
  public:
    //Arithmetic
    typedef TInteger Integer;
    typedef typename IntegerTraits<Integer>::UnsignedVersion UnsignedInteger;

    //Size & Dimension
    typedef TSize Size;
    typedef TDimension Dimension;

    // Cells
    typedef KhalimskyCell< dim, Integer > Cell;
    typedef SignedKhalimskyCell< dim, Integer > SCell;
    typedef bool Sign;
    typedef CellDirectionIterator< Integer, Dimension > DirIterator;    
    
    //Points and Vectors
    typedef PointVector< dim, Integer > Point;
    typedef PointVector< dim, Integer > Vector;
    
    typedef SpaceND<dim, Integer, Size> Space;
    typedef KhalimskySpaceND<dim, Integer, Size, Dimension> KhalimskySpace;

    // static constants
    static const Dimension staticDimension = dim;
    static const Dimension DIM = dim;
    static const Sign POS = true;
    static const Sign NEG = false;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~KhalimskySpaceND();

    /**
     * Constructor.
     */
    KhalimskySpaceND();

    /**
     * Specifies the upper and lower bounds for the maximal cells in
     * this space.
     *
     * @param lower_included the lowest point in this space (digital coords)
     * @param upper_excluded the upper point in this space (digital coords) + (1,...,1)
     * @param closed 'true' if this space is closed, 'false' if open.
     *
     * @return true if the initialization was valid (ie, such bounds
     * are representable with these integers).
     */
    bool init( const Point & lower_included,
	       const Point & upper_excluded,
	       bool closed );

    // ------------------------- Basic services ------------------------------
  public:
    /**
     * @return the dimension of the space.
     */
    Dimension dimension() const;

    /**
     * @param k a coordinate (from 0 to 'dim()-1').
     * @return the width of the space in the [k]-dimension.
     */
    Size size( Dimension k ) const;

    // ----------------------- Cell creation services --------------------------
  public:

    /**
     * From the Khalimsky coordinates of a cell, builds the
     * corresponding unsigned cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @return the unsigned cell.
     */
    Cell uCell( const Point & kp ) const;

    /**
     * From the digital coordinates of a point in Zn and a cell type,
     * builds the corresponding cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology.
     *
     * @return the cell having the topology of [c] and the given
     * digital coordinates [p].
     */
    Cell uCell( const Point & p, const Cell & c ) const;

    /**
     * From the Khalimsky coordinates of a cell and a sign, builds the
     * corresponding unsigned cell.
     *
     * @param kp an integer point (Khalimsky coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     * @return the unsigned cell.
     */
    SCell sCell( const Point & kp, Sign sign = POS ) const;

    /**
     * From the digital coordinates of a point in Zn and a signed cell
     * type, builds the corresponding signed cell.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param c another cell defining the topology and sign.
     *
     * @return the cell having the topology and sign of [c] and the given
     * digital coordinates [p].
     */
    SCell sCell( const Point & p, const SCell & c ) const;

   /**
     * From the digital coordinates of a point in Zn, creates the spel
     * (cell of maximal dimension) with these coordinates.
     *
     * @param p an integer point (digital coordinates of cell).
     *
     * @return the spel having the given digital coordinates [p].
     */
    Cell uSpel( const Point & p ) const;

   /**
     * From the digital coordinates of a point in Zn, creates the spel
     * (cell of maximal dimension) with these coordinates.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     *
     * @return the signed spel having the given digital coordinates [p].
     */
    SCell sSpel( const Point & p, Sign sign = POS ) const;

   /**
     * From the digital coordinates of a point in Zn, creates the pointel
     * (cell of dimension 0) with these coordinates.
     *
     * @param p an integer point (digital coordinates of cell).
     *
     * @return the pointel having the given digital coordinates [p].
     */
    Cell uPointel( const Point & p ) const;

   /**
     * From the digital coordinates of a point in Zn, creates the pointel
     * (cell of dimension 0) with these coordinates.
     *
     * @param p an integer point (digital coordinates of cell).
     * @param sign the sign of the cell (either POS or NEG).
     *
     * @return the signed pointel having the given digital coordinates [p].
     */
    SCell sPointel( const Point & p, Sign sign = POS ) const;


    // ----------------------- Read accessors to cells ------------------------
  public:
    /**
     * @param c any unsigned cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    Integer uKCoord( const Cell & c, Dimension k ) const;

    /**
     * @param c any unsigned cell.
     * @param k any valid dimension.
     * @return its digital coordinate  along [k].
     */
    Integer uCoord( const Cell & c, Dimension k ) const;

    /**
     * @param c any unsigned cell.
     * @return its Khalimsky coordinates.
     */
    Point uKCoords( const Cell & c ) const;

    /**
     * @param c any unsigned cell.
     * @return its digital coordinates.
     */
    Point uCoords( const Cell & c ) const;

    /**
     * @param c any signed cell.
     * @param k any valid dimension.
     * @return its Khalimsky coordinate along [k].
     */
    Integer sKCoord( const SCell & c, Dimension k ) const;

    /**
     * @param c any signed cell.
     * @param k any valid dimension.
     * @return its digital coordinate  along [k].
     */
    Integer sCoord( const SCell & c, Dimension k ) const;

    /**
     * @param c any signed cell.
     * @return its Khalimsky coordinates.
     */
    Point sKCoords( const SCell & c ) const;

    /**
     * @param c any signed cell.
     * @return its digital coordinates.
     */
    Point sCoords( const SCell & c ) const;

    /**
     * @param c any signed cell.
     * @return its sign.
     */
    Sign sSign( const SCell & c ) const;

    // ----------------------- Write accessors to cells ------------------------
  public:

    /**
     * Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any unsigned cell.
     * @param k any valid dimension.
     * @param i an integer coordinate within the space.
     */
    void usetKCoord( Cell & c, Dimension k, const Integer & i ) const;

    /**
     * Sets the [k]-th Khalimsky coordinate of [c] to [i].
     * @param c any signed cell.
     * @param k any valid dimension.
     * @param i an integer coordinate within the space.
     */
    void ssetKCoord( SCell & c, Dimension k, const Integer & i ) const;

    /**
     * Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any unsigned cell.
     * @param k any valid dimension.
     * @param i an integer coordinate within the space.
     */
    void usetCoord( Cell & c, Dimension k, const Integer & i ) const;

    /**
     * Sets the [k]-th digital coordinate of [c] to [i].
     * @param c any signed cell.
     * @param k any valid dimension.
     * @param i an integer coordinate within the space.
     */
    void ssetCoord( SCell & c, Dimension k, const Integer & i ) const;

    /**
     * Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any unsigned cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    void usetKCoords( Cell & c, const Point & kp ) const;

    /**
     * Sets the Khalimsky coordinates of [c] to [kp].
     * @param c any signed cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    void ssetKCoords( SCell & c, const Point & kp ) const;

    /**
     * Sets the digital coordinates of [c] to [kp].
     * @param c any unsigned cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    void usetCoords( Cell & c, const Point & kp ) const;

    /**
     * Sets the digital coordinates of [c] to [kp].
     * @param c any signed cell.
     * @param kp the new Khalimsky coordinates for [c].
     */
    void ssetCoords( SCell & c, const Point & kp ) const;

    /**
     * Sets the sign of the cell.
     * @param c (modified) any signed cell.
     * @param s any sign.
     */
    void ssetSign( SCell & c, Sign s ) const;

    // -------------------- Conversion signed/unsigned ------------------------
  public:
    /**
     * Creates a signed cell from an unsigned one and a given sign.
     * @param p any unsigned cell.
     * @param s a sign.
     * @return the signed version of the cell [p] with sign [s].
     */
    SCell signs( const Cell & p, Sign s ) const;

    /**
     * Creates an unsigned cell from a signed one.
     * @param p any signed cell.
     * @return the unsigned version of the cell [p].
     */
    Cell unsigns( const SCell & p ) const;

    /**
     * Creates the signed cell with the inverse sign of [p].
     * @param p any signed cell.
     * @return the cell [p] with opposite sign.
     */
    SCell sopp( const SCell & p ) const;

    // ------------------------- Cell topology services -----------------------
  public:
    /**
     * @param p any unsigned cell.
     * @return the topology word of [p].
     */
    Integer utopology( const Cell & p ) const;

    /**
     * @param p any signed cell.
     * @return the topology word of [p].
     */
    Integer stopology( const SCell & p ) const;

    /**
     * @param p any unsigned cell.
     * @return the dimension of the cell [p].
     */
    Dimension udim( const Cell & p ) const;

    /**
     * @param p any signed cell.
     * @return the dimension of the cell [p].
     */
    Dimension sdim( const SCell & p ) const;

    /**
     * @param b any unsigned cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    bool uisSurfel( const Cell & b ) const;

    /**
     * @param b any signed cell.
     * @return 'true' if [b] is a surfel (spans all but one coordinate).
     */
    bool sisSurfel( const SCell & b ) const;
  



    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    Point myLowerIncluded;
    Point myUpperExcluded;
    Point myKLowerIncluded;
    Point myKUpperExcluded;
    bool myIsClosed;
    // ------------------------- Hidden services ------------------------------
  protected:


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    KhalimskySpaceND ( const KhalimskySpaceND & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    KhalimskySpaceND & operator= ( const KhalimskySpaceND & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class KhalimskySpaceND


  /**
   * Overloads 'operator<<' for displaying objects of class 'KhalimskySpaceND'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'KhalimskySpaceND' to write.
   * @return the output stream after the writing.
   */
  template < std::size_t dim,
	     typename TInteger = DGtal::int32_t,
	     typename TSize = DGtal::uint32_t,
	     typename TDimension = DGtal::uint32_t >
  std::ostream&
  operator<< ( std::ostream & out, 
	       const KhalimskySpaceND<dim, TInteger, TSize, TDimension > & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/KhalimskySpaceND.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined KhalimskySpaceND_h

#undef KhalimskySpaceND_RECURSES
#endif // else defined(KhalimskySpaceND_RECURSES)
