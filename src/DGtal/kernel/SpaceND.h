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
 * @file SpaceND.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/14
 *
 * Header file for module SpaceND.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SpaceND_RECURSES)
#error Recursive header files inclusion detected in SpaceND.h
#else // defined(SpaceND_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SpaceND_RECURSES

#if !defined SpaceND_h
/** Prevents repeated inclusion of headers. */
#define SpaceND_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CUnsignedInteger.h"
#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class SpaceND
  /**
   * Description of class 'SpaceND' <p>
   *
   * Aim: SpaceND defines the fundamental structure of a Digital Space in ND.
   *
   * This class just defines fundamental types associated to a digital
   * space in dimension n. For instance, it specifies the type of a
   * point, the type of a vector or the type of subspace.
   * 
   * Code snippet:
   *@code

#include <DGtal/kernel/SpaceND.h>

//...

//We define the type of a digital domain on dimension 4 using the
//"int" arithmetic ring.

typedef SpaceND<4, int> Space4Int;
   
//We get the associated type to represent a point in this space
typedef Space4::Point Point4Int;

//and we use it (see PointVector documentation).
Point4Int a= {2, 3 , -5 , 6};
   @endcode
   *
   **/

  template < DGtal::uint32_t dim,
	     typename Integer = DGtal::int32_t >
  class SpaceND
  {
  public:

    BOOST_CONCEPT_ASSERT(( CInteger<Integer> ) );
    BOOST_CONCEPT_ASSERT(( CSignedInteger<Integer> ) );

    ///Type used to represent sizes in the digital space.
    typedef typename IntegerTraits<Integer>::UnsignedVersion Size;
    
    ///Type used to handle the space dimension.
    typedef DGtal::uint32_t Dimension;

    ///Points in DGtal::SpaceND.
    typedef PointVector<dim,Integer> Point;

    ///Vectors in DGtal::SpaceND.
    typedef PointVector<dim,Integer> Vector;

    ///Type to denote the space itself.
    typedef SpaceND<dim, Integer> Space;

    ///static constants to store the dimension.
    static const Dimension staticDimension = dim;

    ///Define the type of a sub co-Space
    template <DGtal::uint32_t codimension>
    struct Subcospace
    {
      typedef SpaceND < dim - codimension, Integer > Type;
    };
   
    ///Define the type of a subspace.
    template <DGtal::uint32_t subdimension>
    struct Subspace
    {
      typedef SpaceND<subdimension, Integer> Type;
    };


    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     *
     */
    SpaceND() {};

    /**
     * Destructor.
     */
    ~SpaceND() {};

    /**
     * @return the digital space of specified subdimension of this space.
     */
    template <DGtal::uint32_t subdimension>
    static
    typename Subspace<subdimension>::Type subspace()
    {
      ASSERT( subdimension <= dim );
      return SpaceND<subdimension, Integer>();
    }


    /**
     * @return the digital space of specified codimension of this space.
     */
    template <DGtal::uint32_t codimension>
    static
    typename Subcospace<codimension>::Type subcospace()
    {
      ASSERT( codimension <= dim );
      return SpaceND < dim - codimension, Integer > ();
    }

    /**
     * @return the dimension of the digital space.
     */
    static Dimension dimension()
    {
      return staticDimension;
    }

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    static void selfDisplay( std::ostream & out )
    {
      out << "[SpaceND dim=" << dimension() << " size_elem=" << sizeof( Integer ) << " ]";
    }

  private:
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    SpaceND & operator=( const SpaceND & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class SpaceND


  /**
   * Overloads 'operator<<' for displaying objects of class 'SpaceND'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SpaceND' to write.
   * @return the output stream after the writing.
   */
  template <std::size_t dim, typename Integer, typename Size, typename Dimension>
  static std::ostream&
  operator<<( std::ostream & out, const SpaceND<dim, Integer, Size, Dimension> & object )
  {
    object.selfDisplay( out );
    return out;
  }


} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SpaceND_h

#undef SpaceND_RECURSES
#endif // else defined(SpaceND_RECURSES)
