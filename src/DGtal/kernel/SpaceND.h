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
   * \todo documentation here!
   *
   * @tparam Dimension the dimension of the Digital Space.
   * @tparam Integer the Integer class used to specify the arithmetic (default type = int).
   */

  template < std::size_t dim,
  typename TInteger = DGtal::int32_t,
  typename TSize = DGtal::uint32_t,
  typename TDimension = DGtal::uint32_t >
  class SpaceND
  {
    public:

      /// \todo fixer des concept check sur Integer
      BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
      BOOST_CONCEPT_ASSERT(( CUnsignedInteger<TSize> ) );
      BOOST_CONCEPT_ASSERT(( CUnsignedInteger<TDimension> ) );

      //Arithmetic
      typedef TInteger Integer;
      typedef typename IntegerTraits<Integer>::UnsignedVersion UnsignedInteger;

      //Size & Dimension
      typedef TSize Size;
      typedef TDimension Dimension;

      //Points and Vectors
      typedef PointVector<dim,Integer> Point;
      typedef PointVector<dim,Integer> Vector;

      typedef SpaceND<dim, Integer, Size> Space;

      // static constants
      static const Dimension staticDimension = dim;

      //typedef Matrix<dimT,DimensionT,Integer> Matrix;
      template <std::size_t codimension>
      struct Subcospace
      {
        typedef SpaceND < dim - codimension, Integer, Size, Dimension > Type;
      };
      template <std::size_t subdimension>
      struct Subspace
      {
        typedef SpaceND<subdimension, Integer, Size, Dimension> Type;
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
      template <std::size_t subdimension>
      static
      typename Subspace<subdimension>::Type subspace()
      {
        ASSERT( subdimension <= dim );
        return SpaceND<subdimension, Integer, Size, Dimension>();
      }


      /**
       * @return the digital space of specified codimension of this space.
       */
      template <std::size_t codimension>
      static
      typename Subcospace<codimension>::Type subcospace()
      {
        ASSERT( codimension <= dim );
        return SpaceND < dim - codimension, Integer, Size, Dimension > ();
      }


      /**
       * @return the dimension of the digital space.
       */
      static Dimension dimension()
      {
        return dim;
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
