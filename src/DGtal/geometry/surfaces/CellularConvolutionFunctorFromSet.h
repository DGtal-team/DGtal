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
 * @file CellularConvolutionFunctorFromSet.h
 * @brief Create a cellular convolution functor from a shape set.
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/04/05
 *
 * This file is part of the DGtal library.
 *
 * @see testConvolver.cpp
 */

#if defined(CellularConvolutionFunctorFromSet_RECURSES)
#error Recursive header files inclusion detected in CellularConvolutionFunctorFromSet.h
#else // defined(CellularConvolutionFunctorFromSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CellularConvolutionFunctorFromSet_RECURSES

#if !defined CellularConvolutionFunctorFromSet_h
/** Prevents repeated inclusion of headers. */
#define CellularConvolutionFunctorFromSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CellularConvolutionFunctorFromSet
  /**
   * Description of template class 'CellularConvolutionFunctorFromSet' <p>
   * \brief Aim: a convolution functor that return a value (between constructor's arguments val1 and val2)
   * from a signed cell.
   *
   * @tparam TDigitalSet a model of a digital set
   * @tparam TKSpace a model of a KSpace from digital set
   */
  template <typename TDigitalSet, typename TKSpace>

  class CellularConvolutionFunctorFromSet
  {
      // ----------------------- Standard services ------------------------------

    public:

      typedef TDigitalSet DigitalSet;
      typedef TKSpace KSpace;
      typedef typename KSpace::Space::Integer Quantity;
      typedef Quantity Value;

      /**
      * Constructor.
      *
      * @param aDigitalSet digital set of the shape, used to compute if a signed cell is inside or not.
      * @param val1 value returned by the functor if the signed cell is inside the shape
      * @param val2 value returned by the functor if the signed cell isn't inside the shape.
      */
      CellularConvolutionFunctorFromSet( const DigitalSet & aDigitalSet,
                                         Quantity val1 = NumberTraits<Quantity>::ZERO,
                                         Quantity val2 = NumberTraits<Quantity>::ONE )
          : myDigitalSet( aDigitalSet ),
          myValFalse( val1 ),
          myValTrue( val2 )
      {
        bool space_ok = myKSpace.init( myDigitalSet.domain().lowerBound(), myDigitalSet.domain().upperBound(), true );
        ASSERT( space_ok );
      }


      /**
       * Assumption aCell -> Spel
       *
       * @param aSpel a signed cell witch we compute if she is inside the shape or not.
       * @return myValTrue if she is inside, myValFalse else
       */
      inline
      Quantity operator()( const typename KSpace::SCell &aSpel ) const
      {
        if ( myDigitalSet.find( myKSpace.sCoords( aSpel ) ) != myDigitalSet.end() )
          return myValTrue;
        else
          return myValFalse;
      }



      /**
       * Destructor.
       */
      ~CellularConvolutionFunctorFromSet();

      // ----------------------- Interface --------------------------------------

    public:

      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay( std::ostream & out ) const;

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const;

      // ------------------------- Protected Datas ------------------------------

    private:
      // ------------------------- Private Datas --------------------------------

      ///DigitalSet of the shape
      const  DigitalSet & myDigitalSet;
      ///Value return by the functor if the cellular isn't inside the shape
      Quantity myValFalse;
      ///Value return by the functor if the cellular is inside the shape
      Quantity myValTrue;
      ///KSpace of the shape
      KSpace myKSpace;

      // ------------------------- Hidden services ------------------------------

    protected:

      /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
      CellularConvolutionFunctorFromSet();

    private:

      /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
      CellularConvolutionFunctorFromSet( const CellularConvolutionFunctorFromSet & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
      CellularConvolutionFunctorFromSet & operator= ( const CellularConvolutionFunctorFromSet & other );

      // ------------------------- Internals ------------------------------------

    private:

  }; // end of class CellularConvolutionFunctorFromSet


  /**
   * Overloads 'operator<<' for displaying objects of class 'CellularConvolutionFunctorFromSet'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CellularConvolutionFunctorFromSet' to write.
   * @return the output stream after the writing.
   */
  template <typename T, typename K>
  std::ostream&
  operator<< ( std::ostream & out, const CellularConvolutionFunctorFromSet<T, K> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/CellularConvolutionFunctorFromSet.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CellularConvolutionFunctorFromSet_h

#undef CellularConvolutionFunctorFromSet_RECURSES
#endif // else defined(CellularConvolutionFunctorFromSet_RECURSES)
