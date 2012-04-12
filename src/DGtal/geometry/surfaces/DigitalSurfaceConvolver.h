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
 * @file DigitalSurfaceConvolver.h
 * @brief Move and Compute a convolution kernel along the input shape outline
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/27
 *
 * This file is part of the DGtal library.
 *
 * @see testConvolver.cpp
 */

#if defined(DigitalSurfaceConvolver_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceConvolver.h
#else // defined(DigitalSurfaceConvolver_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceConvolver_RECURSES

#if !defined DigitalSurfaceConvolver_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceConvolver_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class DigitalSurfaceConvolver
  /**
   * Description of class 'DigitalSurfaceConvolver' <p>
   * \brief Aim: Compute a functor at a position of the input shape, and move it along the outline of this one.
   *
   * @tparam TDigitalSurface a model of digital surface.
   * @tparam TCellularKernel a model of a convolution kernel on signed cells
   */
  template <typename TDigitalSurface, typename TCellularKernel>

  class DigitalSurfaceConvolver
  {
      // ----------------------- Types ------------------------------------------

    public:


      ///@todo test CDigitalSurfaceContainer ?

      typedef TDigitalSurface DigitalSurface;
      typedef TCellularKernel CellularKernel;
      typedef typename TDigitalSurface::ConstIterator ConstIterator;
      typedef typename CellularKernel::Quantity Quantity;

      // BOOST_CONCEPT_ASSERT(( CCellularConvolutionKernel::same_type< DigitalSurface::KSpace, CellularKernel::KSpace ))

      // BOOST_CONCEPT_ASSERT(( ConceptUtils::same_type< DigitalSurface::KSpace, CellularKernel::KSpace

      typedef typename TDigitalSurface::KSpace::SCell Spel;

      // ----------------------- Standard services ------------------------------

    public:

      /**
       * Constructor.
       * 
       * @param aDigitalSurface digital surface of the shape
       * @param aKernelFunctor a cell convolution kernel
       */
      DigitalSurfaceConvolver( const DigitalSurface & aDigitalSurface,
                               const CellularKernel & aKernelFunctor );


      /**
       * Destructor.
       */
      ~DigitalSurfaceConvolver() {}

      // ----------------------- Interface --------------------------------------

    public:


      /**
       * @return the estimated quantity at *it
       */
      Quantity eval( const ConstIterator& it );

      /**
       * @return the estimated quantity
       * from itb till ite (exculded)
       */
      template <typename OutputIterator>
      OutputIterator eval( const ConstIterator& itb,
                           const ConstIterator& ite,
                           OutputIterator result );

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const;

      // ------------------------- Private Datas --------------------------------

    private:

      ///Copy to the digital surface
      const DigitalSurface & mySurface;

      ///Copy of the kernel convolution functor
      const CellularKernel & myKernel;

      ///Internal copy of the surface kspace
      typename DigitalSurface::KSpace mySurfaceKSpace;


      // ------------------------- Hidden services ------------------------------

    protected:

      /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
      DigitalSurfaceConvolver();

    private:

      /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
      DigitalSurfaceConvolver( const DigitalSurfaceConvolver & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
      DigitalSurfaceConvolver & operator= ( const DigitalSurfaceConvolver & other );

      // ------------------------- Internals ------------------------------------

    private:

  }; // end of class DigitalSurfaceConvolver


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfaceConvolver'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfaceConvolver' to write.
   * @return the output stream after the writing.
   */
  template <typename TD,  typename K>
  std::ostream&
  operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver<TD, K> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/surfaces/DigitalSurfaceConvolver.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceConvolver_h

#undef DigitalSurfaceConvolver_RECURSES
#endif // else defined(DigitalSurfaceConvolver_RECURSES)
