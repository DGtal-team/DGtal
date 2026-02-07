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
 * @file STBReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/14
 *
 * Header file for module STBReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(STBReader_RECURSES)
#error Recursive header files inclusion detected in STBReader.h
#else // defined(STBReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define STBReader_RECURSES

#if !defined STBReader_h
/** Prevents repeated inclusion of headers. */
#define STBReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/colormaps/BasicColorToScalarFunctors.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class STBReader
  /**
   * Description of template class 'STBReader' <p>
   * \brief Aim: Image reader using the `stb_image.h` header only code.
   *
   *
   *
   */
  template <typename TImageContainer,typename TFunctor=functors::ColorRGBEncoder<typename TImageContainer::Value> >
  class STBReader
  {
    //  ----------------------- Standard services ------------------------------
  public:
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain::Vector Vector;
    typedef typename TImageContainer::Value Value;
    typedef TFunctor Functor ;

    BOOST_STATIC_ASSERT( (ImageContainer::Domain::dimension == 2));
    BOOST_CONCEPT_ASSERT((concepts::CUnaryFunctor<TFunctor,  DGtal::Color, Value> )) ;

    /**
     * Main method to import an Image into an instance of the
     * template parameter ImageContainer.
     *
     * @param filename the file name to import.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (DGtal::Color -> Value functor).
     * @return an instance of the ImageContainer.
     */
    static ImageContainer import(const std::string & filename,
                                 const Functor & aFunctor =  Functor());
  }; // end of class STBReader



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers//STBReader.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
#endif // !defined STBReader_h

#undef STBReader_RECURSES
#endif // else defined(STBReader_RECURSES)
