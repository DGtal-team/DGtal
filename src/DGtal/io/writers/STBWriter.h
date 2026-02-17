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
 * @file STBWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/14
 *
 * Header file for module STBWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(STBWriter_RECURSES)
#error Recursive header files inclusion detected in STBWriter.h
#else // defined(STBWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define STBWriter_RECURSES

#if !defined STBWriter_h
/** Prevents repeated inclusion of headers. */
#define STBWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/colormaps/BasicColorToScalarFunctors.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class STBWriter
  /**
   * Description of template class 'STBWriter' <p>
   * \brief Aim: Image Writer using the `stb_image.h` header only code.
   *
   * These methods export an image in various formats (png, jpg, tga, bmp) by
   * first using the functor to convert the image values to colors.
   *
   * @tparam TImageContainer the image type
   * @tparam TFunctor the functor type that maps the image values to DGtal::Color
   */
  template <typename TImageContainer,typename TFunctor=functors::Identity>
  class STBWriter
  {
    //  ----------------------- Standard services ------------------------------
  public:
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain::Vector Vector;
    typedef typename TImageContainer::Value Value;
    typedef TFunctor Functor ;

    BOOST_STATIC_ASSERT( (ImageContainer::Domain::dimension == 2));
    BOOST_CONCEPT_ASSERT((concepts::CUnaryFunctor<TFunctor,  Value, DGtal::Color > )) ;

    /**
     * Export an image as PNG
     *
     * @param filename the file name to export.
     * @param anImage the image to export.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (Value -> DGtal::Color functor).
     * @return an instance of the ImageContainer.
     */
    static bool exportPNG(const std::string & filename,
                       const ImageContainer& anImage,
                       const Functor & aFunctor =  Functor());

    /**
     * Export an image as TGA
     *
     * @param filename the file name to export.
     * @param anImage the image to export.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (Value -> DGtal::Color functor).
     * @return an instance of the ImageContainer.
     */
    static bool exportTGA(const std::string & filename,
                          const ImageContainer& anImage,
                          const Functor & aFunctor =  Functor());

    /**
     * Export an image as BMP
     *
     * @param filename the file name to export.
     * @param anImage the image to export.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (Value -> DGtal::Color functor).
     * @return an instance of the ImageContainer.
     */
    static bool exportBMP(const std::string & filename,
                          const ImageContainer& anImage,
                          const Functor & aFunctor =  Functor());

    /**
     * Export an image as JPG
     *
     * @param filename the file name to export.
     * @param anImage the image to export.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (Value -> DGtal::Color functor).
     * @param quality the jpg compression quality (in [0,100], 100 is full quality, def = 70)
     * @return an instance of the ImageContainer.
     */
    static bool exportJPG(const std::string & filename,
                          const ImageContainer& anImage,
                          const Functor & aFunctor =  Functor(),
                          int quality = 70);

  }; // end of class STBWriter


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/STBWriter.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
#endif // !defined STBWriter_h

#undef STBWriter_RECURSES
#endif // else defined(STBWriter_RECURSES)
