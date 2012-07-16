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
 * @file MagickReader.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Header file for module MagickReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MagickReader_RECURSES)
#error Recursive header files inclusion detected in MagickReader.h
#else // defined(MagickReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MagickReader_RECURSES

#if !defined MagickReader_h
/** Prevents repeated inclusion of headers. */
#define MagickReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <cstdio>
#include <Magick++.h>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class MagickReader
  /**
   * Description of template class 'MagickReader' <p>
   * \brief Aim: implements methods to read a 2D image using the ImageMagick library.
   *
   *
   * @tparam TImageContainer the image container to use. 
   *
   */
  template <typename TImageContainer>
  struct MagickReader
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain::Vector Vector;

    BOOST_STATIC_ASSERT( (ImageContainer::Domain::dimension == 2));

    /** 
     * Main method to import an Image into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param filename the file name to import.
     * @param topbotomOrder if true, the point of coordinate (0,0) will be the bottom left corner image point (default) else the center of image coordinate will be the top left of the image (not usual).  
     * @return an instance of the ImageContainer.
     */
    static ImageContainer importImage(const std::string & filename, bool topbotomOrder = true) throw(DGtal::IOException);
    
  }; // end of class MagickReader


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/MagickReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MagickReader_h

#undef MagickReader_RECURSES
#endif // else defined(MagickReader_RECURSES)
