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
#include <boost/static_assert.hpp>
#include "Magick++.h"
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

    BOOST_STATIC_ASSERT( (ImageContainer::Domain::staticDimension == 2));

    /** 
     * Main method to import an Image into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param filename the file name to import.
     * @return an instance of the ImageContainer.
     */
    static ImageContainer importImage(const std::string & filename) throw(DGtal::DGtalIOException);
    
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
