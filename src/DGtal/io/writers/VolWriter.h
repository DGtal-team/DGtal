#pragma once

/**
 * @file VolWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/22
 *
 * Header file for module VolWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VolWriter_RECURSES)
#error Recursive header files inclusion detected in VolWriter.h
#else // defined(VolWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VolWriter_RECURSES

#if !defined VolWriter_h
/** Prevents repeated inclusion of headers. */
#define VolWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class VolWriter
  /**
   * Description of template struct 'VolWriter' <p>
   * \brief Aim: Export a 3D Image using the Vol formats.
   *
   * @tparam TImage the Image type.
   * @tparam TColormap the type of the colormap to use in the export.
   */
  template <typename TImage, typename TColormap>
  struct VolWriter
  {
    // ----------------------- Standard services ------------------------------

    BOOST_STATIC_ASSERT(TImage::Domain::staticDimension == 3);

    BOOST_STATIC_ASSERT((boost::is_same< typename TColormap::ValueType, 
			 typename TImage::ValueType>::value));
    
    typedef TImage Image;
    typedef typename TImage::ValueType ValueType;
    typedef TColormap Colormap;

    /** 
     * Export an Image with the Vol format.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    static bool exportVol(const std::string & filename, const Image &aImage, 
			  const ValueType & minV, const ValueType & maxV);
    
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers//VolWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VolWriter_h

#undef VolWriter_RECURSES
#endif // else defined(VolWriter_RECURSES)
