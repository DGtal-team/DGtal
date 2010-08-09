#pragma once

/**
 * @file RawWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/22
 *
 * Header file for module RawWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RawWriter_RECURSES)
#error Recursive header files inclusion detected in RawWriter.h
#else // defined(RawWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RawWriter_RECURSES

#if !defined RawWriter_h
/** Prevents repeated inclusion of headers. */
#define RawWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/io/colormaps/CColorMap.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class RawWriter
  /**
   * Description of template struct 'RawWriter' <p>
   * \brief Aim: Raw binary export of an Image.
   *
   * @tparam TImage the Image type.
   * @tparam TColormap the type of the colormap to use in the export.
   */
  template <typename TImage, typename TColormap>
  struct RawWriter
  {
    // ----------------------- Standard services ------------------------------

    BOOST_CONCEPT_ASSERT((CColorMap<TColormap>));

    BOOST_STATIC_ASSERT((boost::is_same< typename TColormap::ValueType, 
			 typename TImage::ValueType>::value));
    
    typedef TImage Image;
    typedef typename TImage::ValueType ValueType;
    typedef TColormap Colormap;

    /** 
     * Export an Image to  Raw format (8bits). The pipeline can be sketched
     * as follows: ValueType --<colormap>--> Board::Color ----> unsigned char.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    static bool exportRaw8(const std::string & filename, const Image &aImage, 
			  const ValueType & minV, const ValueType & maxV);
    
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers//RawWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RawWriter_h

#undef RawWriter_RECURSES
#endif // else defined(RawWriter_RECURSES)
