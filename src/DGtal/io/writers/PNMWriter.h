#pragma once

/**
 * @file PNMWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/22
 *
 * Header file for module PNMWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PNMWriter_RECURSES)
#error Recursive header files inclusion detected in PNMWriter.h
#else // defined(PNMWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PNMWriter_RECURSES

#if !defined PNMWriter_h
/** Prevents repeated inclusion of headers. */
#define PNMWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PNMWriter
  /**
   * Description of template struct 'PNMWriter' <p>
   * \brief Aim: Export a 2D Image using the Netpbm formats.
   *
   * @tparam TImage the Image type.
   * @tparam TColormap the type of the colormap to use in the export.
   */
  template <typename TImage, typename TColormap>
  struct PNMWriter
  {
    // ----------------------- Standard services ------------------------------

    typedef TImage Image;
    typedef typename TImage::ValueType ValueType;
    typedef TColormap Colormap;

    /** 
     * Export an Image with PPM format.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    bool exportPPM(const std::string & filename, const Image &aImage, 
		   const ValueType & minV, const ValueType & maxV) const;
    
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers//PNMWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PNMWriter_h

#undef PNMWriter_RECURSES
#endif // else defined(PNMWriter_RECURSES)
