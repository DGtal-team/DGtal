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
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/io/colormaps/CColorMap.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PNMWriter
  /**
   * Description of template struct 'PNMWriter' <p>
   * \brief Aim: Export a 2D and a 3D Image using the Netpbm formats (ASCII mode).
   *  - PPM: RGB 
   *  - PGM: grayscale
   *  - PPM3D: 3D variant of PPM
   *  - PGM3D: 3D variant of PGM
   *
   * Usage example:
   * @code
   * ...
   * typedef SpaceND<int,2> TSpace;
   * typedef TSpace::Point Point;
   * typedef HyperRectDomain<TSpace> Domain;
   * typedef HueShadeColorMap<unsigned char> Hue;
   * typedef ImageSelector<Domain, unsigned char>::Type Image;
   * ...
   * Point a ( 1, 1);
   * Point b ( 16, 16);
   * Image image(a,b);
   * ... //Do something in image
   * PNMWriter<Image,Hue>::exportPPM("export-hue.ppm",image,0,255);
   * 
   * @endcode
   *
   * @tparam TImage the Image type.
   * @tparam TColormap the type of the colormap to use in the export.
   *
   * @todo check if a magick number exists for PGM3D/PPM3D.
   *
   * @see testPNMRawWriter.cpp
   */
  template <typename TImage, typename TColormap>
  struct PNMWriter
  {
    // ----------------------- Standard services ------------------------------

    BOOST_CONCEPT_ASSERT((CColorMap<TColormap>));

    BOOST_STATIC_ASSERT( (TImage::Domain::dimension == 2) || 
       (TImage::Domain::dimension == 3));

    BOOST_STATIC_ASSERT((boost::is_same< typename TColormap::Value, 
       typename TImage::Value>::value));
    
    typedef TImage Image;
    typedef typename TImage::Value Value;
    typedef TColormap Colormap;

    /** 
     * Export an Image with PPM format. The colormap specified
     * in the template arguments is used to convert Value 
     * to RBG colors.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    static bool exportPPM(const std::string & filename, const Image &aImage, 
			  const Value & minV, const Value & maxV, bool topbotomOrder=true);

    /** 
     * Export an Image with PPM3Dformat. The colormap specified
     * in the template arguments is used to convert Value 
     * to RBG colors.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    static bool exportPPM3D(const std::string & filename, const Image &aImage, 
          const Value & minV, const Value & maxV);


    /** 
     * Export an Image with PGM format. The colormap specified
     * in the template arguments is used to convert Value 
     * to RBG colors. Then, a RGB to Grayscale conversion is performed.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * @param saveASCII used to save image with ASCII pixel value and with white space. 
     *        (default= false since ASCII mode is not efficient).     
     * 
     * 
     * @return true if no errors occur.
     */
    static bool exportPGM(const std::string & filename, const Image &aImage, 
			  const Value & minV, const Value & maxV,  bool saveASCII=false, bool topbotomOrder=true);
  

    /** 
     * Export an Image with PGM3D format. The colormap specified
     * in the template arguments is used to convert Value 
     * to RBG colors. Then, a RGB to Grayscale conversion is performed.
     * 
     * @param filename name of the output file
     * @param aImage the image to export
     * @param minV the minimum value of aImage (for colormap)
     * @param maxV the maximum value of aImage (for colormap) 
     * 
     * @return true if no errors occur.
     */
    static bool exportPGM3D(const std::string & filename, const Image &aImage, 
			    const Value & minV, const Value & maxV);
    
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/PNMWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PNMWriter_h

#undef PNMWriter_RECURSES
#endif // else defined(PNMWriter_RECURSES)
