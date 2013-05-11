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
 * @file GenericReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Header file for module GenericReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericReader_RECURSES)
#error Recursive header files inclusion detected in GenericReader.h
#else // defined(GenericReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericReader_RECURSES

#if !defined GenericReader_h
/** Prevents repeated inclusion of headers. */
#define GenericReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericReader
  /**
   * Description of template class 'GenericReader' <p>
   * \brief Aim: Provide a mechanism to load with the bestloader according to an image (2D or 3D) filename (by parsing the extension).
   *  
   * The typical use is very simple:
   * - First include the header of the generic reader (and StdDefs) and define image type:
   @code 
   #include "DGTal/io/readers/GenericReader.h"
   #include "DGtal/helpers/StdDefs.h"
   typedef DGtal::ImageContainerBySTLMap<DGtal::Z3i::Domain, unsigned int> Image3D;
   typedef DGtal::ImageContainerBySTLMap<DGtal::Z2i::Domain, unsigned int> Image2D;
   @endcode
   - Use the same import function for both 2D or 3D images:
   @code
   Image3D an3Dimage= DGtal::GenericReader<Image3D>::import("example.vol");
   Image2D an2Dimage= DGtal::GenericReader<Image2D>::import("example.pgm");
   @endcode
   *
   * @advanced the file format value type will be cast to
   * TContainer::Value.  For instance, VOL file format deals with
   * "unsigned char" and if the TContainer::Value type is different, you
   * could have type conversion issues.
   *
   *
   * @tparam TContainer the container (mainly an ImageContainer like ImageContainerBySTLVector or ImageContainerBySTLMap).
   * @tparam Tdim the dimension of the container (by default given by the container).
   *
   *
   *
   */



  template <typename TContainer, int Tdim=TContainer::Point::dimension >
  struct GenericReader
  {
    /**
     * Import a volume nd image file.  For the special format of raw
     * image, the default parameter of the image size must be given in the optional function vector parameter (dimSpace) .
     * @param filename the image filename to imported.
     * @param dimSpace a vector containing the n dimensional image size. 
     *
     **/
    static TContainer import(const std::string &filename, 
			     std::vector<unsigned int> dimSpace= std::vector<unsigned int > () )  throw(DGtal::IOException);
  };


  /**
   * GenericReader
   * Template partial specialisation for volume images of dimension 3
   **/
  template <typename TContainer>
  struct GenericReader<TContainer, 3 >
  {

    /**
     * Import a volume image file.  For the special format of raw
     * image, the default parameter x,y, z need to be updated
     * according to the dimension if the image.
     * @param x the size in the x direction. 
     * @param y the size in the y direction. 
     * @param z the size in the z direction. 
     *
     **/

    static TContainer import(const std::string &filename,  unsigned int x=0, 
			     unsigned int y=0, unsigned int z=0)  throw(DGtal::IOException);

  };

  /**
   * GenericReader
   * Template partial specialisation for volume images of dimension 2
   **/
  template <typename TContainer>
  struct GenericReader<TContainer, 2>
  {

    /**
     * Import a volume image file.  For the special format h5 (you need to set WITH_HDF5 of cmake build),
     *  the default parameter datasetName needs to be updated
     * according to the dimension if the image.
     * @param datasetName  the name of the dataset contained in the image. 
     *
     **/

    static TContainer import(const std::string &filename,  const std::string &datasetName="empty")  throw(DGtal::IOException);

  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/GenericReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericReader_h

#undef GenericReader_RECURSES
#endif // else defined(GenericReader_RECURSES)
