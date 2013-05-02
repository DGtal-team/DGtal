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
 * @file GenericWriter.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Header file for module GenericWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericWriter_RECURSES)
#error Recursive header files inclusion detected in GenericWriter.h
#else // defined(GenericWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericWriter_RECURSES

#if !defined GenericWriter_h
/** Prevents repeated inclusion of headers. */
#define GenericWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerBySTLMap.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericWriter
  /**
   * Description of template class 'GenericWriter' <p>
   * \brief Aim: Provide a mechanism to save image (2D or 3D) into file  with the best saver loader according to an filename (by parsing the extension).
   *  
   * The typical use is very simple:
   * - First include the header of the generic reader (and StdDefs) and define image type:
   @code 
   #include "DGTal/io/readers/GenericWriter.h"
   #include "DGtal/helpers/StdDefs.h"
   typedef DGtal::ImageContainerBySTLMap<DGtal::Z3i::Domain, unsigned int> Image3D;
   typedef DGtal::ImageContainerBySTLMap<DGtal::Z2i::Domain, unsigned int> Image2D;
   @endcode
   - After contructing and filling an image (anImage2D or anImage3D), just save it with:
   @code
   anImage3D >> "aFilename.pgm3d";
   anImage3D >> "aFilename.vol";
   anImage2D >> "aFilename.pgm";
   @endcode
  
  */

  template <typename TContainer, int Tdim=TContainer::Point::dimension, typename TFunctor = DefaultFunctor >
  struct GenericWriter
  {
    /**
     * Export an  image.
     * @param filename the filename of the saved image (with a extension name). 
     * @param anImage the image to be saved. 
     * @param aFunctor to apply image transformation before saving. 
     *
     **/
    static bool exporT(const std::string &filename, const TContainer &anImage,  
		       const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);
  };

  /**
   * GenericWriter
   * Template partial specialisation for volume images of dimension 3
   **/
  template <typename TContainer, typename TFunctor>
  struct GenericWriter<TContainer, 3 , TFunctor>
  {

    /**
     * Export a volume image.
     * @param filename the filename of the saved image (with a extension name). 
     * @param anImage the image to be saved. 
     * @param aFunctor to apply image transformation before saving. 
     *
     **/
    static bool exporT(const std::string &filename,  const TContainer &anImage,
		       const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);

  };

  /**
   * GenericWriter
   * Template partial specialisation for volume images of dimension 2
   **/
  template <typename TContainer, typename TFunctor>
  struct GenericWriter<TContainer, 2, TFunctor>
  {

    /**
     * Write a volume image file.  
     *
     **/

    static bool exporT(const std::string &filename, const TContainer &anImage,
		       const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);

  }; 


  /**
   *  'operator>>' for exporting an ImageContainerBySTLVector.
   *  This operator automatically selects the good method according to
   *  the filename extension (pgm, pgm3D, raw, vol).
   *  
   * @param aContainer the ImageContainerBySTLVector to be exported.
   * @param aFilename the filename of the file to be exported. 
   * @return true, if the export was successful. 
   */
  template <typename TDomain, typename TValue >
  bool
  operator >> ( const ImageContainerBySTLVector<TDomain, TValue> & aContainer,  const std::string & aFilename  ) throw (DGtal::IOException);
  


  /**
   *  'operator>>' for exporting an ImageContainerBySTLMap.
   *  This operator automatically selects the good method according to
   *  the filename extension (pgm, pgm3D, raw, vol).
   *  
   * @param aContainer the ImageContainerBySTLMap to be exported.
   * @param aFilename the filename of the file to be exported. 
   * @return true, if the export was successful. 
   */
  template <typename TDomain, typename TValue >
  bool
  operator >> ( const ImageContainerBySTLMap<TDomain, TValue> & aContainer,  const std::string & aFilename  ) throw (DGtal::IOException);
  





} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/GenericWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericWriter_h

#undef GenericWriter_RECURSES
#endif // else defined(GenericWriter_RECURSES)
