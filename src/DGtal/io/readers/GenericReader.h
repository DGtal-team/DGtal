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
#include "DGtal/images/CImage.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/readers/LongvolReader.h"
#include "DGtal/io/readers/PPMReader.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/readers/RawReader.h"
#ifdef WITH_HDF5
#include "DGtal/io/readers/HDF5Reader.h"
#endif
#ifdef WITH_MAGICK
#include "DGtal/io/readers/MagickReader.h"
#endif
#ifdef WITH_ITK
#include "DGtal/io/readers/DicomReader.h"
#endif
#include "DGtal/io/colormaps/BasicColorToScalarFunctors.h"

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
   */
  template <typename TContainer, int Tdim=TContainer::Point::dimension >
  struct GenericReader
  {
    BOOST_CONCEPT_ASSERT((  CImage<TContainer> )) ;

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
    BOOST_CONCEPT_ASSERT((  CImage<TContainer> )) ;
    /**
     * Import a volume image file.  For the special format of raw
     * image, the default parameter x,y, z need to be updated
     * according to the dimension if the image.
     *
     * @param filename the image filename to be imported.
     * @param x the size in the x direction.
     * @param y the size in the y direction.
     * @param z the size in the z direction.
     *
     **/

    static TContainer import(const std::string &filename,  unsigned int x=0,
                             unsigned int y=0, unsigned int z=0)  throw(DGtal::IOException);



    /**
     * Import an image file by specifying a value functor
     *  (used for gray scale image format: vol, longvol, pgm3D, raw).
     *
     * @tparam TFunctor The type of the functor (should verify the concept CUnaryFunctor<TFunctor, unsigned char , TContainer::Value > ).
     * @param aFunctor an ColorRGBEncoder. The type of the functor (should verify the concept CUnaryFunctor<TFunctor, TContainer::Value, DGtal::Color > ).
     * @param x x
     * @param y y
     * @param z z
     *
     **/
    template<typename TFunctor>
    static TContainer importWithValueFunctor(const std::string &filename,
                                             const TFunctor &aFunctor,
                                             unsigned int x=0,
                                             unsigned int y=0, unsigned int z=0)  throw(DGtal::IOException){
      BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, unsigned char, typename TContainer::Value > )) ;
      DGtal::IOException dgtalio;
      std::string extension = filename.substr(filename.find_last_of(".") + 1);

      if(extension=="vol")
        return  VolReader<TContainer>::importVol( filename, aFunctor );

      if(extension=="longvol")
        return  LongvolReader<TContainer>::importLongvol( filename, aFunctor  );

      if(extension=="pgm3d"|| extension=="pgm3D" || extension=="p3d" || extension=="pgm")
        return PGMReader<TContainer>::importPGM3D(filename, aFunctor);

      if(extension=="raw")
        {
          ASSERT(x!=0 && y!=0 && z!=0);
          typename TContainer::Point pt (x,y,z);
          return RawReader< TContainer >::importRaw8 ( filename, pt, aFunctor  );
        }

#ifdef WITH_HDF5
      if (extension=="h5")
        return HDF5Reader<TContainer>::importHDF5_3D(filename, "UInt8Array3D", aFunctor);
#endif

#ifdef WITH_ITK
      if(extension=="dcm")
        return DicomReader<TContainer>::importDicom(filename, aFunctor);
#endif

      trace.error() << "Extension " << extension<< " not yet implemented in DGtal GenericReader." << std::endl;
      throw dgtalio;
    }
  };

  /**
   * GenericReader
   * Template partial specialisation for volume images of dimension 2
   **/
  template <typename TContainer>
  struct GenericReader<TContainer, 2>
  {
    BOOST_CONCEPT_ASSERT((  CImage<TContainer> )) ;

    /**
     * Import a volume image file.  For the special format h5 (you need to set WITH_HDF5 of cmake build),
     *  the default parameter datasetName needs to be updated
     * according to the dimension if the image.
     *
     * @param filename the image filename to be imported.
     *
     **/

    static TContainer import(const std::string &filename)  throw(DGtal::IOException);


    /**
     * Import an image file by specifying a color encoder functor
     *  (used only for color image format ppm, ( gif, jpeg, ... if the
     *  magick image lib is installed) .
     *
     * @tparam TFunctor The type of the functor (should verify the concept CUnaryFunctor<TFunctor, TContainer::Value, DGtal::Color > ).
     * @param aFunctor an ColorRGBEncoder. The type of the functor (should verify the concept CUnaryFunctor<TFunctor, TContainer::Value, DGtal::Color > ).
     *  image.
     *
     **/
    template<typename TFunctor>
    static TContainer importWithColorFunctor(const std::string &filename,
                                             const  TFunctor &aFunctor)  throw(DGtal::IOException){

      BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, typename TContainer::Value, DGtal::Color> )) ;
      DGtal::IOException dgtalio;
      //Getting image extension
      std::string extension = filename.substr(filename.find_last_of(".") + 1);

      if(extension=="ppm")
        return PPMReader<TContainer>::importPPM(filename, aFunctor);

      if( extension=="gif" || extension=="jpg" || extension=="png" || extension=="jpeg" || extension=="bmp")
        {
#ifdef WITH_MAGICK
          MagickReader<TContainer> reader;
          return reader.importImage( filename, aFunctor );
#else
          trace.error() << "Extension " << extension<< " not yet implemented in DGtal but you can add Magick option to deal with this image type." << std::endl;
          throw dgtalio;
#endif
        }

      trace.error() << "Extension " << extension<< " not yet implemented in DGtal GenericReader." << std::endl;
      throw dgtalio;
    }


    /**
     * Import an image file by specifying a value functor used for
     *  grayscale image.
     *
     * @tparam TFunctor The type of the functor (should verify the concept CUnaryFunctor<TFunctor, unsigned char, TContainer::Value > ).
     * @param aFunctor to transform input unsigned char of image value into the given image type.
     *  image.
     *
     **/
    template<typename TFunctor>
    static TContainer importWithValueFunctor(const std::string &filename,
                                             const  TFunctor &aFunctor)  throw(DGtal::IOException){
      BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, unsigned char, typename TContainer::Value > )) ;

      DGtal::IOException dgtalio;
      //Getting image extension
      std::string extension = filename.substr(filename.find_last_of(".") + 1);

      if(extension=="pgm")
        return PGMReader<TContainer>::importPGM(filename, aFunctor);

#ifdef WITH_HDF5
      if (extension=="h5")
        return HDF5Reader<TContainer>::importHDF5(filename, "image8bit", aFunctor);
#endif

      trace.error() << "Extension " << extension<< " not yet implemented in DGtal GenericReader." << std::endl;
      throw dgtalio;

    }

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
