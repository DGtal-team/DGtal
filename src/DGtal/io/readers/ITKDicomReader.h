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
 * @file ITKDicomReader.h
 * @author Boris Mansencal (\c boris.mansencal@labri.fr )
 * LaBRI (CNRS, UMR 5800, University of Bordeaux, Bordeaux-INP), France
 *
 * @date 2019/02/05
 *
 * Header file for module ITKDicomReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ITKDicomReader_RECURSES)
#error Recursive header files inclusion detected in ITKDicomReader.h
#else // defined(ITKDicomReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ITKDicomReader_RECURSES

#if !defined ITKDicomReader_h
/** Prevents repeated inclusion of headers. */
#define ITKDicomReader_h

#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/images/CImage.h"
#include "DGtal/io/ITKIOTrait.h"
#include "DGtal/images/ImageContainerByITKImage.h"
#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include <itkImageFileReader.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

namespace DGtal
{

  /**
   * Description of template class 'ITKDicomReader'
   * \brief Aim: Import a 2D/3D DICOM Image from file series.
   *
   *  This class requires ITK installation 
   *  (http://www.itk.org/ITK/resources/software.html)
   *  and to compile DGtal with  -DDGTAL_WITH_ITK option.
   *
   *  Simple example: (extract from test/io/readers/testITKDicomReader.cpp)
   *
   *
   * @tparam TImage the Image type.
   * @see ITKWriter
   * @see ITKIOTrait
   */
  template <typename TImage>
  struct ITKDicomReader
  {
    typedef TImage Image;
    typedef typename TImage::Value Value;
    typedef typename ITKIOTrait<Value>::ValueOut ValueOut;

    BOOST_CONCEPT_ASSERT(( concepts::CImage<TImage> ));
    BOOST_STATIC_ASSERT(( (TImage::Domain::dimension == 3)
			  || (TImage::Domain::dimension == 2) ));

    /**
     * Import an Image from files belonging to the same DICOM serie.
     *
     * First an ImageContainerByITKImage is constructed by using the
     * source type of the input ITK image, and in a second step the
     * resulting image type is adapted to the TImage type with the use
     * of the given Functor.
     *
     * @param filenames fullnames of file of a DICOM serie. They may be 
     * gathered with an itk::GDCMSeriesFileNames instance.
     * @param aFunctor functor used to cast image values.
     * @tparam TFunctor the type of functor used in the export.
     *
     * @return read image
     */
    template <typename TFunctor =
              typename ITKIOTrait<typename TImage::Value>::DefaultReadFunctor>
    static Image importDICOM( const std::vector<std::string> & filenames,
			      const TFunctor & aFunctor = TFunctor() );


    private:

    template <typename Domain, typename PixelType>
    static inline
    ImageContainerByITKImage<Domain, PixelType>
    importDicomFiles( const std::vector<std::string> & filenames );
    

    
    template <typename Image, typename Domain, typename OrigValue,
              typename TFunctor, typename Value>
    struct Aux
    {
      static inline Image
      readDGtalImageFromITKtypes( const std::vector<std::string> & filenames,
				  const TFunctor & aFunctor );
    };

    //specialization
    template <typename Domain, typename OrigValue, typename TFunctor,
              typename Value>
    struct Aux<ImageContainerByITKImage<Domain, Value>, Domain, OrigValue,
               TFunctor, Value>
    {
      static inline ImageContainerByITKImage<Domain, Value>
      readDGtalImageFromITKtypes( const std::vector<std::string> & filenames,
				  const TFunctor & aFunctor );
    };
    
    
    /**
     * Read an DGtal image of type TypeDGtalImage from files belonging 
     * to the same DICOM serie.
     *
     * @param filenames fullnames of file of a DICOM serie. They may be 
     * gathered with an itk::GDCMSeriesFileNames instance.
     * @param aFunctor functor used to cast image values
     * @tparam TFunctor the type of functor used in the export.
     *
     * @return read image
     */
    template <typename TypeDGtalImage, typename TFunctor>
    static Image
    readDGtalImageFromITKtypes( const std::vector<std::string> & filenames,
				const TFunctor & aFunctor );
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/ITKDicomReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ITKDicomReader_h

#undef ITKDicomReader_RECURSES
#endif // else defined(ITKDicomReader_RECURSES)
