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
 * @file ITKReader.h
 * @author Pierre Gueth (\c pierre.gueth@gmail.com )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS,
 * UMR 5205), CNRS, France
 *
 * @author Bertrand Kerautret (\c bertrand.kerautret@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Lorraine, France
 *
 * @date 2013/10/28
 *
 * Header file for module ITKReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ITKReader_RECURSES)
#error Recursive header files inclusion detected in ITKReader.h
#else // defined(ITKReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ITKReader_RECURSES

#if !defined ITKReader_h
/** Prevents repeated inclusion of headers. */
#define ITKReader_h

#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/BasicFunctors.h"
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
   * Description of template class 'ITKReader'
   * \brief Aim: Import a 2D/3D Image using the ITK formats.
   *
   * @tparam TImage the Image type.
   * @see ITKWriter
   * @see ITKIOTrait
   */
  template <typename TImage>
  struct ITKReader
  {
    typedef TImage Image;
    typedef typename TImage::Value Value;
    typedef typename ITKIOTrait<Value>::ValueOut ValueOut;

    BOOST_CONCEPT_ASSERT(( concepts::CImage<TImage> ));
    BOOST_STATIC_ASSERT(( (TImage::Domain::dimension == 3)
			  || (TImage::Domain::dimension == 2) ));

    /**
     * Import an Image with a format supported by ITK.
     *
     * First an ImageContainerByITKImage is constructed by using the
     * source type of the input ITK image, and in a second step the
     * resulting image type is adapted to the TImage type with the use
     * of the given Functor.
     *
     * @param filename name of the input file.
     * @param aFunctor functor used to cast image values.
     * @param shiftDomainUsingOrigin  shift the image domain according to the physical information of the source image)
     * @tparam TFunctor the type of functor used in the export.
     *
     * @return read image
     */
    template <typename TFunctor =
              typename ITKIOTrait<typename TImage::Value>::DefaultReadFunctor>
    static Image importITK(
    const std::string & filename,
    const TFunctor & aFunctor = TFunctor(), bool shiftDomainUsingOrigin=true);

    /**
     * Import an Image with a format supported by ITK.
     *
     * First an ImageContainerByITKImage is constructed by using the
     * source type of the input ITK image, and in a second step the
     * resulting image type is adapted to the TImage type with the use
     * of a default ITKIOTrait::DefaultReadFunctor.
     *
     * @param filename name of the input file.
     * @param shiftDomainUsingOrigin  shift the image domain according to the physical information of the source image)
     * @return read image
     */
    static Image importITK(const std::string & filename, bool shiftDomainUsingOrigin);


    /**
     * Get the type of the ITK image.
     *
     * @param filename  name of the input file.
     * @return the ITK image component type.
     *
     **/
    static itk::ImageIOBase::IOComponentType
    getITKComponentType( const std::string & filename );

    private:

    template <typename Domain, typename PixelType>
    static inline ImageContainerByITKImage<Domain, PixelType>
    readDGtalITKImage(const std::string & filename, bool shiftDomainUsingOrigin=true);


    template <typename Image, typename Domain, typename OrigValue,
              typename TFunctor, typename Value>
    struct Aux
    {
      static inline Image
      readDGtalImageFromITKtypes( const std::string & filename,
				  const TFunctor & aFunctor, bool shiftDomainUsingOrigin=true);
    };

    //specialization
    template <typename Domain, typename OrigValue, typename TFunctor,
              typename Value>
    struct Aux<ImageContainerByITKImage<Domain, Value>, Domain, OrigValue,
               TFunctor, Value>
    {
      static inline ImageContainerByITKImage<Domain, Value>
      readDGtalImageFromITKtypes( const std::string & filename,
				  const TFunctor & aFunctor, bool shiftDomainUsingOrigin=true);
    };


    /**
     * Read an DGtal image of type TypeDGtalImage with a format supported by
     * ITK. (used by importITK)
     *
     * @param filename name of the input file
     * @param aFunctor functor used to cast image values
     * @param shiftDomainUsingOrigin  shift the image domain according to the physical information of the source image)
     * @tparam TFunctor the type of functor used in the export.
     *
     * @return read image
     */
    template <typename TypeDGtalImage, typename TFunctor>
    static Image readDGtalImageFromITKtypes(
    const std::string & filename,
    const TFunctor & aFunctor, bool shiftDomainUsingOrigin=true);
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/ITKReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ITKReader_h

#undef ITKReader_RECURSES
#endif // else defined(ITKReader_RECURSES)
