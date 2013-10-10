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
 * @file DicomReader.h
 * @author Adrien Krähenbühl (\c adrien.krahenbuhl@loria.fr )
 * LORIA (CNRS, UMR 7503), Université de Lorraine, France
 *
 * @date 2013/10/10
 *
 * Header file for module DicomReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DicomReader_RECURSES)
#error Recursive header files inclusion detected in DicomReader.h
#else
/** Prevents recursive inclusion of headers. */
#define DicomReader_RECURSES

#if !defined DicomReader_h
/** Prevents repeated inclusion of headers. */
#define DicomReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"

#include <itkMetaDataDictionary.h>

//////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
#pragma warning(disable : 4290)
#endif

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class DicomReader
/**
 * Description of class 'DicomReader' <p>
 * \brief Aim: Import a 2D or 3D using the Netpbm formats (ASCII mode).
 * - PPM: RGB
 *  - Dicom: grayscale
 *  - PPM3D: 3D variant of PPM
 *  - Dicom3D: 3D variant of Dicom
 *
 *
 *  Simple example: (extract from test file testDicomReader.cpp)
 *
 *  @code
 *  #include "DGtal/helpers/StdDefs.h"
 *  #include "DGtal/io/readers/DicomReader.h"
 *  #include "DGtal/kernel/images/ImageSelector.h"
 *  ...
 *  string filename = "test.Dicom";
 *  typedef ImageSelector < Z2i::Domain, uint>::Type Image;
 *  Image image = DicomReader<Image>::importDicomImage( filename );
 *   @endcode
 *  You can then for instance display a threshold part of the image:
 *  @code
 *  #include "DGtal/kernel/imagesSetsUtils/SetFromImage.h"
 *  ...
 *  Z2i::DigitalSet set2d (image.domain());
 *  // Threshold all pixel in ]0, 255] in a DigitalSet
 *  SetFromImage<Z2i::DigitalSet>::append<Image>(set2d, image, 0, 255);
 *  Board2D board;
 *  board << image.domain() << set2d; // display domain and set
 *  @endcode
 *
 * @tparam TImageContainer the type of the image container
 *
 * @tparam TFunctor the type of functor used in the import (by default set to CastFunctor< TImageContainer::Value>) .
 *
 */
  template <typename TImageContainer,
		typename TFunctor = CastFunctor< typename TImageContainer::Value > >
  struct DicomReader
  {
	// ----------------------- Standard services ------------------------------
  public:

	typedef TImageContainer ImageContainer;
	typedef typename TImageContainer::Value Value;
	typedef typename TImageContainer::Domain::Vector Vector;

	typedef TFunctor Functor;

	BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, unsigned char, Value > )) ;



	BOOST_STATIC_ASSERT(ImageContainer::Domain::dimension == 3);

	/**
	 * Main method to import a Dicom (8bits) into an instance of the
	 * template parameter ImageContainer.
	 *
	 * @param aFilename the file name to import.
	 * @param aFunctor the functor used to import and cast the source
	 * image values into the type of the image container value (by
	 * default set to CastFunctor < TImageContainer::Value > .
	 *
	 * @param topbotomOrder
	 * if true, the point of coordinate (0,0) will be the bottom left
	 * corner image point (default) else the center of image
	 * coordinate will be the top left of the image (not usual).
	 * @return an instance of the ImageContainer.
	 */
	static  ImageContainer importDicom(const std::string & aDirname,
									 const Functor & aFunctor =  Functor()) throw(DGtal::IOException);


	//! Dicom tag corresponding to the entry ID from dictionary
	static std::string getTag( const std::string &entryId, const itk::MetaDataDictionary &dictionary );

 }; // end of class  DicomReader



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/DicomReader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DicomReader_h

#undef DicomReader_RECURSES
#endif // else defined(DicomReader_RECURSES)
