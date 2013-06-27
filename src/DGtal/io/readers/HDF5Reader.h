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
 * @file HDF5Reader.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 *
 * @date 2013/04/16
 *
 * Header file for module HDF5Reader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HDF5Reader_RECURSES)
#error Recursive header files inclusion detected in HDF5Reader.h
#else // defined(HDF5Reader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HDF5Reader_RECURSES

#if !defined HDF5Reader_h
/** Prevents repeated inclusion of headers. */
#define HDF5Reader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/images/CImage.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HDF5Reader
/**
 * Description of class 'HDF5Reader' <p>
 * \brief Aim: Import a HDF5 file with 2D image dataset(s) (8-bit with palette and 24-bit truecolor with INTERLACE_PIXEL).
 *
 * @tparam TImageContainer the image container to use. 
 * @tparam TFunctor the type of functor used in the import (by default set to CastFunctor< TImageContainer::Value>). 
 *
 */
  template <typename TImageContainer, typename TFunctor= CastFunctor< typename TImageContainer::Value > >
  struct HDF5Reader
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain::Vector Vector;
    typedef typename TImageContainer::Value Value;    
    typedef TFunctor Functor;
    
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));
    BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, unsigned char, Value > )) ;    
    
    BOOST_STATIC_ASSERT( (ImageContainer::Domain::dimension == 2) );

    /** 
     * Main method to import a HDF5 image file into an instance of the 
     * template parameter ImageContainer.
     * 
     * @param aFilename the file name to import.
     * @param aFilename the dataset name to import.
     * @param aFunctor the functor used to import and cast the source
     * image values into the type of the image container value (by
     * default set to CastFunctor < TImageContainer::Value > .
     * @param topbotomOrder
     * if true, the point of coordinate (0,0) will be the bottom left
     * corner image point (default) else the center of image
     * coordinate will be the top left of the image (not usual).
     * @return an instance of the ImageContainer.
     *
     */
    static  ImageContainer importHDF5(const std::string & aFilename, const std::string & aDataset,
				      const Functor & aFunctor =  Functor(), bool topbotomOrder = true) throw(DGtal::IOException);
    
 }; // end of class  HDF5Reader

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/HDF5Reader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HDF5Reader_h

#undef HDF5Reader_RECURSES
#endif // else defined(HDF5Reader_RECURSES)
