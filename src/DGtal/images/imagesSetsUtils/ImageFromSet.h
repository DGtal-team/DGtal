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
 * @file ImageFromSet.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/26
 *
 * Header file for module ImageFromSet.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageFromSet_RECURSES)
#error Recursive header files inclusion detected in ImageFromSet.h
#else // defined(ImageFromSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageFromSet_RECURSES

#if !defined ImageFromSet_h
/** Prevents repeated inclusion of headers. */
#define ImageFromSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/images/CImage.h"
#include "DGtal/kernel/sets/CDigitalSet.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ImageFromSet
  /**
   * Description of template class 'ImageFromSet' <p>
   * \brief Aim: Define utilities to convert a digital set into an
   * image.
   *
   * @tparam TImage an model of CImageContainer concept.
   */
  template <concepts::CImage TImage>
  struct ImageFromSet
  {
    typedef TImage Image;
    typedef typename TImage::Value Value;
  

    /** 
     * Create an Image from a DigitalSet. The size of the output image
     * is given from the set bounding box.
     *
     * @tparam Set model of CDigitalSet 
     * @param aSet an instance of Set to convert into an image
     * @param defaultValue the default value for points in the set
     * @param addBorder if true, we add a border of size 1 of
     * defaultValue around the set. 
     * @param itBegin ConstIterator on the set to specify the first point
     * to copy.
     * @param itEnd ConstIterator on the set to specify the last point
     * to copy.
     * @param keepSetDomain if true, the set domain is used to create
     * the resulting image, else the bouding box of the set is used.
    
     * @return an image.
     */
    template <typename Set>
    static
    Image create(const Set &aSet, const Value &defaultValue,
                 const bool addBorder,
                 typename Set::ConstIterator itBegin, 
                 typename Set::ConstIterator itEnd,
                 const bool keepSetDomain=false);

    /** 
     * Create an Image from a DigitalSet. The size of the output image
     * is given from the set bounding box.
     *
     * @tparam Set model of CDigitalSet 
     * @param aSet an instance of Set to convert into an image
     * @param defaultValue the default value for points in the set
     * @param addBorder if true, we add a border of size 1 of
     * defaultValue around the set. 
     * @param keepSetDomain if true, the set domain is used to create
     * the resulting image, else the bouding box of the set is used.
     *
     * @return an image.
     */
    template <typename Set>
    static
    Image create(const Set &aSet, const Value &defaultValue, const bool addBorder=false,
                 const bool keepSetDomain=false)
    {
      return create(aSet,defaultValue,addBorder,aSet.begin(), aSet.end(), keepSetDomain);
    }        
    
    
    /** 
     * Append a Set to an existing image. Only points in the Set
     * between itBegin and itEnd contained in the image domain are
     * considered. 
     * 
     * @tparam Set model of CDigitalSet
     * @param aImage an image
     * @param defaultValue the default value for points in the set
     * @param itBegin ConstIterator on the set to specify the first point
     * to copy of a Set.
     * @param itEnd ConstIterator on the set to specify the last point
     * to copy of a Set.
     */
    template<typename Set>
    static
    void append(Image &aImage, const Value &defaultValue,
                typename Set::ConstIterator itBegin, 
                typename Set::ConstIterator itEnd);
    
    /** 
     * Append a Set to an existing image. Only points in the Set
     * contained in the image domain are considered. 
     * 
     * @tparam Set model of CDigitalSet
     * @param aImage an image
     * @param aSet  an instance of Set to convert into an image
     * @param defaultValue the default value for points in the set
     * to copy.
     */
    template<typename Set>
    static
    void append(Image &aImage, const Set &aSet, const Value &defaultValue)
    {
      append<Set>(aImage,defaultValue,aSet.begin(),aSet.end());
    }
  }   ; // end of class ImageFromSet


 
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/imagesSetsUtils/ImageFromSet.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageFromSet_h

#undef ImageFromSet_RECURSES
#endif // else defined(ImageFromSet_RECURSES)
