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
 * @file SimpleForegroundPredicate.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/26
 *
 * Header file for module SimpleForegroundPredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleForegroundPredicate_RECURSES)
#error Recursive header files inclusion detected in SimpleForegroundPredicate.h
#else // defined(SimpleForegroundPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleForegroundPredicate_RECURSES

#if !defined SimpleForegroundPredicate_h
/** Prevents repeated inclusion of headers. */
#define SimpleForegroundPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/images/CImageContainer.h"
#include "DGTal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   * Description of template class 'SetFromImage' <p>
   * \brief Aim: Define a simple Foreground predicate thresholding
   * image values  between to constant values.
   *
   * @tparam TImage an model of CImageContainer concept. 
   */
  template <typename Image>
  class SimpleForegroundPredicate
  {
  public:
    BOOST_CONCEPT_ASSERT(( CImageContainer<Image> ));
    
    typedef typename Image::Value Value;

    /** 
     * Constructor. This functor can be used to threshold image values
     * in the interval ]minVal,maxVal].
     * 
     * @param minVal the minimum value (first value excluded).
     * @param maxVal the maximum value (last value considered).
     */
    SimpleForegroundPredicate(const Image & aImage,
			      const Value minVal, 
			      const Value maxVal): 
      myImage(new Image(aImage)), myMaxVal(maxVal), myMinVal(minVal) {};
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::Point &aPoint) const
    {
      return ((*myImage)(aPoint) > myMinVal) && ((*myImage)(aPoint) <= myMaxVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::Iterator &it) const
    {
      return ((*myImage)(it) > myMinVal) && ((*myImage)(it) <= myMaxVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::ConstIterator &it) const
    {
      return ((*myImage)(it) > myMinVal) && ((*myImage)(it) <= myMaxVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::SpanIterator &it) const
    {
      return ((*myImage)(it) > myMinVal) && ((*myImage)(it) <= myMaxVal);
    }
    
  private:
    Value myMaxVal, myMinVal;
    CountedPtr<Image> myImage;
    
  protected:
    SimpleForegroundPredicate();
    
  };



}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SimpleForegroundPredicate_h

#undef SimpleForegroundPredicate_RECURSES
#endif // else defined(SimpleForegroundPredicate_RECURSES)
