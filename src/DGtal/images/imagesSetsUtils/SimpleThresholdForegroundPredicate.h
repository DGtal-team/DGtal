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
 * @file SimpleThresholdForegroundPredicate.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/26
 *
 * Header file for module SimpleThresholdForegroundPredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleThresholdForegroundPredicate_RECURSES)
#error Recursive header files inclusion detected in SimpleThresholdForegroundPredicate.h
#else // defined(SimpleThresholdForegroundPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleThresholdForegroundPredicate_RECURSES

#if !defined SimpleThresholdForegroundPredicate_h
/** Prevents repeated inclusion of headers. */
#define SimpleThresholdForegroundPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/CountedPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   * Description of template class 'SimpleThresholdForegroundPredicate' <p>
   * \brief Aim: Define a simple Foreground predicate thresholding
   * image values  given a single thresold.
   * More precisely, the functor operator() returns true if the value
   * is greater than a given threshold.
   *
   * @tparam TImage an model of CImageContainer concept. 
   */
  template <typename Image>
  class SimpleThresholdForegroundPredicate
  {
  public:
    BOOST_CONCEPT_ASSERT(( CImage<Image> ));
    
    typedef typename Image::Value Value;
    typedef typename Image::Point Point;

    /** 
     * Constructor. This functor can be used to threshold image values
     * greater (>) than @a value.
     * 
     * @param value  the threshold value.
     */
    SimpleThresholdForegroundPredicate(const Image & aImage,
				       const Value value):
      myImage(new Image(aImage)), myVal(value) {};
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::Point &aPoint) const
    {
      return ((*myImage)(aPoint) > myVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::Iterator &it) const
    {
      return ((*myImage)(it) > myVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::ConstIterator &it) const
    {
      return ((*myImage)(it) > myVal);
    }
    
    /** 
     * @return True if the point belongs to the value interval.
     */
    bool operator()(const typename Image::SpanIterator &it) const
    {
      return ((*myImage)(it) > myVal);
    }
    
  private:
    CountedPtr<Image> myImage;
    Value myVal;
    
  protected:
    SimpleThresholdForegroundPredicate();
    
  };



}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SimpleThresholdForegroundPredicate_h

#undef SimpleThresholdForegroundPredicate_RECURSES
#endif // else defined(SimpleThresholdForegroundPredicate_RECURSES)
