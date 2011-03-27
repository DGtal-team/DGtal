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
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   * Default foregroundPredicate : we return true if the value at a
   * point differs from zero.
   *
   * @todo Refactoring needed to generalize this class !
     */
  template <typename Image>
  class SimpleForegroundPredicate
  {
  public:
    BOOST_CONCEPT_ASSERT(( CImageContainer<Image> ));
    
    typedef typename Image::Value Value;

    SimpleForegroundPredicate(const Value maxVal, const Value minVal): 
      myMaxVal(maxVal), myMinVal(minVal) {};
    
    
    bool operator()(const Image &aImage, const typename Image::Point &aPoint) const
    {
      return (aImage(aPoint) >= myMinVal) && (aImage(aPoint) <= myMaxVal);
    }
    
    bool operator()(const Image &aImage, const typename Image::Iterator &it) const
    {
      return (aImage(it) >= myMinVal) && (aImage(it) <= myMaxVal);
    }
    
    bool operator()(const Image &aImage, const typename Image::ConstIterator &it) const
    {
      return (aImage(it) >= myMinVal) && (aImage(it) <= myMaxVal);
    }
    
    bool operator()(const Image &aImage, const typename Image::SpanIterator &it) const
    {
      return (aImage(it) >= myMinVal) && (aImage(it) <= myMaxVal);
    }
    
  private:
    Value myMaxVal, myMinVal;
    
    
  };



}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SimpleForegroundPredicate_h

#undef SimpleForegroundPredicate_RECURSES
#endif // else defined(SimpleForegroundPredicate_RECURSES)
