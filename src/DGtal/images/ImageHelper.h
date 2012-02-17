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
 * @file ImageHelper.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/15
 *
 * Header file for module ImageHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageHelper_RECURSES)
#error Recursive header files inclusion detected in ImageHelper.h
#else // defined(ImageHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageHelper_RECURSES

#if !defined ImageHelper_h
/** Prevents repeated inclusion of headers. */
#define ImageHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <algorithm>
#include <functional>

#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/CConstSinglePassRange.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/kernel/CPointFunctor.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/SetValueIterator.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /// useful functions
  /**
   * Fill a set through the inserter @a ito
   * with the points of the range [@a itb , @a ite )
   * such that @a aPred is true
   *
   * @param itb begin iterator on points
   * @param ite end iterator on points
   * @param ito output iterator on points
   * @param aPred any predicate
   *
   * @tparam I any model of input iterator
   * @tparam O any model of output iterator
   * @tparam P any model of CPointPredicate
   */
  template<typename I, typename O, typename P>
  void setFromPointsRangeAndPredicate(const I& itb, const I& ite, const O& ito, const P& aPred); 

  /**
   * Fill a set through the inserter @a ito
   * with the points of the range [@a itb , @a ite )
   * such that their associated value 
   * (returned by @a aFunctor ) is less than or
   * equal to @a aThreshold
   *
   * @param itb begin iterator on points
   * @param ite end iterator on points
   * @param ito output iterator on points
   * @param aFunctor any functor on points
   * @param aThreshold any value (default: 0)
   *
   * @tparam I any model of input iterator
   * @tparam O any model of output iterator
   * @tparam F any model of CPointFunctor
   */
  template<typename I, typename O, typename F>
  void setFromPointsRangeAndFunctor(const I& itb, const I& ite, 
				    const O& ito, const F& aFunctor, 
				    const typename F::Value& aThreshold = 0); 

  /**
   * Fill a set through the inserter @a ito
   * with the points lying within the domain 
   * of the image @a aImg whose value 
   * (in the image) is less than or equal to 
   * @a aThreshold
   *
   * @param aImg any image
   * @param ito set inserter
   * @param aThreshold any value (default: 0)
   *
   * @tparam I any model of CConstImage
   * @tparam O any model of output iterator
   */
  template<typename I, typename O>
  void setFromImage(const I& aImg, 
		    const O& ito, 
		    const typename I::Value& aThreshold = 0); 

  /**
   * Fill a set through the inserter @a ito
   * with the points lying within the domain 
   * of the image @a aImg whose value 
   * (in the image) lies between @a low and @a up
   * (both included) 
   *
   * @param aImg any image
   * @param ito set inserter
   * @param low lower value
   * @param up upper value
   *
   * @tparam I any model of CConstImage
   * @tparam O any model of output iterator
   */
  template<typename I, typename O>
  void setFromImage(const I& aImg, 
		    const O& ito, 
		    const typename I::Value& low,
		    const typename I::Value& up); 


  /**
   * Set the values of @a aImg at @a aValue
   * for each points of the range [ @a itb , @a ite )
   *
   * @param itb begin iterator on points
   * @param ite end iterator on points
   * @param aImg (returned) image
   * @param aValue any value (default: 0)
   *
   * @tparam It any model of forward iterator
   * @tparam Im any model of CImage
   */
  template<typename It, typename Im>
  void imageFromRangeAndValue(const It& itb, const It& ite, Im& aImg, 
			      const typename Im::Value& aValue = 0); 

  /**
   * Set the values of @a aImg at @a aValue
   * for each points of the range @a aRange
   *
   * @param aRange any range
   * @param aImg (returned) image
   * @param aValue any value (default: 0)
   *
   * @tparam R any model of CConstSinglePassRange
   * @tparam I any model of CImage
   */
  template<typename R, typename I>
  void imageFromRangeAndValue(const R& aRange, I& aImg, 
			      const typename I::Value& aValue = 0); 

  /**
   * In a window corresponding to the domain of @a aImg, 
   * copy the values of @a aFun into @a aImg
   *
   * @param aImg (returned) image
   * @param aFun a unary functor
   *
   * @tparam I any model of CImage
   * @tparam F any model of CPointFunctor
   */
  template<typename I, typename F>
  void imageFromFunctor(I& aImg, const F& aFun); 

  /**
   * Copy the values of @a aImg2 into @a aImg1 .
   *
   * @param aImg1 the image to fill
   * @param aImg2 the image to copy
   *
   * @tparam I any model of CImage
   */
  template<typename I>
  void imageFromImage(I& aImg1, const I& aImg2); 


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/images/ImageHelper.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageHelper_h

#undef ImageHelper_RECURSES
#endif // else defined(ImageHelper_RECURSES)
