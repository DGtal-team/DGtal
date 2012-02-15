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
#include "DGtal/images/SetValueIterator.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/kernel/CPointFunctor.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/images/CImage.h"
#include "DGtal/kernel/domains/CDomain.h"

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
  void setFromDomainAndPredicate(const I& itb, const I& ite, const O& ito, const P& aPred); 

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
  void setFromDomainAndFunctor(const I& itb, const I& ite, 
			       const O& ito, const F& aFunctor, 
			       const typename F::Value& aThreshold = 0); 

  /**
   * Fill a set through the inserter @a ito
   * with the points lying within the domain 
   * of the image @aImg such whose value 
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

  /// to be continued 

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/images/ImageHelper.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageHelper_h

#undef ImageHelper_RECURSES
#endif // else defined(ImageHelper_RECURSES)
