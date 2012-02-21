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
 * @file FirstOrderLocalDistance.h
 *
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) 
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2012/02/21
 *
 * @brief Distance computation within a small neighborhood around a point
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(FirstOrderLocalDistance_RECURSES)
#error Recursive header files inclusion detected in FirstOrderLocalDistance.h
#else // defined(FirstOrderLocalDistance_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FirstOrderLocalDistance_RECURSES

#if !defined FirstOrderLocalDistance_h
/** Prevents repeated inclusion of headers. */
#define FirstOrderLocalDistance_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <queue>
#include "DGtal/base/Common.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // useful function for computing local distance

  /////////////////////////////////////////////////////////////////////////////
  // template class FirstOrderIncrementalMetric
  /**
   * Description of template class 'FirstOrderIncrementalMetric' <p>
   * \brief Aim: Metric mapping a point p to a value (according to 
   * to the available values of the 1-neighbors of p). 
   * It is a model of CIncrementalMetric.
   *
   * @tparam TPoint type of point
   * @tparam TMetricHelper  any model of CFirstOrderIncrementalMetricHelper 
   */
  template <typename TImage, typename TPointPredicate>
  class L2FirstOrderLocalDistance
  {

    // ----------------------- Types ------------------------------
  public:


    //concept assert
    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));

    //point
    typedef typename TImage::Point Point;
    typedef typename TImage::Dimension Dimension;
    static const Dimension dimension = Point::dimension;

    //value
    typedef typename TImage::Value Value; 

  private: 

    typedef priority_queue<Value> Values; 
  
    // ----------------------- Interface --------------------------------------
  public:

    /** 
     * Euclidean distance computation at @a aPoint , 
     * from the distance values stored in @a aImg
     * of the 1-neighbors of @a aPoint for which 
     * @a aPred equals 'true'.
     *
     * @param aImg any image
     * @param aPred any point predicate
     * @param aPoint the point for which the distance is computed
     *
     * @return the distance value at @a aPoint.
     *
     * @tparam TImage any model of CImage
     * @tparam TPointPredicate any model of CPointPredicate
     */
    Value 
    L2FirstOrderLocalDistance(TImage& aImg, TPointPredicate& aPred, 
                              const Point& aPoint);

    // ----------------------- Internals -------------------------------------

  private: 

    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of the neighbors (one per dimension)
     * @param aDimensionList  the list of relevant dimensions for the computation
     * @return the computed distance.
     */
    Value compute(const Values& aValueList, 
		  Dimensions& aDimensionList) const; 


    /**
     * Returns the squared euclidean norm of the gradient 
     * of the distance function
     * 
     * @param aValue  the distance of the point where the gradient is computed
     * @param aValueList  the distance of the neighbors (one per dimension)
     *
     * @return the computed gradient norm.
     */
    double gradientNorm(const Value& aValue, const Values& aValueList) const;
  }; 

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FirstOrderLocalDistance.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FirstOrderLocalDistance_h

#undef FirstOrderLocalDistance_RECURSES
#endif // else defined(FirstOrderLocalDistance_RECURSES)
