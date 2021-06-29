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
 * @file ScaledMedialAxis.h
 * @brief Linear in time distance transformation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/12/08
 *
 * Header file for module 
 *
 * This file is part of the DGtal library.
 *
 * @see testReducedMedialAxis.cpp, testReducedMedialAxisND.cpp, testReverseDT.cpp
 */

#if defined(ScaledMedialAxis_RECURSES)
#error Recursive header files inclusion detected in ScaledMedialAxis.h
#else // defined(ScaledMedialAxis_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ScaledMedialAxis_RECURSES

#if !defined ScaledMedialAxis_h
/** Prevents repeated inclusion of headers. */
#define ScaledMedialAxis_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <math.h>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/geometry/volumes/distance/CPowerSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/PowerMap.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/geometry/volumes/distance/ReducedMedialAxis.h"
#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/Image.h"
#include "DGtal/images/ImageSelector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ReducedMedialAxis
  /**
   * Description of template class 'ReducedMedialAxis' <p>
   * \brief Aim: Implementation of the separable medial axis
   * extraction.
   *
   * This utility struct extract medial axis balls from a
   * PowerMap. Basically, each (weighted) site of the PowerMap defines
   * a digital maximal ball if its digital power cell restricted to
   * the input shape is not empty @cite dcoeurjo_pami_RDMA .
   *
   *        Optimal Separable Algorithms to Compute the Reverse
   *        Euclidean Distance Transformation and Discrete Medial Axis in
   *        Arbitrary Dimension, D. Coeurjolly and A. Montanvert, IEEE
   *        Transactions on Pattern Analysis and Machine Intelligence,
   *        29(3):437-448, 2007.
   *
   * The output is an image associating ball radii (weight of the
   * power map site) to maximal ball centers. Most methods output a
   * lightweight proxy to an image container (of type ImageContainer,
   * see below).
   *
   * @note Following ReverseDistanceTransformation, the input shape is
   * defined as points with negative power distance.
   *
   * @tparam TPowerMap any specialized PowerMap type @tparam
   * TImageContainer any model of CImage to store the medial axis
   * points (default: ImageContainerBySTLVector).
   *
   * @see testReducedMedialAxis.cpp
   */ 
  template <typename TDistanceTransformation,
            typename TPowerSeparableMetric,
            typename TImageContainer =  ImageContainerBySTLMap<typename TDistanceTransformation::Domain, double> >
  struct ScaledMedialAxis
  {
    //MA Container
    typedef Image<TImageContainer> Type;
    typedef TDistanceTransformation DT;

    /**
     * Extract the scaled medial axis from a distance transformation.
     * This methods is in @f$ O(|distanceTransformation|)@f$.
     *
     * @param dt the input DistanceTransformation
     * @param alpha the double ratio we use to expand and compress the RDMA
     *
     * @return a lightweight proxy to the ImageContainer specified in
     * template arguments.
     */
    
    static Type getScaleAxisFromWeightedMap(DT & dt, double alpha, TPowerSeparableMetric powerSeparableMetric) {

      TImageContainer *computedMA = new TImageContainer( dt.domain() );
      TImageContainer *bigdt = new TImageContainer( dt.domain() );
  
      for (typename DT::Domain::ConstIterator it = dt.domain().begin(); it != dt.domain().end(); it++) {
        bigdt->setValue((*it), pow(dt(*it), 2)*alpha);
      }
      
      PowerMap<TImageContainer, TPowerSeparableMetric> power(bigdt->domain(), bigdt, powerSeparableMetric);

      Type BigRdma = ReducedMedialAxis<PowerMap<TImageContainer, TPowerSeparableMetric>, TImageContainer >::getReducedMedialAxisFromPowerMap(power);

      for (typename Type::Domain::ConstIterator it = BigRdma.domain().begin(); it != BigRdma.domain().end(); it++) {
        const auto v = BigRdma(*it); // Value of distance in BigRdma
        //std::cout << v << std::endl;
        const auto compressedValue = v/alpha;
        computedMA->setValue((*it), compressedValue);

      }
      /*
      for (typename Type::Domain::ConstIterator it = computedMA->domain().begin(); it != computedMA->domain().end(); it++) {
        const auto x = (*computedMA)(*it);
        std::cout << x << std::endl;
      }
      */
      return Type( computedMA );
    }
  }; // end of class ScaledMedialAxis

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ScaledMedialAxis_h

#undef ScalededMedialAxis_RECURSES
#endif // else defined(ScaledMedialAxisdesign pa_RECURSES)
