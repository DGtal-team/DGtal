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
#include "DGtal/geometry/volumes/distance/ReverseDistanceTransformation.h"
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
  template <typename TSpace,
            typename TImageContainer =  ImageContainerBySTLMap<HyperRectDomain<TSpace>, double> >
  struct ScaleMedialAxis
  {
    //MA Container
    typedef TSpace Space;

    typedef typename Space::Point Point;

    typedef typename Space::RealPoint RealPoint;

    typedef HyperRectDomain<Space> Domain;

    typedef ExactPredicateLpSeparableMetric<Space, 2> L2Metric;

    typedef ExactPredicateLpPowerSeparableMetric<Space, 2> L2PowerMetric;

    typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;

    typedef DistanceTransformation<Space, DigitalSet, L2Metric> DT;

    typedef ReverseDistanceTransformation<TImageContainer, L2PowerMetric> RDT;

    typedef std::pair<Point, double> Ball;

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
    
    static std::vector<Ball> computeScaleAxisFromDT(DT & dt, double alpha) 
    {
      std::vector<Ball> medialAxis;
      L2PowerMetric l2PowerMetric;

      // SquaredDT construction, there might be a better way to skip this step
      // All balls get alpha times bigger
      TImageContainer *bigdt = new TImageContainer( dt.domain() );
      for (auto it : dt.domain()) {
        bigdt->setValue(it, pow(dt(it), 2) * alpha);
      }

      // Medial Axis Extraction
      PowerMap<TImageContainer, L2PowerMetric> power(bigdt->domain(), bigdt, l2PowerMetric);
      std::vector<Ball> BigRdma = ReducedMedialAxis<PowerMap<TImageContainer, L2PowerMetric>, TImageContainer >::getReducedMedialAxisFromPowerMap(power);
      
      // alpha time reduction of all balls size 
      for (Ball ball : BigRdma) {
        const auto v = ball.second; // Value of distance in BigRdma
        const auto compressedValue = v/alpha;
        Ball compressedBall(ball.first, compressedValue);

        if (std::find(medialAxis.begin(), medialAxis.end(), compressedBall) == medialAxis.end()) {
          medialAxis.push_back(compressedBall);
        }
      }
      return medialAxis;
    }

    /**
     * Extract the scaled medial axis from a distance transformation.
     * This methods is in @f$ O(|distanceTransformation|)@f$.
     *
     * @param ballSet the input std::vector<std::pair<Point, double> > set of balls
     * @param alpha the double ratio we use to expand and compress the RDMA
     *
     * @return a lightweight proxy to the ImageContainer specified in
     * template arguments.
     */
    
    static std::vector<Ball> computeScaleAxisFromVector(std::vector<Ball> & ballSet, Domain & domain, double alpha) {

      L2Metric l2Metric;
      L2PowerMetric l2PowerMetric;
      
      //Reconstruction of the shape
      TImageContainer *weightedMap = new TImageContainer( domain );
      for (Ball ball : ballSet) {
        weightedMap->setValue(ball.first, ball.second);
      }
      
      RDT rdt(domain, *weightedMap, l2PowerMetric);
      DigitalSet set(domain);
      for (typename RDT::Domain::ConstIterator it = rdt.domain().begin(); it != rdt.domain().end(); it++) {
        if (rdt(*it) < 0) {
          set.insert(*it);
        }
      }
      DT dt(domain, set, l2Metric);

      std::vector<Ball> medialAxis;

      medialAxis = computeScaleAxisFromDT(dt, alpha);
      
      return medialAxis;
    }

  }; // end of class ScaledMedialAxis

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ScaledMedialAxis_h

#undef ScalededMedialAxis_RECURSES
#endif // else defined(ScaledMedialAxisdesign pa_RECURSES)
