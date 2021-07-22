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
 * @file LambdaMedialAxis.h
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

#if defined(LambdaMedialAxis_RECURSES)
#error Recursive header files inclusion detected in ReducedMedialAxis.h
#else // defined(ReducedMedialAxis_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LambdaMedialAxis_RECURSES

#if !defined LambdaMedialAxis_h
/** Prevents repeated inclusion of headers. */
#define LambdaMedialAxis_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/geometry/volumes/distance/PowerMap.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/Image.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/geometry/volumes/distance/VoronoiMap.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/geometry/tools/ImplicitBallTools.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class LambdaMedialAxis
  /**
   * Description of template class 'LambdaMedialAxis' <p>
   * \brief Aim: Implementation of the separable lambda medial axis
   * extraction.
   *
   * This utility struct extract lambda medial axis balls from a
   * Set. Basically, each (weighted) site of the PowerMap defines
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
   * @tparam TSet any set type (to be generalized as PointPredicate)
   * @tparam TDistanceTransformation any DistanceTransformation type
   * that has SmallSet as its PointPredicate
   * @tparam TPowerMap any specialized PowerMap type 
   * @tparam TSmallObject a SmallObject type in dimension 2 or 3
   * @tparam TTopology a topology type that match the SmallObject
   * @tparam TImageContainer any model of CImage to store the medial axis
   * points (default: ImageContainerBySTLVector).
   *
   * @see testLambdaMedialAxis.cpp //TODO
   */

  
  template <typename TSpace,
            typename TImageContainer =  ImageContainerBySTLMap<HyperRectDomain<TSpace>, double> > 
  struct LambdaMedialAxis
  {
    typedef TSpace Space;

    typedef DGtal::int32_t Integer;

    typedef typename Space::Point Point;

    typedef typename Space::RealPoint RealPoint;

    typedef HyperRectDomain<Space> Domain;

    typedef ExactPredicateLpSeparableMetric<Space, 2> L2Metric;

    typedef ExactPredicateLpPowerSeparableMetric<Space, 2> L2PowerMetric;

    typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;

    typedef typename DigitalSetSelector < Domain, SMALL_DS + HIGH_ITER_DS >::Type SmallSet;
    
    typedef Image<TImageContainer> Type;
    
    typedef DistanceTransformation<Space, DigitalSet, L2Metric> DT;

    typedef PowerMap<TImageContainer, L2PowerMetric> PMap;

    typedef VoronoiMap<Space, DigitalSet, L2Metric> VMap;

    typedef ReverseDistanceTransformation<TImageContainer, L2PowerMetric> RDT;

    typedef std::pair<Point, double> Ball;

    /**
     * Detail on algorithm
     * 
     * @param set a set of points
     * @param lambda the minimal radius to accept a ball in the lambda medial axis
     * 
     * @return a std::vector< pair <Point, double> > that contains the balls
     * computed by the lambda medial axis
     */
    static
    std::vector<Ball> computeLambdaAxisFromSet(DigitalSet & set, double lambda) {
      // Initialize metrics and topology
      L2Metric l2Metric;
      L2PowerMetric l2PowerMetric;
       
      std::vector<Ball> medialAxis;
      
      TImageContainer *squaredDT = new TImageContainer( set.domain() );
      
      // Building the VoronoiMap, DistanceTransformation,
      // SquaredDistanceTransformation and PowerMap for the next steps
      VMap vmap(set.domain(), set, l2Metric);
      DT dt(set.domain(), set, l2Metric);
      for (typename DT::Domain::ConstIterator it = dt.domain().begin(); it != dt.domain().end(); it++) {
        squaredDT->setValue((*it), pow(dt(*it), 2));
      }
      PMap aPowerMap(squaredDT->domain(), squaredDT, l2PowerMetric);
      
      for (typename PMap::Domain::ConstIterator it = aPowerMap.domain().begin(),
              itend = aPowerMap.domain().end(); it != itend; ++it)
        {
          const auto v  = aPowerMap( *it );
          const auto pv = aPowerMap.projectPoint( v );
          
          if ( aPowerMap.metricPtr()->powerDistance( *it, v, aPowerMap.weightImagePtr()->operator()( pv ) )
                      < NumberTraits<typename L2PowerMetric::Value>::ZERO ) {
            
            //Lambda Medial Axis filter
            Point voronoiPoint(vmap(*it));
            double voronoiDistance = (voronoiPoint - *it).norm(); //Squared Distance to its Voronoi map point
            std::vector<Point> otherVoronoiPoints;
            DigitalSet box(set.domain());
            Point center = *it;
            subBox(center, 1, set, box); // point neighborhood
            for (auto neighbor : box) {
              Point neighborVoronoiPoint(vmap(neighbor)); // Voronoi point of the neighbor
              double distanceToTest = (neighborVoronoiPoint - *it).norm(); // Distance between the point, and the neighbor voronoi point
              if (distanceToTest == voronoiDistance) {
                otherVoronoiPoints.push_back(neighborVoronoiPoint);
              }
            }
            SmallSet R(set.domain());
            ImplicitBall<Space> enclosing(smallestEuclideanEnclosingBall(otherVoronoiPoints, R));
            if (enclosing.radius() > lambda) {
              Ball ball(v, aPowerMap.weightImagePtr()->operator()( pv ) );
              if (std::find(medialAxis.begin(), medialAxis.end(), ball) == medialAxis.end()) {
                medialAxis.push_back(ball);
              }
            }
            // End of filter
          
        }
      }
      
      return medialAxis;
    }

    static std::vector<Ball> computeLambdaAxisFromVector(std::vector<Ball> & ballSet, Domain & domain, double lambda) {

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

      std::vector<Ball> medialAxis;

      medialAxis = computeLambdaAxisFromSet(set, lambda);
      
      return medialAxis;
    }


    /**
     * @param center
     * @param size the box goes from -size to +size in all dimensions
     * @param set the set we want a subBox of
     * 
     * @return a subset of type DigitalSet that contains the box
     * 
     */
    static void subBox(Point & center,
                             Integer size, 
                             DigitalSet & set,
                             DigitalSet & box, 
                             Integer currentDimension = 0, 
                             std::vector<Integer> indexes = std::vector<Integer>()) {
      Integer dimension = Point::dimension;
      if (indexes.size() == 0) {
        for (Integer i = 0; i < dimension; i++) {
          indexes.push_back(-size);
        }
      }
      Point point;
      for (Integer i = 0; i < dimension; i++) {
        point[i] = indexes[i];
      }

      while(indexes[currentDimension] <= size) {
        if (currentDimension == dimension - 1) {
          if (set(center + point)) {
            box.insert(center + point);
          }
        } else {
          subBox(center, size, set, box, currentDimension+1, indexes);
        }
        indexes[currentDimension] += 1;
        point[currentDimension] += 1;
      }

      return;
    }

  }; // end of class LambdaMedialAxis

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LambdaMedialAxis_h

#undef LambdaMedialAxis_RECURSES
#endif // else defined(LambdaMedialAxisdesign pa_RECURSES)
