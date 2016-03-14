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
 * @file RobustHoughAccumulatorNormalVectorEstimator.h
 * @brief Computes normal vector directions using Hough (spherical)
 * accumulator.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2016/03/14
 *
 * Header file for module RobustHoughAccumulatorNormalVectorEstimator.cpp
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(RobustHoughAccumulatorNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in RobustHoughAccumulatorNormalVectorEstimator.h
#else // defined(RobustHoughAccumulatorNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RobustHoughAccumulatorNormalVectorEstimator_RECURSES

#if !defined RobustHoughAccumulatorNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define RobustHoughAccumulatorNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/topology/SCellsFunctors.h>
#include <vector>
#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include <random>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class RobustHoughAccumulatorNormalVectorEstimator
  /**
   * Description of template class 'RobustHoughAccumulatorNormalVectorEstimator' <p>
   * \brief Aim: 
   *
   *
   * model of CLocalEstimatorFromSurfelFunctor
   *
   * @tparam TSurfel type of surfels
   * @tparam TEmbedder type of functors which embed surfel to @f$ \mathbb{R}^3@f$
   */
  template <typename TSurfel, typename TEmbedder>
  class RobustHoughAccumulatorNormalVectorEstimator
  {
  public:

    typedef TSurfel Surfel;
    typedef TEmbedder SCellEmbedder;
    typedef typename SCellEmbedder::RealPoint RealPoint;
    typedef RealPoint Quantity;
    /**
     * Constructor.
     *
     * @param anEmbedder embedder to map surfel to R^n.
     * @param h grid step
     */
    RobustHoughAccumulatorNormalVectorEstimator(ConstAlias<SCellEmbedder> anEmbedder,
                                                const double h,
                                                const unsigned int nbTrial = 100,
                                                const unsigned int accumulatorSize = 10):
      myEmbedder(&anEmbedder), myH(h) , myNbTrial( nbTrial ) ,  mySize(accumulatorSize)
    {
      
    }

    /**
     * Add the geometrical embedding of a surfel to the point list and
     * update the normal spherical hough voting.
     *
     * @param aSurf a surfel to add
     * @param aDistance  distance of aSurf to the neighborhood boundary (NOT USED HERE)
     */
    void pushSurfel(const Surfel & aSurf,
                    const double aDistance)
    {
      BOOST_VERIFY(aDistance == aDistance);
      RealPoint p = myH * ( myEmbedder->operator()(aSurf) );
      myPoints.push_back(p);
    }

    /**
     * Evaluate the 
     *
     * @return the feature score
     */
    Quantity eval( ) const
    {
      std::default_random_engine generator;
      std::uniform_int_distribution<int> distribution(0,myPoints.size() );
      
      SphericalAccumulator<RealPoint> accumulator(mySize);
      
      for(unsigned int t = 0; t < myNbTrial ; ++t)
      {
        unsigned int i,j,k;
        
        i = distribution(generator);
        j = distribution(generator);
        while ( (j = distribution(generator)) == i);
        while (( (k = distribution(generator)) == i) || (k == j) );
        
        RealPoint vector = getNormal(i,j,k);
        if (vector.norm() != 0.0)
        {
          trace.info()<<" pushing " << vector<<std::endl;
          //we have a random triple (i,j,k)
          accumulator.addDirection( vector );
        }
      }
      
      //We return the max bin orientation-
      typename SphericalAccumulator<RealPoint>::Size posPhi,posTheta;
      accumulator.maxCountBin(posPhi, posTheta);
      return accumulator.representativeDirection(posPhi, posTheta);
    }

    /**
     * Reset the point list.
     *
     */
    void reset()
    {
      myPoints.clear();
    }

  private:

    Quantity getNormal(const unsigned int i, const unsigned int j, const unsigned int k) const
    {
      RealPoint v = myPoints[i] - myPoints[j];
      RealPoint u = myPoints[i] - myPoints[k];
      trace.info()<< "i "<<i<<" j "<<j<<"  k "<<k<<"   "<< u << "  "<<v<<std::endl;
      return v.crossProduct(u);
    }
    
    ///Alias of the geometrical embedder
    const SCellEmbedder * myEmbedder;

    ///vector of embedded surfels
    std::vector<RealPoint> myPoints;
 
    //Grid step
    double myH;

    ///Size of the accumulator
    const unsigned int myNbTrial;
    
    ///Number of trials in the neignborhood
    const unsigned int mySize;
    
    
  }; // end of class RobustHoughAccumulatorNormalVectorEstimator

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RobustHoughAccumulatorNormalVectorEstimator_h

#undef RobustHoughAccumulatorNormalVectorEstimator_RECURSES
#endif // else defined(RobustHoughAccumulatorNormalVectorEstimator_RECURSES)
