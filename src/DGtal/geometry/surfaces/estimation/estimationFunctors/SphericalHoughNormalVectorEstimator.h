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
 * @file SphericalHoughNormalVectorEstimator.h
 * @brief Computes normal vector directions using Hough (spherical)
 * accumulator.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2016/03/14
 *
 * Header file for module SphericalHoughNormalVectorEstimator.cpp
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(SphericalHoughNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in SphericalHoughNormalVectorEstimator.h
#else // defined(SphericalHoughNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SphericalHoughNormalVectorEstimator_RECURSES

#if !defined SphericalHoughNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define SphericalHoughNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/topology/SCellsFunctors.h>
#include <vector>
#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include <random>
#include "DGtal/math/linalg/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class SphericalHoughNormalVectorEstimator
  /**
   * Description of template class 'SphericalHoughNormalVectorEstimator' <p>
   * \brief Aim:
   *
   *
   * model of CLocalEstimatorFromSurfelFunctor
   *
   * @tparam TSurfel type of surfels
   * @tparam TEmbedder type of functors which embed surfel to @f$ \mathbb{R}^3@f$
   */
  template <typename TSurfel, typename TEmbedder>
  class SphericalHoughNormalVectorEstimator
  {
  public:
    
    typedef TSurfel Surfel;
    typedef TEmbedder SCellEmbedder;
    typedef typename SCellEmbedder::RealPoint RealPoint;
    typedef RealPoint Quantity;
    typedef SimpleMatrix<double,3,3> Matrix;
    
    /**
     * Constructor.
     *
     * @param anEmbedder embedder to map surfel to R^n.
     * @param h grid step
     */
    SphericalHoughNormalVectorEstimator(ConstAlias<SCellEmbedder> anEmbedder,
                                                const double h,
                                                const double minimalAspectRatio = 0.001,
                                                const unsigned int nbTrials = 100,
                                                const unsigned int accumulatorSize = 10,
                                                const unsigned int nbAccumulators = 5) : myEmbedder(&anEmbedder),myH(h), myAspectRatio(minimalAspectRatio),
    myNbTrials( nbTrials), mySize(accumulatorSize) , myNbAccumulators(nbAccumulators)
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
      std::uniform_int_distribution<int> distribution(0, myPoints.size() );
      
      //Accumulators init
      SphericalAccumulator<RealPoint> accum(mySize);
      std::vector< SphericalAccumulator<RealPoint> > accumulators(myNbAccumulators, accum);
      std::vector< Matrix > rotations;
      std::vector< Matrix > inverseRotations;
      
      //We compute random rotations.
      for(auto i=0; i < myNbAccumulators; ++i)
      {
        //we reuse uniform generator
        Matrix m = randomRotation();
        rotations.push_back( m );
        inverseRotations.push_back( m.inverse() );
      }
      
      
      for(auto t = 0; t < myNbTrials ; ++t)
      {
        unsigned int i,j,k;
        
        i = distribution(generator);
        j = distribution(generator);
        while ( (j = distribution(generator)) == i);
        while (( (k = distribution(generator)) == i) || (k == j) );
        
        RealPoint vector = getNormal(i,j,k);
        if (vector.norm() > myAspectRatio)
        {
          //we have an admissible triangle, we push both normal vectors
          for(auto acc=0; acc < myNbAccumulators; ++acc)
          {
            RealPoint shifted = rotations[acc]*vector;
            //trace.info() << "Pushing shifted "<< shifted << " oorig=" << vector<< "  inverse= "<< inverseRotations[acc]*shifted<<std::endl;
            accumulators[acc].addDirection( shifted );
            accumulators[acc].addDirection( -shifted );
          }
        }
      }
      //We return the max bin orientation summing up all accumulators vote
      typename SphericalAccumulator<RealPoint>::Size posPhi,posTheta;
      RealPoint vote;
      for(auto acc=0; acc < myNbAccumulators; ++acc)
      {
        accumulators[acc].maxCountBin(posPhi, posTheta);
        RealPoint dir = inverseRotations[acc]*accumulators[acc].representativeDirection(posPhi, posTheta).getNormalized() ;
        if ( dir.dot(RealPoint(0,0,1)) >0 )
          vote += dir;
        else
          vote += -dir;
      }
      return vote.getNormalized();
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
    
    Matrix randomRotation() const
    {
      double theta = (rand()+0.f)/RAND_MAX * 2* M_PI;
      double phi = (rand()+0.f)/RAND_MAX * 2* M_PI;
      double psi = (rand()+0.f)/RAND_MAX * 2* M_PI;
      Matrix Rt;
      Rt.setComponent(0,0,1);
      Rt.setComponent(1,0,0);
      Rt.setComponent(2,0,0);
      Rt.setComponent(0,1,0);
      Rt.setComponent(1,1,cos(theta));
      Rt.setComponent(2,1,-sin(theta));
      Rt.setComponent(0,2,0);
      Rt.setComponent(1,2,sin(theta));
      Rt.setComponent(2,2,cos(theta));
      
      
      Matrix Rph;
      Rph.setComponent(0,0,cos(phi));
      Rph.setComponent(1,0,0);
      Rph.setComponent(2,0,sin(phi));
      Rph.setComponent(0,1,0);
      Rph.setComponent(1,1,1);
      Rph.setComponent(2,1,0);
      Rph.setComponent(0,2,-sin(phi));
      Rph.setComponent(1,2,0);
      Rph.setComponent(2,2,cos(phi));
      
      Matrix Rps;
      Rps.setComponent(0,0,cos(psi));
      Rps.setComponent(1,0,-sin(psi));
      Rps.setComponent(2,0,0);
      Rps.setComponent(0,1,sin(psi));
      Rps.setComponent(1,1,cos(psi));
      Rps.setComponent(2,1,0);
      Rps.setComponent(0,2,0);
      Rps.setComponent(1,2,0);
      Rps.setComponent(2,2,1);
      
      return Rt*Rph*Rps;
    }
    
    Quantity getNormal(const unsigned int i, const unsigned int j, const unsigned int k) const
    {
      RealPoint v = myPoints[i] - myPoints[j];
      RealPoint u = myPoints[i] - myPoints[k];
      return v.crossProduct(u);
    }
    
    ///Alias of the geometrical embedder
    const SCellEmbedder * myEmbedder;
    
    ///Grid step
    const double myH;
    
    ///Minimal aspect ratio (norm of the cross-product) to consider a given triangle
    const double myAspectRatio;
    
    ///Number of trials in the neignborhood
    const unsigned int myNbTrials;
    
    ///Size of the accumulator
    const unsigned int mySize;
    
    ///Number of randomly shifted spherical accumulators to consider
    const unsigned int myNbAccumulators;
    
    ///vector of embedded surfels
    std::vector<RealPoint> myPoints;
    
  }; // end of class SphericalHoughNormalVectorEstimator
  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SphericalHoughNormalVectorEstimator_h

#undef SphericalHoughNormalVectorEstimator_RECURSES
#endif // else defined(SphericalHoughNormalVectorEstimator_RECURSES)
