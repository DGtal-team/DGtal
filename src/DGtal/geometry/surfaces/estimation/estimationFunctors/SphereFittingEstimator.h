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
 * @file SphereFittingEstimator.h
 * @brief Computes the true quantity to each element of a range associated to a parametric shape.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/31
 *
 * Header file for module SphereFittingEstimator.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testLengthEstimators.cpp, testTrueLocalEstimator.cpp
 */

#if defined(SphereFittingEstimator_RECURSES)
#error Recursive header files inclusion detected in SphereFittingEstimator.h
#else // defined(SphereFittingEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SphereFittingEstimator_RECURSES

#if !defined SphereFittingEstimator_h
/** Prevents repeated inclusion of headers. */
#define SphereFittingEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/topology/SCellsFunctors.h>

#ifndef WITH_PATATE
#error You need to have activated Patate (WITH_PATATE) to include this file.
#endif

//Patate
#include <Patate/grenaille.h>
#include <Eigen/Eigen>
#include <vector>

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace functors
  {
    /////////////////////////////////////////////////////////////////////////////
    // template class SphereFittingEstimator
    /**
     * Description of template class 'SphereFittingEstimator' <p>
     * \brief Aim: 
     *
     * model of CLocalEstimatorFromSurfelFunctor.
     *
     *
     * @tparam TSurfel type of surfels
     * @tparam TEmbedder type of functors which embed surfel to @f$ \mathbb{R}^3@f$
     */
    template <typename TSurfel, typename TEmbedder>
    class SphereFittingEstimator
    {
    public:

      
      class PatatePoint
      {
      public:
	enum {Dim = 3};
	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, Dim, 1>   VectorType;
	typedef Eigen::Matrix<Scalar, Dim, Dim> MatrixType;
        
	MULTIARCH inline PatatePoint(const VectorType& _pos    = VectorType::Zero(), 
                                     const VectorType& _normal = VectorType::Zero())
          : m_pos(_pos), m_normal(_normal) {}
        
	MULTIARCH inline const VectorType& pos()    const { return m_pos; }  
	MULTIARCH inline const VectorType& normal() const { return m_normal; }

	MULTIARCH inline VectorType& pos()    { return m_pos; }  
	MULTIARCH inline VectorType& normal() { return m_normal; }

      private:
	VectorType m_pos, m_normal;
      };
      
      typedef TSurfel Surfel;
      typedef TEmbedder SCellEmbedder;
      typedef double Quantity;
      typedef typename SCellEmbedder::RealPoint RealPoint;

      typedef PatatePoint::Scalar Scalar;
      typedef PatatePoint::VectorType VectorType;
      
      typedef Grenaille::DistWeightFunc<PatatePoint,Grenaille::SmoothWeightKernel<Scalar> > WeightFunc; 
      typedef Grenaille::Basket<PatatePoint,WeightFunc,Grenaille::OrientedSphereFit, Grenaille::GLSParam> Fit;

      /**
       * Constructor.
       *
       * @param anEmbedder embedder to map surfel to R^n.
       * @param h gridstep.
       */
      SphereFittingEstimator(ConstAlias<SCellEmbedder> anEmbedder,
                             const double h:
        myEmbedder(&anEmbedder), myH(h) 
      {
      }

      /**
       * Add the geometrical embedding of a surfel to the point list
       *
       * @param aSurf a surfel to add
       * @param aDistance of aSurf to the neighborhood boundary
       */
      void pushSurfel(const Surfel & aSurf,
                      const double aDistance)
      {
        BOOST_VERIFY(aDistance==aDistance);

        RealPoint p = myEmbedder->operator()(aSurf);
        PatatePoint pp(p[0]*myH,p[1]*myH,p[2]*myH);
        
        myPoints.push_back(pp);
      }

      /**
       * Evaluate the curvature from Monge form.
       *
       * @return the mean curvature
       */
      Quantity eval()
      {
        /*  CGALMongeForm monge_form;
        CGALMongeViaJet monge_fit;
        
        monge_form = monge_fit(myPoints.begin() , myPoints.end(), myD, (2<myD)? myD : 2);

        double k1 = monge_form.principal_curvatures ( 0 );
        double k2 = monge_form.principal_curvatures ( 1 );
        return 0.5*(k1+k2);*/
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

      ///Alias of the geometrical embedder
      const SCellEmbedder * myEmbedder;

      ///Array of CGAL points
      std::vector<PatatePoint> myPoints;

      ///Grid step
                             double myH;

    }; // end of class SphereFittingEstimator
  }
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SphereFittingEstimator_h

#undef SphereFittingEstimator_RECURSES
#endif // else defined(SphereFittingEstimator_RECURSES)
