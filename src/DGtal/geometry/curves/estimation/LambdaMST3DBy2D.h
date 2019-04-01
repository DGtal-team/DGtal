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
 * @file LambdaMST3DBy2D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2015/06/16
 *
 * This file is part of the DGtal library.
 */

#if defined(LAMBDAMST3DBy2D_RECURSES)
#error Recursive header files inclusion detected in LambdaMST3DBy2D.h
#else // defined(LAMBDAMST3DBy2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LAMBDAMST3DBy2D_RECURSES

#if !defined LAMBDAMST3DBy2D_h
/** Prevents repeated inclusion of headers. */
#define LAMBDAMST3DBy2D_h

#include <algorithm>
#include <cmath>
#include <list>
#include <iterator>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/geometry/curves/estimation/LambdaMST2D.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

namespace DGtal {
 /**
  * Aim: Implement 3D Lambda MST tangent estimators. This class is a model of CCurveLocalGeometricEstimator.
  * @tparam Iterator3D iterator over a sequence of 3D integer points
  * @tparam Functor a functor that constructs 3D tangent vector from two 2D projections
  * @tparam LambdaFunctor model of CLMSTTangentFrom2DSS
  * @tparam CONNECTIVITY connectivity for the projected curves used by 2D Lambda estimators. Note that, for now
  * this value cannot be set per projection
  */
  template < typename Iterator3D, typename Functor, typename LambdaFunctor, int CONNECTIVITY = 8 >
  class LambdaMST3DBy2DEstimator
  {
  public:
    // ----------------------- Types ------------------------------
  public:
    typedef PointVector < 3, double > RealVector3D;
    typedef PointVector < 3, int > Point3D;
    typedef PointVector < 2, int > Point2D;
    typedef PointVector < 2, double > RealVector2D;
    typedef std::vector < Point2D > TCurve2D;
    typedef ArithmeticalDSSComputer < typename TCurve2D::const_iterator, int, CONNECTIVITY > SegmentComputer2D;
    typedef SaturatedSegmentation < SegmentComputer2D > Segmentation2D;
    typedef typename Functor::MAIN_AXIS MAIN_AXIS;

    // ----------------------- Private types ------------------------------
  private:
    typedef LambdaMST2D < Segmentation2D, LambdaFunctor > TEstimator;
    typedef functors::Projector < SpaceND < 2, int > > Projector2d;
    
    // ----------------------- Standard services ------------------------------
  public:
    LambdaMST3DBy2DEstimator();
    
    /**
     * Initialisation.
     * @param itB begin iterator
     * @param itE end iterator
     * @param axis the main axis of the functional 3D curve
     */
    void init ( Iterator3D itB, Iterator3D itE, MAIN_AXIS axis );
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid ( ) const;
    
    /**
     * @param point to calculate A and B for it
     * @return A and B
     */
    RealVector3D eval ( const Point3D & point );

    /**
     * @tparam OutputIterator writable iterator.
     * A way to compute tangent for points in a range. NOTE: In contrary to LambdaMST3D::eval() this method
     * is NOT faster than calling eval (const Point &) for each points in a range. In fact, eval ( const Point & )
     * is internally called.
     * @param itb begin iterator
     * @param ite end iterator
     * @param result writable iterator over a container which stores estimated tangent directions.
     */
    template <typename OutputIterator>
    OutputIterator eval ( Iterator3D itb, Iterator3D ite, OutputIterator result );

    // ------------------------- Internals ------------------------------------
  protected:
    
    RealVector2D Estimate2DTangent ( TCurve2D::const_iterator itb, TCurve2D::const_iterator ite, const Point2D & point );
    template < typename OutputIterator >
    OutputIterator Estimate2DTangent ( TCurve2D::const_iterator itb, TCurve2D::const_iterator ite, OutputIterator result );

    // ------------------------- Private Datas --------------------------------
  private:
    Iterator3D myBegin;
    Iterator3D myEnd;
    Functor myFunctor;
    MAIN_AXIS myAxis;
    TCurve2D tXY, tXZ, tYZ;
    /// projectors
    Projector2d myProjXY, myProjXZ, myProjYZ;
  }; // end of class LambdaMST3DBy2DEstimator
  
  
  class TangentFromDSS3DBy2DFunctor
  {
  public:
    // ----------------------- Types ------------------------------
    typedef PointVector<3, double> Vector3D;
    typedef PointVector<2, double> Vector2D;
    enum MAIN_AXIS {X = 0, Y = 1, Z = 2};
    
    // ----------------------- Interface --------------------------------------
    Vector3D operator() ( MAIN_AXIS mainAxis, const Vector2D & v0, const Vector2D & v1 ) const;
  };
  
  //-------------------------------------------------------------------------------------------
  
  // Template class LambdaMST3D
  /**
   * \brief Aim: Simplify creation of Lambda MST tangent estimator.
   *
   */
  template < 
  typename Iterator3D, typename LambdaFunctor = functors::Lambda64Function, int CONNECTIVITY = 8 >
  class LambdaMST3DBy2D:
  public LambdaMST3DBy2DEstimator < Iterator3D, TangentFromDSS3DBy2DFunctor, LambdaFunctor, CONNECTIVITY >
  {
    typedef LambdaMST3DBy2DEstimator < Iterator3D, TangentFromDSS3DBy2DFunctor, LambdaFunctor, CONNECTIVITY > Super;
    
  public:
    /**
     * Default Constructor.
     */
    LambdaMST3DBy2D() : Super() {}
  };
}// namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LAMBDAMST3DBy2D_h

#undef LAMBDAMST3DBy2D_RECURSES
#endif // else defined(LAMBDAMST3DBy2D_RECURSES)
