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

#include <cmath>
#include <iterator>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

namespace DGtal {
 /**
  * Aim: Implement the 3D tangent estimator combining the 2D Lambda MST
  * tangents of two projections of a curve functional along one axis: the
  * projections onto the two coordinate planes containing that main axis.
  *
  * Points are identified by their position along the curve, never by their
  * coordinates, so a curve that visits the same lattice point twice gets
  * one estimate per visit.
  *
  * @tparam Iterator3D random-access iterator over a sequence of 3D integer points
  * @tparam Functor a functor that constructs 3D tangent vector from two 2D projections
  * @tparam LambdaFunctor lambda function, see FunctorsLambdaMST.h
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
    typedef functors::Projector < SpaceND < 2, int > > Projector2d;
    // Direction/eccentricity-from-DSS functor, the one LambdaMST2D uses,
    // applied directly to the maximal segments of the projected context
    // (see Estimate2DTangent).
    typedef TangentFromDSS2DFunctor < SegmentComputer2D, LambdaFunctor > Value2DFunctor;
    typedef typename Value2DFunctor::Value Value2D;
    
    // ----------------------- Standard services ------------------------------
  public:
    LambdaMST3DBy2DEstimator();
    
    /**
     * Initialisation.
     *
     * @pre [itB, itE) is functional along @a axis: its coordinate along
     * @a axis changes by the same +1 or -1 between all consecutive points.
     *
     * @param itB begin iterator
     * @param itE end iterator
     * @param axis the main axis of the functional 3D curve
     */
    void init ( Iterator3D itB, Iterator3D itE, MAIN_AXIS axis );

    /**
     * Initialisation with a wider context.
     *
     * [itB, itE) is the part of the curve whose points eval() may report.
     * [extB, extE), with extB <= itB and itE <= extE, is a possibly larger
     * range used only as context: both
     * projections are segmented over the whole context, so the maximal
     * segments that cover points of [itB, itE) are not cut at itB or itE
     * when the curve allows them to go further. Context points outside
     * [itB, itE) never get a tangent estimate of their own.
     *
     * The two projections used for @a axis are the two coordinate planes
     * containing it (X: XY and XZ, Y: XY and YZ, Z: XZ and YZ).
     *
     * @pre [extB, extE), and so [itB, itE), is functional along @a axis: its
     * coordinate along @a axis changes by the same +1 or -1 between all
     * consecutive points. LambdaMSTAxisLabeller gives ranges and contexts
     * that satisfy it.
     *
     * @param itB   begin of the range whose points may be reported
     * @param itE   end of the range whose points may be reported
     * @param extB  begin of the context, extB <= itB
     * @param extE  end of the context, itE <= extE
     * @param axis  the main axis of the functional 3D curve
     */
    void init ( Iterator3D itB, Iterator3D itE, Iterator3D extB, Iterator3D extE, MAIN_AXIS axis );

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid ( ) const;
    
    /**
     * Tangent directions of the points in [itb, ite), one per curve position.
     *
     * Each tangent is oriented along the traversal of the curve: its
     * component along the main axis has the sign of the curve's advance
     * along that axis. A point is identified by its position, so the
     * estimate for a single point at position it is eval(it, it + 1, ...).
     *
     * @pre itB <= itb < ite <= itE, for the [itB, itE) given to init().
     *
     * @tparam OutputIterator writable iterator.
     * @param itb begin iterator
     * @param ite end iterator
     * @param result writable iterator over a container which stores estimated tangent directions.
     */
    template <typename OutputIterator>
    OutputIterator eval ( Iterator3D itb, Iterator3D ite, OutputIterator result );

    // ------------------------- Internals ------------------------------------
  protected:
    
    // [ctxb, ctxe): the projected context, segmented as a whole.
    // [coreb, coree): the sub-range of it whose points get a tangent.
    template < typename OutputIterator >
    OutputIterator Estimate2DTangent ( TCurve2D::const_iterator ctxb, TCurve2D::const_iterator ctxe,
                                        TCurve2D::const_iterator coreb, TCurve2D::const_iterator coree,
                                        OutputIterator result );

    /**
     * The sign of the context's advance along its main axis (+1 or -1), the
     * orientation every tangent is given. See eval().
     */
    int traversalSign ( ) const;

    // ------------------------- Private Data --------------------------------
  private:
    Iterator3D myCoreBegin; // range whose points eval() may report
    Iterator3D myCoreEnd;
    Iterator3D myBegin; // context, segmented as a whole
    Iterator3D myEnd;
    Functor myFunctor;
    MAIN_AXIS myAxis;
    /// the context projected onto the first and the second plane
    TCurve2D myCurve1, myCurve2;
    /// projectors
    Projector2d myProjXY, myProjXZ, myProjYZ;
    /// projectors of the first and second projection for myAxis
    Projector2d myProj1, myProj2;
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
  
  // Template class LambdaMST3DBy2D
  /**
   * \brief Aim: Simplify creation of the 3D Lambda MST tangent estimator by 2D projections.
   * @tparam Iterator3D random-access iterator over a sequence of 3D integer points
   * @tparam LambdaFunctor lambda function, see FunctorsLambdaMST.h
   * @tparam CONNECTIVITY connectivity of the projected 2D curves
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
