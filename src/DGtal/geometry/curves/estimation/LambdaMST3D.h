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
 * @file LambdaMST3D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/10/06
 *
 * This file is part of the DGtal library.
 */

#if defined(LAMBDAMST3D_RECURSES)
#error Recursive header files inclusion detected in LambdaMST3D.h
#else // defined(LAMBDAMST3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LAMBDAMST3D_RECURSES

#if !defined LAMBDAMST3D_h
/** Prevents repeated inclusion of headers. */
#define LAMBDAMST3D_h

#include <algorithm>
#include <iterator>
#include <cmath>
#include <map>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"
#include "DGtal/geometry/curves/CForwardSegmentComputer.h"
#include "DGtal/geometry/curves/estimation/CLMSTTangentFromDSS.h"

namespace DGtal {
  /**
   * Aim: Implement Lambda MST tangent estimator.
   * @tparam TSpace model of CSpace
   * @tparam TSegmentation tangential cover obtained by a segmentation of a 2D digital curve by maximal straight segments
   * @tparam Functor model of CLMSTTangentFrom2DSS
   */
  template < typename TSpace, typename TSegmentation, typename Functor >
  class LambdaMST3DEstimator
  {
  public: 
    //Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 3 ));
    BOOST_CONCEPT_ASSERT(( concepts::CLMSTTangentFromDSS<Functor> ));
    BOOST_CONCEPT_ASSERT(( CForwardSegmentComputer<typename TSegmentation::SegmentComputer> ));
    // ----------------------- Types ------------------------------
  public:
    typedef TSegmentation Segmentation;
    typedef typename TSegmentation::SegmentComputer SegmentComputer;
    typedef typename SegmentComputer::ConstIterator ConstIterator;
    typedef typename Functor::Value Value;
    typedef typename TSpace::RealVector RealVector;
    typedef typename TSpace::Point Point;
    
    // ----------------------- Standard services ------------------------------
  public:
    //! Default constructor.
    LambdaMST3DEstimator();
    
    /**
     * Initialization.
     * @param itb begin iterator
     * @param ite end iterator
     */
    void init ( const ConstIterator& itb, const ConstIterator& ite );
    
    /**
     * @param segmentComputer - DSS segmentation algorithm
     */
    void attach ( ConstAlias<TSegmentation> segmentComputer );
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;
    
    /**
     * @param point of the underlying curve to calculate a tangent to it
     * @return tangent direction
     */
    RealVector eval ( const Point & point );
    
    /**
     * More efficient way to compute tangent directions for all points of a curve.
     * @param result back_insert_iterator to insert element to underlying container.
     */
    template < typename Container >
    void eval ( std::back_insert_iterator < Container > result );
    
    // ------------------------- Internals ------------------------------------
  protected:
    
    /**
     * @brief Accumulate partial results obtained for each point.
     * In 3D it can happen that DSSs' direction vectors over same point are opposite.
     * To avoid this problem we measure angle between segments' direction vectors and if this angle
     * is bigger than \pi/2, then one of the vectors is reversed. 
     * Finally, tangent direction is estimated and stored.
     * 
     * @tparam Container type of container which stores estimated tangent directions.
     * @param outValues partial results for each point
     * @param result back_insert_iterator over Container which stores estimated tangent directions.
     */
    template < typename Container >
    void accumulate ( std::multimap < Point, Value > & outValues, std::back_insert_iterator < Container > result );
    
    // ------------------------- Private Datas --------------------------------
  private:
    /**
     * Iterator which corresponds to the beginning of a valid range - [myBegin, myEnd)
     */
    ConstIterator myBegin;
    /**
     * Iterator which corresponds to the end of a valid range - [myBegin, myEnd)
     */
    ConstIterator myEnd;
    /**
     * Constant pointer to a curve segmentation algorithm.
     */
    const TSegmentation * dssSegments;
    /**
     * Functor which takes:
     * reference to digital straight segment - DSS, position of given point in DSS and length of DSS
     * and returns DSS's direction and the eccentricity of the point in the DSS.
     */
    Functor myFunctor;
    
  }; // end of class LambdaTangentFromDSSEstimator
  
  //-------------------------------------------------------------------------------------------
  
  // Template class LambdaMST3D
  /**
   * \brief Aim: Simplify creation of Lambda MST tangent estimator.
   * @tparam DSSSegmentationComputer tangential cover obtained by segmentation of a 2D digital curve by maximal straight segments
   * @tparam LambdaFunction @see FunctorsLambdaMST.h and @see CLambdaFunctor.h
   */
  template < typename DSSSegmentationComputer, typename lambdaFunction = functors::Lambda64Function>
  class LambdaMST3D:
  public LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
    TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, lambdaFunction> >
    {
      typedef 
      LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
      TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, lambdaFunction> > Super;
      
    public: 
      /**
       * Default Constructor.
       */
      LambdaMST3D() : Super() {}
    };
}// namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/geometry/curves/estimation/LambdaMST3D.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LAMBDAMST3D_h

#undef LAMBDAMST3D_RECURSES
#endif // else defined(LAMBDAMST3D_RECURSES)
