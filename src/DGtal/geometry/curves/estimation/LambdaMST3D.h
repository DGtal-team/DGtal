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
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"
#include "DGtal/geometry/curves/CForwardSegmentComputer.h"
#include "DGtal/geometry/curves/estimation/CLMSTTangentFromDSS.h"
#include "DGtal/geometry/curves/estimation/CLMSTDSSFilter.h"

namespace DGtal {
  /**
   * Aim: Implement 3D Lambda MST tangent estimators.
   *
   * The tangent at a point is the lambda-weighted average of the directions
   * of the maximal segments in its pencil. Points are identified by their
   * position along the curve, never by their coordinates, so a curve that
   * visits the same lattice point twice gets one estimate per visit.
   *
   * @tparam TSpace model of CSpace
   * @tparam TSegmentation tangential cover obtained by a segmentation of a
   * 3D digital curve by maximal straight segments
   * @tparam Functor model of CLMSTTangentFromDSS
   * @tparam DSSFilter decides which maximal segments enter a point's pencil,
   * the set of segments whose directions are averaged into its tangent (see
   * CLMSTDSSFilter): operator()(dss) is true for a segment that is too short
   * to enter any pencil, and reach() is how far, in points along the curve,
   * a kept segment may lie from a point it does not contain and still enter
   * its pencil. DSSMuteFilter (no segment removed, reach 0) gives the plain
   * 3D lambda-MST; DSSLengthLessEqualFilter(T) removes segments shorter than
   * T points and lets the others reach T points beyond their ends.
   */
  template < typename TSpace, typename TSegmentation, typename Functor, typename DSSFilter >
  class LambdaMST3DEstimator
  {
  public: 
    //Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace < TSpace > ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 3 ));
    BOOST_CONCEPT_ASSERT(( concepts::CLMSTTangentFromDSS < Functor > ));
    BOOST_CONCEPT_ASSERT(( concepts::CLMSTDSSFilter < DSSFilter > ));
    BOOST_CONCEPT_ASSERT(( concepts::CForwardSegmentComputer< typename TSegmentation::SegmentComputer > ));
    // ----------------------- Types ------------------------------
  public:
    /// Tangential cover algorithm
    typedef TSegmentation Segmentation;
    /// Curve segmentation algorithm
    typedef typename TSegmentation::SegmentComputer SegmentComputer;
    /// Type of iterator, at least readable and forward
    typedef typename SegmentComputer::ConstIterator ConstIterator;
    /// Type returned by model of CLMSTTangentFromDSS
    typedef typename Functor::Value Value;
    /// Type of 3d real vector
    typedef typename TSpace::RealVector RealVector;
    /// Type of 3d real point
    typedef typename TSpace::Point Point;
    
    // ----------------------- Standard services ------------------------------
  public:
    //! Default constructor.
    LambdaMST3DEstimator ( );
    
    /**
     * Initialization.
     * @param itb begin iterator
     * @param ite end iterator
     */
    void init ( ConstIterator itb, ConstIterator ite );
    
    /**
     * @param segmentComputer - DSS segmentation algorithm
     */
    void attach ( Alias<TSegmentation> segmentComputer );
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid ( ) const;
    
    /**
     * Tangent directions of the points in [itb, ite), one per curve position.
     *
     * The segmentation must be attached over the same range [myBegin, myEnd)
     * given to init(). Its maximal segments are maximal in that whole range,
     * so a segment is never cut at itb or ite: it contributes to every point
     * of [itb, ite) whose pencil it enters, and the pencils are complete at
     * both ends of the range. A point is identified by its position, so the
     * estimate for a single point at position it is eval(it, it + 1, ...).
     *
     * @pre ConstIterator is random access and myBegin <= itb < ite <= myEnd.
     *
     * @tparam OutputIterator writable iterator.
     * @param itb begin iterator
     * @param ite end iterator
     * @param result writable iterator over a container which stores estimated tangent directions.
     */
    template <typename OutputIterator>
    OutputIterator eval ( ConstIterator itb, ConstIterator ite, OutputIterator result );

    /**
     *
     * @return the internal dss filter
     */
    DSSFilter & getDSSFilter ( );

    /**
     * Positions, in the last eval(), of the points that received no
     * contribution from any segment, and therefore a zero tangent.
     *
     * This can only happen with a filter that removes segments: a point gets
     * no contribution when no kept segment contains it or reaches it. With
     * DSSLengthLessEqualFilter(l / 2), l being the number of points of the
     * shortest 2D maximal segment of the curve's projections onto the two
     * coordinate planes containing its main axis, it cannot happen on a curve
     * that is monotone along that axis. With DSSMuteFilter the list is always
     * empty.
     *
     * Positions are counted from the @a itb passed to eval(), and the list is
     * cleared by every call to eval().
     *
     * @return the positions, in increasing order
     */
    const std::vector< std::size_t > & uncoveredPoints ( ) const;
    
    // ------------------------- Internals ------------------------------------
  protected:


    /**
     * @brief Accumulate partial results obtained for each point.
     *
     * Every contribution is already oriented along its own segment (last
     * point minus first), so the contributions at a point are summed as they
     * are and the tangent is their weighted mean.
     *
     * outValues is indexed by curve position (one list per point, in the
     * order the covering DSSes were visited), not by point value: keying on
     * coordinates would merge the contributions of two different positions
     * that share the same lattice point.
     *
     * @tparam OutputIterator writable iterator.
     * @param outValues per-position vote lists
     * @param result writable iterator over a container which stores estimated tangent directions.
     */
    template <typename OutputIterator>
    void accumulate ( std::vector< std::vector< Value > > & outValues, OutputIterator & result );

    /**
     * The contributions of one maximal segment to the points of [itb, ite)
     * whose pencil it enters, the points it contains, and the points within
     * the filter's reach of one of its ends, appended to outValues.
     */
    void contribute ( const SegmentComputer & dss, ConstIterator itb, ConstIterator ite,
                      std::vector< std::vector< Value > > & outValues ) const;

    // ------------------------- Private Data --------------------------------
  private:
    /**
     * Iterator which corresponds to the beginning of a valid range: [myBegin, myEnd)
     */
    ConstIterator myBegin;
    /**
     * Iterator which corresponds to the end of a valid range: [myBegin, myEnd)
     */
    ConstIterator myEnd;
    /**
     * Pointer to a curve segmentation algorithm.
     */
    TSegmentation * dssSegments;
    /**
     * Functor which takes:
     * reference to digital straight segment - DSS, position of given point in DSS and length of DSS
     * and returns DSS's direction and the eccentricity of the point in the DSS.
     */
    Functor myFunctor;

    DSSFilter myDSSFilter;

    /**
     * Positions, relative to the last eval()'s itb, of the points no DSS
     * ended up contributing to. See uncoveredPoints().
     */
    std::vector< std::size_t > myUncovered;

  }; // end of class LambdaMST3DEstimator
  
  //-------------------------------------------------------------------------------------------
  
  // Template class LambdaMST3D
  /**
   * \brief Aim: Simplify creation of Lambda MST tangent estimator.
   * @tparam DSSSegmentationComputer tangential cover obtained by segmentation of a 3D digital curve by maximal straight segments
   * @tparam LambdaFunction Lambda functor @see FunctorsLambdaMST.h
   * @tparam DSSFilter which maximal segments enter a point's pencil: DSSMuteFilter (default, plain 3D lambda-MST) or DSSLengthLessEqualFilter (short segments removed, the others reaching beyond their ends)
   */
  template < typename DSSSegmentationComputer, typename LambdaFunction = functors::Lambda64Function,
             typename DSSFilter = DSSMuteFilter < typename DSSSegmentationComputer::SegmentComputer > >
  class LambdaMST3D:
  public LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
    TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, LambdaFunction >, DSSFilter >
    {
      typedef 
      LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
      TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, LambdaFunction >, DSSFilter > Super;
      
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
