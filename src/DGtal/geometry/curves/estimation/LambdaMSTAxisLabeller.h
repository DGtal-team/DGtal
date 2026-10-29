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
 * @file LambdaMSTAxisLabeller.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2026/10/28
 *
 * This file is part of the DGtal library.
 */

#if defined(LAMBDAMSTAXISLABELLER_RECURSES)
#error Recursive header files inclusion detected in LambdaMSTAxisLabeller.h
#else // defined(LAMBDAMSTAXISLABELLER_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LAMBDAMSTAXISLABELLER_RECURSES

#if !defined LAMBDAMSTAXISLABELLER_h
/** Prevents repeated inclusion of headers. */
#define LAMBDAMSTAXISLABELLER_h

#include <iterator>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/Naive3DDSSComputer.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

namespace DGtal
{
  /**
   * Description of template class 'LambdaMSTAxisLabeller' <p>
   * \brief Aim: Cut an open 3D digital curve into parts functional along one
   * axis, as needed by LambdaMST3DBy2D, and compute the length threshold of
   * DSSLengthLessEqualFilter for LambdaMST3D.
   *
   * LambdaMST3DBy2D estimates tangents on a part of a curve that is
   * functional along one axis (its coordinate along that axis changes by
   * exactly one between consecutive points), and has to be told which axis.
   * A general 3D digital curve changes main axis along the way. This class
   * provides, from the digital curve alone:
   *
   * - axes(): the main axis of each point, defined as the main axis of the
   *   longest 3D maximal segment covering it (on a tie, the segment that
   *   starts first along the curve);
   * - runs(): the maximal runs of consecutive points with the same axis, cut
   *   further where the curve is not functional along that axis. Together
   *   they cover the curve, in order. Each run comes with its context: the
   *   largest range around it that is still functional along its axis, over
   *   which the 2D maximal segments of the projections are computed;
   * - shortest2DDSSLength(): l, the number of points of the shortest 2D
   *   maximal segment of the two projections of the runs onto the coordinate
   *   planes containing their axis, each projection being computed on the
   *   run's context. Run::shortest2DDSSLength gives the same length for
   *   each run alone; l is the smallest of them;
   * - evalByRuns(): the tangents of LambdaMST3DBy2D at every point of the
   *   curve, one run after the other.
   *
   * Points are identified by their position along the curve, never by their
   * coordinates, so a curve may visit the same lattice point twice.
   *
   * Typical use:
   * @code
   * typedef std::vector< Z3i::Point >::const_iterator ConstIterator;
   * LambdaMSTAxisLabeller< ConstIterator > labeller;
   * labeller.init ( curve.begin(), curve.end() );
   *
   * // Tangents of the 3D lambda-MST by 2D projections, at every point.
   * LambdaMST3DBy2D< ConstIterator > lmstBy2D;
   * std::vector< Z3i::RealVector > tangents;
   * labeller.evalByRuns ( lmstBy2D, std::back_inserter ( tangents ) );
   *
   * // Threshold of the filtered 3D lambda-MST.
   * LambdaMST3D< Segmentation, functors::Lambda64Function,
   *              DSSLengthLessEqualFilter< SegmentComputer > > lmst;
   * lmst.getDSSFilter().init ( labeller.shortest2DDSSLength() / 2. );
   * @endcode
   *
   * @tparam TIterator random-access iterator over the points of a 3D digital
   * curve (26-connected, no two consecutive points equal)
   * @tparam TInteger integer type of the coordinates
   * @tparam connectivity connectivity of the projected 2D curves (8 or 4)
   */
  template < typename TIterator, typename TInteger = int, int connectivity = 8 >
  class LambdaMSTAxisLabeller
  {
    // ----------------------- Types ------------------------------
  public:
    /// Iterator over the points of the curve
    typedef TIterator ConstIterator;
    /// 3D maximal segment recognition
    typedef Naive3DDSSComputer< TIterator, TInteger, connectivity > SegmentComputer3D;
    /// 3D tangential cover
    typedef SaturatedSegmentation< SegmentComputer3D > Segmentation3D;
    /// 2D point of a projection
    typedef PointVector< 2, TInteger > Point2d;
    /// Projected curve
    typedef std::vector< Point2d > Curve2d;
    /// 2D maximal segment recognition
    typedef ArithmeticalDSSComputer< typename Curve2d::const_iterator, TInteger, connectivity > SegmentComputer2D;
    /// 2D tangential cover
    typedef SaturatedSegmentation< SegmentComputer2D > Segmentation2D;
    /// Projection onto a coordinate plane
    typedef functors::Projector< SpaceND< 2, TInteger > > Projector2d;

    /**
     * A part of the curve functional along one axis, and its context.
     */
    struct Run
    {
      /// first point of the run
      ConstIterator begin;
      /// past-the-end point of the run
      ConstIterator end;
      /// first point of the context, contextBegin <= begin
      ConstIterator contextBegin;
      /// past-the-end point of the context, end <= contextEnd
      ConstIterator contextEnd;
      /// the axis the run is functional along: 0 (x), 1 (y) or 2 (z)
      Dimension axis;
      /// the number of points of the shortest 2D maximal segment of at least
      /// two points of the two projections of the context that meets the
      /// run's own points; 0 if there is none (a context of one point)
      unsigned int shortest2DDSSLength;
    };

    // ----------------------- Standard services ------------------------------
  public:
    /// Default constructor.
    LambdaMSTAxisLabeller ( );

    /**
     * Computes the axes, the runs and the threshold of the curve [itb, ite).
     * @param itb begin iterator
     * @param ite end iterator
     */
    void init ( ConstIterator itb, ConstIterator ite );

    /**
     * @return 'true' if init() was given a non-empty curve.
     */
    bool isValid ( ) const;

    /**
     * @return the main axis of each point of the curve, in curve order.
     */
    const std::vector< Dimension > & axes ( ) const;

    /**
     * @return the runs of the curve, in curve order. They cover the curve
     * and do not overlap.
     */
    const std::vector< Run > & runs ( ) const;

    /**
     * @return l, the number of points of the shortest 2D maximal segment of
     * at least two points over the projections of all runs, each projection
     * computed on the run's context; 0 if there is none (a curve of one
     * point). It is the smallest non-zero Run::shortest2DDSSLength of runs().
     */
    unsigned int shortest2DDSSLength ( ) const;

    /**
     * Tangents at every point of the curve, computed run by run with a
     * LambdaMST3DBy2D estimator initialized on each run, its context and its
     * axis.
     *
     * @tparam TEstimator a LambdaMST3DBy2D (or LambdaMST3DBy2DEstimator) over
     * TIterator
     * @tparam OutputIterator writable iterator
     * @param estimator the estimator, re-initialized for every run
     * @param result where the tangents are written, one per point, in curve
     * order
     * @return the output iterator after the last tangent written
     */
    template < typename TEstimator, typename OutputIterator >
    OutputIterator evalByRuns ( TEstimator & estimator, OutputIterator result ) const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    // ------------------------- Internals ------------------------------------
  protected:
    /// The main axis of the longest maximal segment covering each point.
    void computeAxes ( );

    /// Cuts the runs of one axis where the curve is not functional along it.
    void computeRuns ( );

    /**
     * Splits [runBegin, runEnd) into subranges functional along @a axis,
     * appended to myRuns (without their context).
     */
    void appendFunctionalSubRanges ( ConstIterator runBegin, ConstIterator runEnd, Dimension axis );

    /**
     * The largest range around [runBegin, runEnd) that is still functional
     * along @a axis, in the same direction.
     */
    void functionalPart ( ConstIterator runBegin, ConstIterator runEnd, Dimension axis,
                          ConstIterator & partBegin, ConstIterator & partEnd ) const;

    /// Run::shortest2DDSSLength of every run, and l over all runs, see
    /// shortest2DDSSLength().
    void computeShortest2DDSSLength ( );

    /**
     * The two projectors of @a axis: onto the two coordinate planes
     * containing it (x: XY and XZ, y: XY and YZ, z: XZ and YZ).
     */
    static void axisProjectors ( Dimension axis, Projector2d & p1, Projector2d & p2 );

    // ------------------------- Private Data --------------------------------
  private:
    /// begin of the curve
    ConstIterator myBegin;
    /// end of the curve
    ConstIterator myEnd;
    /// main axis of each point
    std::vector< Dimension > myAxes;
    /// runs and their contexts
    std::vector< Run > myRuns;
    /// l, see shortest2DDSSLength()
    unsigned int myShortest;
  }; // end of class LambdaMSTAxisLabeller

  /**
   * Overloads 'operator<<' for displaying objects of class 'LambdaMSTAxisLabeller'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'LambdaMSTAxisLabeller' to write.
   * @return the output stream after the writing.
   */
  template < typename TIterator, typename TInteger, int connectivity >
  std::ostream &
  operator<< ( std::ostream & out, const LambdaMSTAxisLabeller< TIterator, TInteger, connectivity > & object );
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/geometry/curves/estimation/LambdaMSTAxisLabeller.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LAMBDAMSTAXISLABELLER_h

#undef LAMBDAMSTAXISLABELLER_RECURSES
#endif // else defined(LAMBDAMSTAXISLABELLER_RECURSES)
