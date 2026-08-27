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
 * @file VisibilityLatticeEstimator.h
 * @author Romain Negro (\c romain.negro@univ-smb.fr )
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2026/08/27
 *
 * Header file for module VisibilityLatticeEstimator.ih
 *
 * This file is part of the DGtal library.
 */

#if defined(VisibilityLatticeEstimator_RECURSES)
#error Recursive header files inclusion detected in VisibilityLatticeEstimator.h
#else // defined(VisibilityLatticeEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VisibilityLatticeEstimator_RECURSES

#if !defined VisibilityLatticeEstimator_h
/** Prevents repeated inclusion of headers. */
#define VisibilityLatticeEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cmath>
#include <algorithm>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Eigenvalues>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/LatticeSetByIntervals.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /**
   * Aim: Computes visibility relations and visibility-based normal vectors
   * from pointel sets by lattice matching.
   *
   * @tparam TSpace a model of concepts::CSpace.
   */
  template <typename TSpace>
  class VisibilityLatticeEstimator
  {
  public:
    typedef VisibilityLatticeEstimator<TSpace> Self;
    typedef TSpace Space;
    typedef typename Space::Integer Integer;
    typedef typename Space::Point Point;
    typedef typename Space::Vector IntegerVector;
    typedef std::vector< IntegerVector > IntegerVectors;
    typedef typename Space::RealPoint RealPoint;
    typedef typename Space::RealVector RealVector;
    typedef typename RealVector::Component Scalar;
    typedef std::vector< RealVector > RealVectors;
    typedef LatticeSetByIntervals<Space> LatticeSet;
    typedef typename LatticeSet::Interval Interval;
    typedef typename LatticeSet::Intervals Intervals;

    class Visibility
    {
    public:
      Dimension mainAxis{};
      std::vector< bool > visibles;
      std::size_t vectorsSize{};
      std::size_t pointsSize{};
      std::map< Point, std::size_t > pointIdxs;
      std::map< IntegerVector, std::size_t > vectorIdxs;

      Visibility() = default;

      Visibility( Dimension axis,
                  const IntegerVectors& vectors,
                  const std::vector< Point >& points )
      {
        reset( axis, vectors, points );
      }

      std::size_t getVectorIdx( const IntegerVector& v ) const
      {
        const auto it = vectorIdxs.find( v );
        if ( it != vectorIdxs.end() ) return it->second;
        return vectorsSize;
      }

      std::size_t getPointIdx( const Point& p ) const
      {
        const auto it = pointIdxs.find( p );
        if ( it != pointIdxs.end() ) return it->second;
        return pointsSize;
      }

      bool isVisible( const Point& p1, const Point& p2 ) const
      {
        if ( p1 == p2 ) return true;
        if ( !isPointLowerThan( p1, p2 ) ) return isVisible( p2, p1 );
        IntegerVector v = p2 - p1;
        v = normalizeVector( v );
        const auto vIdx = getVectorIdx( v );
        if ( vIdx == vectorsSize ) return false;
        for ( IntegerVector p = p1; p != p2; p += v )
        {
          const auto pointIdx = getPointIdx( p );
          if ( pointIdx == pointsSize ) return false;
          if ( !visibles[ vIdx * pointsSize + pointIdx ] ) return false;
        }
        return true;
      }

      void setVisible( const Point& p1, const Point& p2 )
      {
        if ( p1 == p2 ) return;
        if ( !isPointLowerThan( p1, p2 ) )
        {
          setVisible( p2, p1 );
          return;
        }
        IntegerVector v = p2 - p1;
        v = normalizeVector( v );
        const auto vIdx = getVectorIdx( v );
        if ( vIdx == vectorsSize ) return;
        for ( IntegerVector p = p1; p != p2; p += v )
        {
          const auto pointIdx = getPointIdx( p );
          if ( pointIdx == pointsSize ) continue;
          visibles[ vIdx * pointsSize + pointIdx ] = true;
        }
      }

      void set( const Point& offset,
                const Intervals& value,
                const std::size_t vectorIdx )
      {
        Point p = offset / 2;
        for ( const auto& interval : value.data() )
        {
          for ( Integer i = interval.first / 2; i <= interval.second / 2; ++i )
          {
            p[ mainAxis ] = i;
            const auto pointIdx = getPointIdx( p );
            if ( pointIdx == pointsSize ) continue;
            visibles[ vectorIdx * pointsSize + pointIdx ] = true;
          }
        }
      }

      bool empty() const
      {
        return std::all_of( visibles.cbegin(), visibles.cend(),
                            [] ( bool v ) { return !v; } );
      }

      void reset( Dimension axis,
                  const IntegerVectors& vectors,
                  const std::vector< Point >& points )
      {
        mainAxis = axis;
        vectorsSize = vectors.size();
        pointsSize = points.size();
        visibles = std::vector< bool >( vectorsSize * pointsSize, false );
        pointIdxs.clear();
        vectorIdxs.clear();
        for ( std::size_t i = 0; i < pointsSize; ++i )
          pointIdxs[ points[ i ] ] = i;
        for ( std::size_t i = 0; i < vectorsSize; ++i )
          vectorIdxs[ vectors[ i ] ] = i;
      }

    private:
      static bool isPointLowerThan( const Point& p1, const Point& p2 );
      static Integer primitiveVectorDivisor( const IntegerVector& v );
      static IntegerVector normalizeVector( IntegerVector v );
    };

    enum class VisibilityKernel
    {
      Gaussian,
      Ring,
      None
    };

    enum class VisibilityCenter
    {
      Centroid,
      Itself
    };

    static int resolveVisibilityRadius( int radius, Scalar gridstep );

    static Visibility getVisibility( const std::vector< Point >& pointels,
                                     int nStar,
                                     int radius,
                                     int verbose = 0 );

    static Visibility getVisibilityNoOpenMP( const std::vector< Point >& pointels,
                                             int nStar,
                                             int radius,
                                             int verbose = 0 );

#if defined(DGTAL_WITH_OPENMP) && !defined(WIN32)
    static Visibility getVisibilityOpenMP( const std::vector< Point >& pointels,
                                           int nStar,
                                           int radius,
                                           int verbose = 0 );
#endif

    static VisibilityKernel visibilityKernelFromString( const std::string& kernel );
    static VisibilityCenter visibilityCenterFromString( const std::string& center );

    template <typename TVisibilityPredicate>
    static RealVectors getVisibilityNormalVectors( const std::vector<Point>& pointels,
                                                   const TVisibilityPredicate& isVisible,
                                                   const Scalar sigma,
                                                   const Scalar neighborhood,
                                                   const bool autoLimiter,
                                                   const Scalar autoLimiterRatio,
                                                   const VisibilityCenter centerChoice,
                                                   const VisibilityKernel kernelChoice,
                                                   const int verbose = 0 );

    template <typename TVisibilityPredicate>
    static RealVectors getVisibilityNormalVectorsNoOpenMP( const std::vector<Point>& pointels,
                                                           const TVisibilityPredicate& isVisible,
                                                           const Scalar sigma,
                                                           const Scalar neighborhood,
                                                           const bool autoLimiter,
                                                           const Scalar autoLimiterRatio,
                                                           const VisibilityCenter centerChoice,
                                                           const VisibilityKernel kernelChoice,
                                                           const int verbose = 0 );

#if defined(DGTAL_WITH_OPENMP) && !defined(WIN32)
    template <typename TVisibilityPredicate>
    static RealVectors getVisibilityNormalVectorsOpenMP( const std::vector<Point>& pointels,
                                                         const TVisibilityPredicate& isVisible,
                                                         const Scalar sigma,
                                                         const Scalar neighborhood,
                                                         const bool autoLimiter,
                                                         const Scalar autoLimiterRatio,
                                                         const VisibilityCenter centerChoice,
                                                         const VisibilityKernel kernelChoice,
                                                         const int verbose = 0 );
#endif

    static RealVectors getNormalVisibility( const std::vector< Point >& pointels,
                                            const Visibility& visibility,
                                            const Scalar sigma,
                                            const Scalar neighborhood,
                                            const bool autoLimiter,
                                            const Scalar autoLimiterRatio,
                                            const VisibilityCenter centerChoice,
                                            const VisibilityKernel kernelChoice,
                                            const int verbose = 0 );

  private:
    static Integer primitiveVectorDivisor( const IntegerVector& v );
    static bool hasPositiveFirstNonZeroCoordinate( const IntegerVector& v );
    static bool isZeroVector( const IntegerVector& v );
    static void appendVisibilityVectors( IntegerVectors& vectors, IntegerVector& candidate, Dimension d, int radius );
    static IntegerVectors getVisibilityVectors( int radius );
    static Dimension getLargeAxis( const std::vector< Point >& pointels );
    static std::pair< Point, Point > getPointBounds( const std::vector< Point >& pointels );
    static std::vector< Dimension > getOrthogonalDimensions( Dimension axis );
    static Intervals checkInterval( const Interval& toCheck, const Intervals& figIntervals );
    static Intervals intersectIntervals( const Intervals& l1, const Intervals& l2 );
    static Intervals matchVector( Intervals current, const Intervals& vectorIntervals, const Intervals& figIntervals );
    static LatticeSet getLatticeVector( const IntegerVector& segment, Dimension axis );
    static LatticeSet getCoverOfShape( const LatticeSet& shapeLattice );
    static LatticeSet getNStarOfShape( const std::vector< Point >& pointels, int nStar, Dimension axis );
    static typename LatticeSet::Container extractLatticeIntervals( const LatticeSet& lattice );
    static std::vector< Point > getLatticePointsFromFigure( const typename LatticeSet::Container& figLattices,
                                                            Dimension axis );
    static void processVisibilitySegment( Visibility& vis,
                                          std::size_t segmentIdx,
                                          const IntegerVector& segment,
                                          const typename LatticeSet::Container& figLattices,
                                          const Point& pmin,
                                          const Point& pmax,
                                          const std::vector< Dimension >& orthDims );
    static void appendShapeCoverNeighbors( LatticeSet& result,
                                           const Point& p,
                                           const Intervals& dilatedIntervals,
                                           const std::vector< Dimension >& orthDims,
                                           std::size_t dimIdx,
                                           std::vector< Integer >& delta );
    static void processVisibilityShifts( Visibility& vis,
                                         std::size_t segmentIdx,
                                         const typename LatticeSet::Container& latticeVector,
                                         const typename LatticeSet::Container& figLattices,
                                         const Point& pmin,
                                         const Point& pmax,
                                         const std::vector< Dimension >& orthDims,
                                         const std::vector< Integer >& minT,
                                         const std::vector< Integer >& maxT,
                                         std::vector< Integer >& shifts,
                                         std::size_t dimIdx );
    static Scalar visibilityWeight( const Scalar squaredDistance,
                                    const VisibilityKernel kernelChoice,
                                    const Scalar minus2SigmaSquare );

    template <typename TVisibilityPredicate>
    static RealVector visibilityNormalAt( const std::vector<Point>& pointels,
                                          std::size_t pidx,
                                          const TVisibilityPredicate& isVisible,
                                          Scalar neighborhood2,
                                          bool autoLimiter,
                                          Scalar autoLimiterRatio,
                                          VisibilityCenter centerChoice,
                                          VisibilityKernel kernelChoice,
                                          Scalar minus2SigmaSquare );
  };
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/VisibilityLatticeEstimator.ih"

#endif // !defined VisibilityLatticeEstimator_h

#undef VisibilityLatticeEstimator_RECURSES
#endif // else defined(VisibilityLatticeEstimator_RECURSES)
