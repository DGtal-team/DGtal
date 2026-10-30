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

/**
 * @file testLambdaMSTAxisLabeller.cpp
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2026/10/28
 *
 * Functions for testing class LambdaMSTAxisLabeller.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>
#include "DGtalCatch.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/parametric/EllipticHelix.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"
#include "DGtal/geometry/curves/estimation/LambdaMSTAxisLabeller.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

namespace
{
  typedef EllipticHelix < Space > Helix;
  typedef NaiveParametricCurveDigitizer3D < Helix > Digitizer;
  typedef Digitizer::DigitalCurve Curve;
  typedef Curve::const_iterator ConstIterator;
  typedef LambdaMSTAxisLabeller < ConstIterator > Labeller;

  /// Every step of [b, e) changes the coordinate along axis by the same +1 or -1.
  bool functional ( ConstIterator b, ConstIterator e, Dimension axis )
  {
    if ( std::distance ( b, e ) < 2 )
      return true;
    const int step = (*( b + 1 ))[ axis ] - (*b)[ axis ];
    if ( step != 1 && step != -1 )
      return false;
    for ( ConstIterator it = b + 1; it != e; ++it )
      if ( (*it)[ axis ] - (*( it - 1 ))[ axis ] != step )
        return false;
    return true;
  }

  typedef PointVector < 2, int > Point2d;
  typedef std::vector < Point2d > Curve2d;
  typedef ArithmeticalDSSComputer < Curve2d::const_iterator, int, 8 > DSSComputer2d;

  /// Number of points of the longest DSS of c starting at position i.
  std::size_t longestDSSFrom ( const Curve2d & c, std::size_t i )
  {
    DSSComputer2d dss;
    dss.init ( c.cbegin ( ) + i );
    while ( dss.end ( ) != c.cend ( ) && dss.extendFront ( ) )
      ;
    return (std::size_t) std::distance ( dss.begin ( ), dss.end ( ) );
  }

  /**
   * Computed without SaturatedSegmentation: the number of points of the
   * shortest 2D maximal segment of at least two points meeting the run, over
   * the projections of the run's context onto the two coordinate planes
   * containing the run's axis; 0 if there is none. With e(i) the end of the
   * longest DSS starting at i, [i, e(i)) is maximal iff i == 0 or
   * e(i - 1) < e(i).
   */
  unsigned int shortestMaximal2DDSS ( const Labeller::Run & run )
  {
    const std::size_t n = (std::size_t) std::distance ( run.contextBegin, run.contextEnd );
    const std::size_t runBegin = (std::size_t) std::distance ( run.contextBegin, run.begin );
    const std::size_t runEnd = (std::size_t) std::distance ( run.contextBegin, run.end );
    unsigned int shortest = 0;
    for ( Dimension other = 0; other < 3; ++other )
    {
      if ( other == run.axis )
        continue;
      const Dimension a = std::min ( run.axis, other ), b = std::max ( run.axis, other );
      Curve2d c;
      for ( ConstIterator it = run.contextBegin; it != run.contextEnd; ++it )
        c.push_back ( Point2d ( (*it)[ a ], (*it)[ b ] ) );
      std::size_t previousEnd = 0;
      for ( std::size_t i = 0; i < n; ++i )
      {
        const std::size_t e = i + longestDSSFrom ( c, i );
        const bool maximal = ( i == 0 || previousEnd < e );
        previousEnd = e;
        if ( ! maximal || e - i < 2 || e <= runBegin || i >= runEnd )
          continue;
        if ( shortest == 0 || e - i < shortest )
          shortest = (unsigned int) ( e - i );
      }
    }
    return shortest;
  }
}

TEST_CASE( "LambdaMSTAxisLabeller on a digitized elliptic helix" )
{
  // An elliptic helix whose main axis changes several times along one turn.
  Helix helix ( 120, 80, 20 );
  Digitizer digitizer;
  digitizer.attach ( &helix );
  digitizer.init ( 0., Helix::getPeriod ( ), 0.0001 );
  Curve curve;
  Digitizer::MetaData metaData;
  digitizer.digitize ( std::back_inserter ( curve ), std::back_inserter ( metaData ) );
  REQUIRE ( curve.size ( ) > 100 );
  REQUIRE ( metaData.size ( ) == curve.size ( ) );

  Labeller labeller;
  labeller.init ( curve.cbegin ( ), curve.cend ( ) );
  REQUIRE ( labeller.isValid ( ) );

  SECTION( "Every point gets an axis" )
  {
    REQUIRE ( labeller.axes ( ).size ( ) == curve.size ( ) );
    for ( Dimension axis : labeller.axes ( ) )
      REQUIRE ( axis < 3 );
  }

  SECTION( "The runs cover the curve in order, each functional along its axis inside a functional context" )
  {
    const std::vector < Labeller::Run > & runs = labeller.runs ( );
    REQUIRE ( runs.size ( ) > 1 );
    ConstIterator expected = curve.cbegin ( );
    for ( const Labeller::Run & run : runs )
    {
      REQUIRE ( run.begin == expected );
      REQUIRE ( run.begin < run.end );
      for ( ConstIterator it = run.begin; it != run.end; ++it )
        REQUIRE ( labeller.axes ( )[ it - curve.cbegin ( ) ] == run.axis );
      REQUIRE ( functional ( run.begin, run.end, run.axis ) );
      REQUIRE ( run.contextBegin <= run.begin );
      REQUIRE ( run.end <= run.contextEnd );
      REQUIRE ( functional ( run.contextBegin, run.contextEnd, run.axis ) );
      expected = run.end;
    }
    REQUIRE ( expected == curve.cend ( ) );
  }

  SECTION( "The contexts are maximal" )
  {
    for ( const Labeller::Run & run : labeller.runs ( ) )
    {
      if ( std::distance ( run.contextBegin, run.contextEnd ) < 2 )
        continue;
      if ( run.contextBegin != curve.cbegin ( ) )
        REQUIRE ( ! functional ( run.contextBegin - 1, run.contextEnd, run.axis ) );
      if ( run.contextEnd != curve.cend ( ) )
        REQUIRE ( ! functional ( run.contextBegin, run.contextEnd + 1, run.axis ) );
    }
  }

  SECTION( "The shortest 2D maximal segment, per run and over all runs" )
  {
    // Each run's length matches the one computed independently, and the
    // length over the curve is the smallest of them.
    unsigned int smallest = 0;
    for ( const Labeller::Run & run : labeller.runs ( ) )
    {
      REQUIRE ( run.shortest2DDSSLength == shortestMaximal2DDSS ( run ) );
      if ( run.shortest2DDSSLength != 0 && ( smallest == 0 || run.shortest2DDSSLength < smallest ) )
        smallest = run.shortest2DDSSLength;
    }
    REQUIRE ( smallest >= 2 );
    REQUIRE ( labeller.shortest2DDSSLength ( ) == smallest );
  }

  SECTION( "evalByRuns gives one unit tangent per point, close to the tangent of the helix" )
  {
    LambdaMST3DBy2D < ConstIterator > lmst;
    std::vector < RealVector > tangents;
    labeller.evalByRuns ( lmst, std::back_inserter ( tangents ) );
    REQUIRE ( tangents.size ( ) == curve.size ( ) );

    double sum = 0.;
    for ( std::size_t i = 0; i < curve.size ( ); ++i )
    {
      REQUIRE ( std::abs ( tangents[ i ].norm ( ) - 1. ) < 1e-9 );
      RealVector truth = helix.xp ( metaData[ i ].first );
      truth /= truth.norm ( );
      sum += ( tangents[ i ] - truth ).norm ( );
    }
    const double mean = sum / (double) curve.size ( );
    trace.info ( ) << labeller << ", mean error " << mean << std::endl;
    // Measured: about 0.021 for this helix.
    REQUIRE ( mean < 0.04 );
  }
}

TEST_CASE( "LambdaMSTAxisLabeller on a curve of one point" )
{
  std::vector < Point > curve { Point ( 1, 2, 3 ) };
  LambdaMSTAxisLabeller < std::vector < Point >::const_iterator > labeller;
  labeller.init ( curve.cbegin ( ), curve.cend ( ) );
  REQUIRE ( labeller.axes ( ).size ( ) == 1 );
  REQUIRE ( labeller.runs ( ).size ( ) == 1 );
  REQUIRE ( labeller.runs ( )[ 0 ].shortest2DDSSLength == 0 );
  REQUIRE ( labeller.shortest2DDSSLength ( ) == 0 );
}

///////////////////////////////////////////////////////////////////////////////
