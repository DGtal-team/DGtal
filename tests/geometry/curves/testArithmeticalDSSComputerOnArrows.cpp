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
 * @file
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Functions for testing class ArithmeticalDSSComputerOnArrows: the
 * segments of n arrows must be the DSSs of the n+1 corresponding points.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <utility>

#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"

#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/CDynamicBidirectionalSegmentComputer.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputerOnArrows.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

using namespace std;
using namespace DGtal;

using Point = Z2i::Point;
using Vector = Z2i::Vector;
using Arrow = std::pair<Point, Vector>;
using PointIterator = std::vector<Point>::const_iterator;
using ArrowIterator = std::vector<Arrow>::const_iterator;
using PointCirculator = Circulator<PointIterator>;
using ArrowCirculator = Circulator<ArrowIterator>;

/**
 * Points of a 4-connected curve read from a Freeman chain of the test
 * samples (the last point is removed if the curve is closed).
 */
std::vector<Point> readPoints(const std::string& name, bool closed)
{
  std::fstream fst( (testPath + "samples/" + name).c_str(), std::ios::in );
  FreemanChain<int> fc( fst );
  std::vector<Point> points( fc.begin(), fc.end() );
  if ( closed ) points.pop_back();
  return points;
}

/**
 * Arrows of a curve: the i-th arrow goes from the i-th point to the
 * following one (the last arrow goes from the last point to the first
 * one for a closed curve).
 */
std::vector<Arrow> arrowsOf(const std::vector<Point>& points, bool closed)
{
  std::vector<Arrow> arrows;
  const std::size_t n = points.size();
  for (std::size_t i = 0; i+1 < n; ++i)
    arrows.push_back( { points[i], points[i+1] - points[i] } );
  if ( closed )
    arrows.push_back( { points[n-1], points[0] - points[n-1] } );
  return arrows;
}

/**
 * For each arrow, the longest segment of arrows starting at this arrow
 * must be the DSS of the corresponding points, and the longest DSS of
 * points starting at the corresponding point must have one more point.
 */
template <typename TPointIt, typename TArrowIt>
void checkLongestSegments(TPointIt pc, TArrowIt ac, std::size_t nbArrows, bool closed)
{
  using DSSComputer = ArithmeticalDSSComputer<TPointIt, int, 4>;
  using ArrowComputer = ArithmeticalDSSComputerOnArrows<TArrowIt, int>;
  for (std::size_t i = 0; i < nbArrows; ++i, ++pc, ++ac)
  {
    INFO( "arrow " << i );
    std::size_t limit = closed ? nbArrows - 1 : nbArrows - i;

    ArrowComputer s;
    s.init( ac );
    std::size_t na = 1;
    while ( (na < limit) && s.extendFront() ) ++na;

    DSSComputer t;
    t.init( pc );
    std::size_t np = 1;
    while ( (np < limit+1) && t.extendFront() ) ++np;

    CHECK( np == na + 1 );
    CHECK( s.primitive() == t.primitive() );
  }
}

/**
 * The saturated segmentations of the arrows and of the points must give
 * the same maximal DSSs.
 */
template <typename TCategory, typename TPointIt, typename TArrowIt>
void checkSegmentations(TPointIt pb, TPointIt pe, TArrowIt ab, TArrowIt ae)
{
  using DSSComputer = ArithmeticalDSSComputer<TPointIt, int, 4, TCategory>;
  using ArrowComputer = ArithmeticalDSSComputerOnArrows<TArrowIt, int, true, TCategory>;
  SaturatedSegmentation<DSSComputer> pointSegmentation( pb, pe, DSSComputer() );
  SaturatedSegmentation<ArrowComputer> arrowSegmentation( ab, ae, ArrowComputer() );
  std::vector<typename DSSComputer::Primitive> pointDSSs, arrowDSSs;
  for (auto it = pointSegmentation.begin(); it != pointSegmentation.end(); ++it)
    pointDSSs.push_back( it->primitive() );
  for (auto it = arrowSegmentation.begin(); it != arrowSegmentation.end(); ++it)
    arrowDSSs.push_back( it->primitive() );
  REQUIRE( pointDSSs.size() > 1 );
  REQUIRE( pointDSSs.size() == arrowDSSs.size() );
  bool same = ( pointDSSs == arrowDSSs );
  CHECK( same );
}

//////////////////////////////////////////////////////////////
TEST_CASE("ArithmeticalDSSComputerOnArrows concepts")
{
  using Computer = ArithmeticalDSSComputerOnArrows<ArrowIterator, int>;
  BOOST_CONCEPT_ASSERT(( concepts::CDynamicBidirectionalSegmentComputer<Computer> ));
  STATIC_REQUIRE( std::is_same_v<SegmentComputerTraits<Computer>::Category,
                                 DynamicBidirectionalSegmentComputer> );
  STATIC_REQUIRE( std::is_same_v<SegmentComputerTraits<Computer::Reverse>::Category,
                                 DynamicBidirectionalSegmentComputer> );
}

TEST_CASE("ArithmeticalDSSComputerOnArrows basic operations")
{
  // right, up, right, up, then left
  std::vector<Arrow> arrows = { { Point(0,0), Vector(1,0) }, { Point(1,0), Vector(0,1) },
                                { Point(1,1), Vector(1,0) }, { Point(2,1), Vector(0,1) },
                                { Point(2,2), Vector(-1,0) } };
  ArithmeticalDSSComputerOnArrows<ArrowIterator, int> s;
  s.init( arrows.cbegin() );
  CHECK( s.isValid() );
  CHECK( s.begin() == arrows.cbegin() );
  CHECK( s.end() == arrows.cbegin()+1 );
  CHECK( s.extendFront() );
  CHECK( s.extendFront() );
  CHECK( s.extendFront() );
  CHECK( s.end() == arrows.cbegin()+4 );
  CHECK( s.primitive() == ArithmeticalDSS<int,int,4>( Point(0,0), Point(2,2) ) );
  // a step to the left after steps to the right is not a DSS
  CHECK( ! s.isExtendableFront() );
  CHECK( ! s.extendFront() );
  CHECK( s.retractBack() );
  CHECK( s.begin() == arrows.cbegin()+1 );
  CHECK( ! s.extendFront() );
  CHECK( s.retractFront() );
  CHECK( s.end() == arrows.cbegin()+3 );
  CHECK( s.extendBack() );
  CHECK( s.begin() == arrows.cbegin() );
  // a segment of one arrow cannot be retracted
  ArithmeticalDSSComputerOnArrows<ArrowIterator, int> t;
  t.init( arrows.cbegin() );
  CHECK( ! t.retractFront() );
  CHECK( ! t.retractBack() );
}

TEST_CASE("ArithmeticalDSSComputerOnArrows versus ArithmeticalDSSComputer")
{
  SECTION("Open curve")
  {
    std::vector<Point> points = readPoints( "manche.fc", false );
    std::vector<Arrow> arrows = arrowsOf( points, false );
    checkLongestSegments( points.cbegin(), arrows.cbegin(), arrows.size(), false );
  }

  SECTION("Closed curve")
  {
    std::vector<Point> points = readPoints( "contourS.fc", true );
    std::vector<Arrow> arrows = arrowsOf( points, true );
    PointCirculator pc( points.cbegin(), points.cbegin(), points.cend() );
    ArrowCirculator ac( arrows.cbegin(), arrows.cbegin(), arrows.cend() );
    checkLongestSegments( pc, ac, arrows.size(), true );
  }
}

TEMPLATE_TEST_CASE("Saturated segmentations of arrows and points", "",
                   ForwardSegmentComputer, BidirectionalSegmentComputer,
                   DynamicSegmentComputer, DynamicBidirectionalSegmentComputer)
{
  SECTION("Open curve")
  {
    std::vector<Point> points = readPoints( "manche.fc", false );
    std::vector<Arrow> arrows = arrowsOf( points, false );
    checkSegmentations<TestType>( points.cbegin(), points.cend(),
                                  arrows.cbegin(), arrows.cend() );
  }

  SECTION("Closed curve")
  {
    std::vector<Point> points = readPoints( "contourS.fc", true );
    std::vector<Arrow> arrows = arrowsOf( points, true );
    PointCirculator pc( points.cbegin(), points.cbegin(), points.cend() );
    ArrowCirculator ac( arrows.cbegin(), arrows.cbegin(), arrows.cend() );
    checkSegmentations<TestType>( pc, pc, ac, ac );
  }
}

/** @ingroup Tests **/
