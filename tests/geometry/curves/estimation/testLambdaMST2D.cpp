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
 * @file testLambdaMST2D.cpp
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/03
 *
 * Functions for testing class LambdaMST2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/estimation/LambdaMST2D.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LambdaMST2D.
///////////////////////////////////////////////////////////////////////////////

/**
 * Each point evaluated alone, on [it, it + 1), must get exactly the value it
 * gets when the whole range is evaluated: the pencils are complete at the
 * ends of any sub-range.
 */
template < typename LMST, typename Range >
bool positionsMatchWholeRange ( LMST & lmst, const Range & curve )
{
  std::vector < RealVector > all;
  lmst.eval ( curve.begin(), curve.end(), std::back_inserter ( all ) );
  if ( all.size() != curve.size() )
    return false;
  for ( auto it = curve.begin(); it != curve.end(); ++it )
  {
    std::vector < RealVector > one;
    lmst.eval ( it, it + 1, std::back_inserter ( one ) );
    if ( one.size() != 1 || ( one[0] - all[ it - curve.begin() ] ).norm() > 1e-12 )
      return false;
  }
  return true;
}

class testLambdaMST2D
{
  typedef vector < Point > Range;
  typedef Range::const_iterator ConstIterator;
  typedef ArithmeticalDSSComputer < ConstIterator, int, 8 > SegmentComputer;
  typedef SaturatedSegmentation<SegmentComputer> Segmentation;
private:
  Range curve;
public:
  testLambdaMST2D ()
  {
    fstream inputStream;
    inputStream.open ( (testPath + "samples/sinus2D4.dat").c_str(), ios::in );
    try {
      curve = PointListReader<Point>::getPointsFromInputStream ( inputStream );
      if ( curve.size() == 0) throw IOException();
    } catch (DGtal::IOException & ioe) {
      trace.error() << "Size is null." << endl;
    }
  }
  bool lambda64ByPoint ()
  {
    Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    return positionsMatchWholeRange ( lmst64, curve );
  }

  /**
   * A curve that visits the lattice point (6, 0) twice, first going right,
   * then going down: each visit must get its own estimate.
   */
  bool lambda64RepeatedPoint ()
  {
    Range loop;
    for ( int x = 0; x <= 10; ++x ) loop.push_back ( Point ( x, 0 ) );
    for ( int y = 1; y <= 4; ++y ) loop.push_back ( Point ( 10 + y / 2, y ) );
    for ( int x = 11; x >= 7; --x ) loop.push_back ( Point ( x, 5 ) );
    for ( int y = 4; y >= 1; --y ) loop.push_back ( Point ( 6 + ( y > 2 ), y ) );
    for ( int y = 0; y >= -4; --y ) loop.push_back ( Point ( 6, y ) );
    const std::size_t first = 6, second = 24; // the two positions of (6, 0)
    if ( loop[ first ] != Point ( 6, 0 ) || loop[ second ] != Point ( 6, 0 ) )
      return false;

    Segmentation segmenter ( loop.begin(), loop.end(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( loop.begin(), loop.end() );
    std::vector < RealVector > all;
    lmst64.eval ( loop.begin(), loop.end(), std::back_inserter ( all ) );
    trace.info() << "(6, 0) first visit: " << all[ first ]
                 << ", second visit: " << all[ second ] << std::endl;
    // Going right on a horizontal run, then going down on a vertical one.
    return positionsMatchWholeRange ( lmst64, loop )
        && ( all[ first ] - RealVector ( 1., 0. ) ).norm() < 1e-12
        && all[ second ][ 1 ] < -0.9;
  }
  bool lambda64()
  {
    Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    std::vector < RealVector > tangent;
    lmst64.eval < back_insert_iterator< vector < RealVector > > > ( curve.begin(), curve.end(),  back_inserter ( tangent ) );
    return true;
  }
};


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int , char**  )
{
    bool res = true;
    testLambdaMST2D testLMST;
    trace.beginBlock ( "Testing LambdaMST2D" );
        trace.beginBlock ( "Testing point only calculation" );
          res &= testLMST.lambda64ByPoint();
        trace.endBlock();
        trace.beginBlock ( "Testing a curve visiting a lattice point twice" );
          res &= testLMST.lambda64RepeatedPoint();
        trace.endBlock();
        trace.beginBlock ( "Testing calculation for whole curve" );
           res &= testLMST.lambda64();
        trace.endBlock();
    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
