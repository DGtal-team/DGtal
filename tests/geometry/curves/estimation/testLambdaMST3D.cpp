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
 * @file testLambdaMST3D.cpp
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/03
 *
 * Functions for testing class LambdaMST3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/geometry/curves/Naive3DDSSComputer.h"
#include "DGtal/geometry/curves/estimation/LambdaMST3D.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;
using namespace functors;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LambdaMST3D.
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

class testLambdaMST3D
{
  typedef DGtal::Z3i::Point Point;
  typedef DGtal::Z2i::Point Point2D;
  typedef std::vector < Point > Range;
  typedef std::list < Point2D > Range2D;
  typedef Range::const_iterator ConstIterator;
  typedef Range2D::const_iterator ConstIterator2D;
  typedef Naive3DDSSComputer < ConstIterator, int, 8 > SegmentComputer;
  typedef SaturatedSegmentation<SegmentComputer> Segmentation;
  typedef ArithmeticalDSSComputer < ConstIterator2D, int, 8 > SegmentComputer2D;
  typedef SaturatedSegmentation<SegmentComputer2D> Segmentation2D;
private:
  Range curve;
public:
  testLambdaMST3D ()
  {
    fstream inputStream;
    inputStream.open ( (testPath + "samples/sinus3D.dat").c_str(), std::ios::in );
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
    LambdaMST3D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    return positionsMatchWholeRange ( lmst64, curve );
  }
  bool lambda64()
  {
    Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
    LambdaMST3D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    vector < RealVector > tangent;
    lmst64.eval < back_insert_iterator< vector < RealVector > > > ( curve.begin(), curve.end(), back_insert_iterator< vector < RealVector > > ( tangent ) );
    return true;
  }

  bool lambda64Filtered()
  {
      Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
      LambdaMST3D < Segmentation, Lambda64Function, DSSLengthLessEqualFilter < SegmentComputer > > lmst64;
      lmst64.attach ( segmenter );
      lmst64.getDSSFilter ( ).init ( 1 );
      lmst64.init ( curve.begin(), curve.end() );
      return positionsMatchWholeRange ( lmst64, curve );
  }

  bool lambdaSinByPoint ()
  {
     Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
     LambdaMST3D < Segmentation, LambdaSinFromPiFunction > lmst;
     lmst.attach ( segmenter );
     lmst.init ( curve.begin(), curve.end() );
     return positionsMatchWholeRange ( lmst, curve );
 }

  bool lambdaSin()
  {
      Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
      LambdaMST3D < Segmentation, LambdaSinFromPiFunction > lmst;
      lmst.attach ( segmenter );
      lmst.init ( curve.begin(), curve.end() );
      vector < RealVector > tangent;
      lmst.eval < back_insert_iterator< vector < RealVector > > > ( curve.begin(), curve.end(), back_insert_iterator< vector < RealVector > > ( tangent ) );
      return true;
  }

  bool lambdaExpByPoint ()
  {
      Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
      LambdaMST3D < Segmentation, LambdaExponentialFunction > lmst;
      lmst.attach ( segmenter );
      lmst.init ( curve.begin(), curve.end() );
      return positionsMatchWholeRange ( lmst, curve );
  }

  bool lambdaExp()
  {
      Segmentation segmenter ( curve.begin(), curve.end(), SegmentComputer() );
      LambdaMST3D < Segmentation, LambdaExponentialFunction > lmst;
      lmst.attach ( segmenter );
      lmst.init ( curve.begin(), curve.end() );
      vector < RealVector > tangent;
      lmst.eval < back_insert_iterator< vector < RealVector > > > ( curve.begin(), curve.end(), back_insert_iterator< vector < RealVector > > ( tangent ) );
      return true;
  }
};


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int , char**  )
{
    bool res = true;
    testLambdaMST3D testLMST;
    trace.beginBlock ( "Testing LambdaMST3D" );
        trace.beginBlock ( "Testing single-position evaluation against the whole curve" );
          res &= testLMST.lambda64ByPoint();
          res &= testLMST.lambda64Filtered();
          res &= testLMST.lambdaSinByPoint();
          res &= testLMST.lambdaExpByPoint();
        trace.endBlock();
        trace.beginBlock ( "Testing calculation for whole curve" );
           res &= testLMST.lambda64();
           res &= testLMST.lambdaSin();
           res &= testLMST.lambdaExp();
        trace.endBlock();
    trace.endBlock();
    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
