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

#ifdef __GNUC__
   #ifndef NDEBUG
      #include <fenv.h>
   #endif
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LambdaMST2D.
///////////////////////////////////////////////////////////////////////////////

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
    inputStream.open ( testPath + "samples/sinus2D4.dat", ios::in );
    try {
      curve = PointListReader<Point>::getPointsFromInputStream ( inputStream );
      if ( curve.size() == 0) throw IOException();
    } catch (DGtal::IOException & ioe) {
      trace.error() << "Size is null." << endl;
    }
  }
  bool lambda64ByPoint ()
  {
    Segmentation segmenter ( curve.cbegin(), curve.cend(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    for ( unsigned int i = 0; i < curve.size(); i++ )
      lmst64.eval ( curve[i] );
    return true;
  }
  bool lambda64()
  {
    Segmentation segmenter ( curve.cbegin(), curve.cend(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    std::vector < RealVector > tangent;
    lmst64.eval < vector < RealVector > > (  back_inserter ( tangent ) );
    return true;
  }
  bool lambda64Both()
  {
    Segmentation segmenter ( curve.cbegin(), curve.cend(), SegmentComputer() );
    LambdaMST2D < Segmentation > lmst64;
    lmst64.attach ( segmenter );
    lmst64.init ( curve.begin(), curve.end() );
    vector < RealVector > tangent;
    lmst64.eval < vector < RealVector > > ( back_inserter ( tangent ) );
    for ( unsigned int i = 0; i < curve.size(); i++ )
      if ( lmst64.eval ( curve[i] ) != tangent[i] )
	return false;
      return true;
  }
};


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int , char**  )
{
#ifdef __GNUC__
   #ifndef NDEBUG
    fetestexcept ( FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW | FE_UNDERFLOW );
   #endif
#endif
    bool res = true;
    testLambdaMST2D testLMST;
    trace.beginBlock ( "Testing LambdaMST2D" );
        trace.beginBlock ( "Testing point only calculation" );
          res &= testLMST.lambda64ByPoint();
        trace.endBlock();
        trace.beginBlock ( "Testing calculation for whole curve" );
           res &= testLMST.lambda64();
        trace.endBlock();
        trace.beginBlock ( "Testing values obtined from the both methods." );
           res &= testLMST.lambda64Both();
        trace.endBlock();
    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
