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
//LICENSE-END
/**
 * @file testMostCenteredMSEstimator.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/06/28
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testMostCenteredMSEstimator <p>
 */

#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <iterator>




#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/io/boards/Board2D.h"

#include "DGtal/topology/KhalimskySpaceND.h"

#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/estimation/MostCenteredMaximalSegmentEstimator.h"


#include "ConfigTest.h"


using namespace DGtal;
using namespace std;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class MostCenteredMaximalSegmentEstimator
///////////////////////////////////////////////////////////////////////////////

/**
 * Test 
 *
 */
bool testEval(string filename)
{


  trace.info() << endl;
  trace.info() << "Reading GridCurve from " << filename << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);
  typedef KhalimskySpaceND<2> Kspace; //space
  GridCurve<Kspace> c; //building grid curve
  c.initFromVectorStream(instream);
  typedef GridCurve<Kspace >::PointsRange Range;//range
  Range r = c.getPointsRange();//building range

  trace.info() << "Building Estimator (process range as"; 
  trace.info() << ( (c.isClosed())?"closed":"open" ) << ")" << endl;

  typedef Range::ConstIterator ConstIterator;//constIterator
  typedef ArithmeticalDSS<ConstIterator,Kspace::Integer,4> SegmentComputer;//segmentComputer
  typedef TangentFromDSSFunctor<SegmentComputer> Functor; //functor
  typedef Functor::Value Value; //value
  typedef MostCenteredMaximalSegmentEstimator<SegmentComputer,Functor> Estimator;//estimator

  SegmentComputer sc;
  Functor f; 

  Estimator e(sc,f); 
  e.init(1,r.begin(),r.end(),c.isClosed());

{
  trace.info() << "Eval at one element" << endl;
  for (ConstIterator i = r.begin(); i != r.end(); ++i) {
    cout << e.eval(i) << " "; 
  }
  cout << endl;
}

{
  trace.info() << "Eval for each element between begin and end " << endl;
  vector<Value> v(r.size()); 
  e.eval(r.begin(),r.end(),v.begin());

  for (vector<Value>::iterator i = v.begin(); i != v.end(); ++i) {
    cout << *i << " "; 
  }
  cout << endl;
}

{
  if (r.size() >= 10) {
    trace.info() << "Eval for each element between begin+4 and begin+9 " << endl;
    ConstIterator it1 = r.begin();
    for (  int compteur = 0; compteur < 4; ++compteur ) ++it1;
    ConstIterator it2 = it1;
    for (  int compteur = 0; compteur < 5; ++compteur ) ++it2;

    vector<Value> v(5); 
    e.eval(it1,it2,v.begin());

    for (vector<Value>::iterator i = v.begin(); i != v.end(); ++i) {
      cout << *i << " "; 
    }
    cout << endl;
  }
}

  return true;
}

/////////////////////////////////////////////////////////////////////////
//////////////// MAIN ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  
  trace.beginBlock ( "Testing class MostCenteredMaximalSegmentEstimator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  std::string sinus2D4 = testPath + "samples/sinus2D4.dat";
  std::string square = testPath + "samples/smallSquare.dat";
  std::string dss = testPath + "samples/DSS.dat";

  bool res = testEval(sinus2D4)
            && testEval(square)
            && testEval(dss)
//other tests
;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;

}
