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
 * @file testFP.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/26
 *
 * Functions for testing class FP.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/FP.h"
#include "DGtal/io/boards/Board2D.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FP.
///////////////////////////////////////////////////////////////////////////////



/**
 * Test 
 *
 */
bool testFP(string filename)
{


  trace.info() << endl;
  trace.info() << "Reading GridCurve from " << filename << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  //range of points
  typedef int Coordinate; 
  typedef KhalimskySpaceND<2,Coordinate> Kspace; //space
  GridCurve<Kspace> c; //building grid curve
  c.initFromVectorStream(instream);
  typedef GridCurve<Kspace >::PointsRange Range;//range
  Range r = c.getPointsRange();//building range


  //faithful polyon
  trace.info() << "Building FP (process digital curve as"; 
  trace.info() << ( (c.isClosed())?"closed":"open" ) << ")" << endl;

  bool res = true; 
  if (c.isClosed())
    {
      typedef FP<Range::ConstCirculator,Coordinate,4> FP; 
      FP theFP( r.c(), r.c() );
      res = theFP.isValid();       
    }
  else 
    {
      typedef FP<Range::ConstIterator,Coordinate,4> FP; 
      FP theFP( r.begin(), r.end() );
      res = theFP.isValid(); 
    }
  return res;
 
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FP" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  std::string sinus2D4 = testPath + "samples/sinus2D4.dat";
  std::string square = testPath + "samples/smallSquare.dat";
  std::string dss = testPath + "samples/DSS.dat";

  bool res = testFP(sinus2D4)
            && testFP(square)
            && testFP(dss)
//other tests
;

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
