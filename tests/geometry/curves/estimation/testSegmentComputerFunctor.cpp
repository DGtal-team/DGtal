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
 * @file testSegmentComputerFunctor.cpp
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/28

 * Functions for testing classes in SegmentComputerFunctor.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"


#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/estimation/SegmentComputerFunctor.h"

#include "DGtal/io/boards/Board2D.h"


#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing classes in SegmentComputerFunctor.h.
///////////////////////////////////////////////////////////////////////////////


/**
 * TangentFromDSSFunctor
 *
 */
template<typename DSSComputer>
bool testTangentFromDSS(
     const typename DSSComputer::ConstIterator& begin,
     const typename DSSComputer::ConstIterator& end  )
{


  trace.info() << "feeding segment computer " << endl;

  typename DSSComputer::ConstIterator i = begin;
  DSSComputer dss;  

  if (i != end) {
    dss.init(i);
    ++i;
    while ( (i!=end)
          &&(dss.extendForward(i)) ) {
      ++i;
    }
  }

  trace.info() << dss << endl;
  trace.info() << endl;

  trace.info() << "building and using the functor " << endl;

  //default constructor
  TangentAngleFromDSSFunctor<DSSComputer> f; 
  //call
  double v1 = f(*begin,dss); 
  double v2 = std::atan2((double)dss.getA(),(double)dss.getB());
  trace.info() << "Tangent orientation : " << v1 << " == " << v2 << endl;

  return (v1 == v2);
}



///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class GridCurve" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  //types
  typedef PointVector<2,int> Point;
  typedef std::vector<Point> Range;
  typedef Range::iterator ConstIterator;
  typedef ArithmeticalDSS<ConstIterator,int,4> DSS4;  
  typedef ArithmeticalDSS<ConstIterator,int,8> DSS8;  

  //input points
  Range curve4;
  curve4.push_back(Point(0,0));
  curve4.push_back(Point(1,0));
  curve4.push_back(Point(1,1));
  curve4.push_back(Point(2,1));
  curve4.push_back(Point(3,1));
  curve4.push_back(Point(3,2));
  curve4.push_back(Point(4,2));
  curve4.push_back(Point(5,2));
  curve4.push_back(Point(6,2));
  curve4.push_back(Point(6,3));
  curve4.push_back(Point(7,3));

  Range curve8;
  curve8.push_back(Point(0,0));
  curve8.push_back(Point(1,1));
  curve8.push_back(Point(2,1));
  curve8.push_back(Point(3,2));
  curve8.push_back(Point(4,2));
  curve8.push_back(Point(5,2));
  curve8.push_back(Point(6,3));
  curve8.push_back(Point(7,3));
  curve8.push_back(Point(8,4));
  curve8.push_back(Point(9,4));
  curve8.push_back(Point(10,5));

  //tests
  bool res = testTangentFromDSS<DSS4>(curve4.begin(), curve4.end())
          && testTangentFromDSS<DSS8>(curve8.begin(), curve8.end())
//add test for other functors
;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
