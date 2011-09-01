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
 * @file testConstIteratorAdapter.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Functions for testing class ConstIteratorAdapter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"


#include "DGtal/kernel/PointVector.h"


#include "DGtal/base/Modifier.h"
#include "DGtal/base/ConstIteratorAdapter.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConstIteratorAdapter.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testConstIteratorAdapter()
{

  typedef PointVector<3,int> Point;
  typedef PointVector<2,int> Point2d;
  typedef std::vector<Point>::iterator Iterator;
  typedef Point3dTo2dXZ<Point,Point2d> Modifier; 
  
  //range of 3d points
  std::vector<Point> r;
  r.push_back(Point(0,0,0));
  r.push_back(Point(1,0,0));
  r.push_back(Point(2,0,0));
  r.push_back(Point(2,1,0));
  r.push_back(Point(2,1,1));
  r.push_back(Point(3,1,1));
  r.push_back(Point(4,1,1));
  r.push_back(Point(4,2,1));
  r.push_back(Point(4,2,2));
  r.push_back(Point(5,2,2));
  r.push_back(Point(6,2,2));
  r.push_back(Point(6,3,2));
  r.push_back(Point(6,3,3));
  r.push_back(Point(6,4,3));
  r.push_back(Point(6,4,4));
  r.push_back(Point(6,5,4));

  
  trace.beginBlock ( "Testing block ..." );

  Iterator it = r.begin();
  Iterator itEnd = r.end();
  for ( ; it != itEnd; ++it) 
  {
    trace.info() << *it << endl; 
  }

  trace.info() << "Projection (XY)" << endl; 
  
  ConstIteratorAdapter<Iterator,Modifier> ait(r.begin()); 
  ConstIteratorAdapter<Iterator,Modifier> aitEnd(r.end()); 
  for ( ; ait != aitEnd; ++ait) 
  {
    trace.info() << *ait << endl; 
  }
  
  trace.endBlock();
  
  //to compare with an already projected range
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ConstIteratorAdapter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testConstIteratorAdapter(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
