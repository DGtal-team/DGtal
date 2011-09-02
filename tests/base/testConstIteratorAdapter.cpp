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

#include <boost/concept_check.hpp>


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
bool testProjection()
{

  bool flag1 = true; 
  bool flag2 = true; 
  
  typedef PointVector<3,int> Point3;
  typedef PointVector<2,int> Point2;
  typedef std::vector<Point3>::iterator Iterator3;
  typedef std::vector<Point2>::iterator Iterator2;
  typedef Point3dTo2dXY<int> Modifier; 
  
  typedef ConstIteratorAdapter<Iterator3,Modifier> Adapter; 
  BOOST_CONCEPT_ASSERT(( boost::RandomAccessIterator<Iterator3> ));
  BOOST_CONCEPT_ASSERT(( boost::RandomAccessIterator<Adapter> ));
  
  //range of 3d Points
  std::vector<Point3> r;
  r.push_back(Point3(0,0,0));
  r.push_back(Point3(1,0,0));
  r.push_back(Point3(2,0,0));
  r.push_back(Point3(2,1,0));
  r.push_back(Point3(2,1,1));
  r.push_back(Point3(3,1,1));
  r.push_back(Point3(4,1,1));
  r.push_back(Point3(4,2,1));
  r.push_back(Point3(4,2,2));
  r.push_back(Point3(5,2,2));
  r.push_back(Point3(6,2,2));
  r.push_back(Point3(6,3,2));
  r.push_back(Point3(6,3,3));
  r.push_back(Point3(6,4,3));
  r.push_back(Point3(6,4,4));
  r.push_back(Point3(6,5,4));
  
  //true projection
  std::vector<Point2> rtrue;
  rtrue.push_back(Point2(0,0));
  rtrue.push_back(Point2(1,0));
  rtrue.push_back(Point2(2,0));
  rtrue.push_back(Point2(2,1));
  rtrue.push_back(Point2(2,1));
  rtrue.push_back(Point2(3,1));
  rtrue.push_back(Point2(4,1));
  rtrue.push_back(Point2(4,2));
  rtrue.push_back(Point2(4,2));
  rtrue.push_back(Point2(5,2));
  rtrue.push_back(Point2(6,2));
  rtrue.push_back(Point2(6,3));
  rtrue.push_back(Point2(6,3));
  rtrue.push_back(Point2(6,4));
  rtrue.push_back(Point2(6,4));
  rtrue.push_back(Point2(6,5));
  
  trace.beginBlock ( "Testing block ..." );

    //display
    trace.info() << "3d points " << endl; 
/*
    Iterator3 it = r.begin();
    Iterator3 itEnd = r.end();
    for ( ; it != itEnd; ++it) 
    {
      trace.info() << *it; 
    }
    trace.info()  << endl;
*/
    trace.info() << "2d points after projection (XY)" << endl; 
    
    Adapter aitBegin(r.begin());
    Adapter ait = aitBegin;    
    Adapter aitEnd(r.end()); 
/*
    for ( ; ait != aitEnd; ++ait) 
    {
      trace.info() << *ait;  
    }
    trace.info()  << endl;
*/
    //comparison
    flag1 = std::equal( rtrue.begin(), rtrue.end(), aitBegin ); 

    //random acces
    ait = aitBegin + 3;
    ait = 2 + ait; 
    trace.info() << "some random access operators" << endl; 
    trace.info() << *aitBegin << *ait << (aitBegin < ait) << endl; 
    if (aitBegin < ait) flag2 = true;
    else flag2 = false; 
    
  trace.endBlock();

    
  return flag1 && flag2;
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

  bool res = testProjection()
  ; // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
