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
 * @file test_RealPointVector.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/03/03
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_trace' <p>
 * Aim: simple test of \ref MeasureOfStraighLines
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/RealPointVector.h"

using namespace DGtal;
using namespace std;

/**
 *
 */

bool testComparison()
{
  const double t[ ] = { 3.5, 4.1, 2.2, 3.2 };
  RealPointVector<4> v ( t );
  RealPointVector<4> v2 ( t );
#ifdef CPP0X_INITIALIZER_LIST
  RealPointVector<4> v3 ( { 3.5, 4.2, 2.2, 3.2 } );
#endif

  trace.beginBlock("Comparison of Points");
  if (v == v2)
    trace.info()<< "v == v2 (true)"<<std::endl;
  else
    trace.info()<< "v == v2 (false)"<<std::endl;

#ifdef CPP0X_INITIALIZER_LIST
  if (v == v3)
    trace.info()<< "v == v3 (true)"<<std::endl;
  else
    trace.info()<< "v == v3 (false)"<<std::endl;
#endif

  if (v < v2)
    trace.info()<< "v < v2 (true)"<<std::endl;
  else
    trace.info()<< "v < v2 (false)"<<std::endl;


  trace.endBlock();

  return ((v == v2) && !(v != v2));
}



/**
 * Test instanciation of Points
 *
 **/
bool testSimplePoint()
{
  RealPointVector<3>  aPVInt3; 

  double t[]={-3 ,4 ,4 ,0};
  RealPointVector<4> aPoint(t);
  RealPointVector<4> aFPoint;

  aPoint *= 5;

  cout << "aPoint=" << aPoint << endl;

  trace.beginBlock ( "Test point dimension" );
  trace.info() << "aPoint dimension="<<aPoint.dimension <<endl;
  trace.endBlock();

  if ( aPoint.dimension != 4 )
    return false;

  double tt[] = { 3, 4, 2, 2 };
  RealPointVector<4> v (tt);
  aPoint = aFPoint + v;

  trace.beginBlock ( "Test point addition with vector" );
  trace.info() << "aPoint = "<< aFPoint << " + " << v << endl;
  trace.info() << "aPoint = "<< aPoint << endl;
  trace.endBlock();

  return true;
}

bool testNorms()
{
  typedef RealPointVector<3> PointType;
  PointType aPoint;

  aPoint.at ( 2 ) =  2;
  aPoint.at ( 1 ) = -1;
  aPoint.at ( 0 ) =  3;

  trace.beginBlock ( "Test of Norms" );
  trace.info() << "aPoint l_2 norm="<<aPoint.norm() <<endl;
  trace.info() << "aPoint l_1 norm="<<aPoint.norm ( PointType::L_1 ) <<endl;
  trace.info() << "aPoint l_infty norm="<<aPoint.norm ( PointType::L_infty ) <<endl;
  trace.endBlock();


  return ( ( aPoint.norm ( PointType::L_1 ) == 6 ) &&
	   ( aPoint.norm ( PointType::L_infty ) == 3 ) );

}

/**
 * Test instancition of Vectors
 *
 **/
bool testSimpleVector()
{
  RealPointVector<3>  aPVInt3;
  RealPointVector<4> aVector;
  RealPointVector<4> aFVector;

  trace.beginBlock ( "Test of Vector Dimension" );
  trace.info() << "aVector dimension="<< aVector.dimension <<endl;
  trace.info() << "aVector = "<< aVector <<endl;
  trace.endBlock();

  if ( aVector.dimension != 4 )
    return false;

  aVector += aFVector;

  return true;
}


bool testIterator()
{
  RealPointVector<25> aPoint;
  RealPointVector<4> avector;
  
  trace.beginBlock("Point Iterator Test");

  for (unsigned int i=0;i<25;++i)
    aPoint.at(i) = i;
  trace.info() << "aPoint="<<aPoint<< std::endl;

  trace.info() << "With iterator: ";
  for (RealPointVector<25>::Iterator it = aPoint.begin() ;  it != aPoint.end(); ++it)
    trace.info() << (*it) <<" " ;

  trace.info() << std::endl;

  trace.endBlock();

  return true;
}

bool testOperators()
{
  trace.beginBlock("Point Operators Test");
  
  double t1[] = {1.0,2.0,3.0,4.0};
  RealPointVector<4> p1( t1 );
  double t2[]= {5.0,4.0,3.0,2.0};
  RealPointVector<4> p2( t2 );

  trace.info() << "p1: "<<p1 <<std::endl;
  trace.info() <<"p2: "<<p2 <<std::endl;
  trace.info() << "p1+p2: "<<p1+p2 <<std::endl;
  trace.info() << "p1-p2: "<<p1-p2 <<std::endl;
  trace.info() << "p1/p2: "<<p1/p2 <<std::endl;
  trace.info() << "inf(p1,p2): "<<p1.inf(p2) <<std::endl;
  trace.info() << "sup(p1,p2): "<<p1.sup(p2) <<std::endl;
  trace.endBlock();

  return true; 
}


int main()
{
  bool res;
  res =  testSimplePoint()  
    && testSimpleVector()
    && testNorms()  
    && testIterator() 
    && testComparison() 
    && testOperators();
  if (res)
    return 0;
  else
    return 1;
}

/** @ingroup Tests **/
