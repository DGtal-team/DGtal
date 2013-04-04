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
 * @file test_PointVector.cpp
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
#include "DGtal/kernel/PointVector.h"

using namespace DGtal;
using namespace std;

/**
 *
 * A PointVector may represent either a  point or a
 * vector depending on the context. For performance reasons, these
 * two types are just aliases. The user should take care how to use
 * it depending on the context. For instance, adding two points has
 * no meaning, but will be authorized by the compiler.
 *
 * The default less than operator is the one of the lexicographic
 * ordering, starting from dimension 0 to N-1.
 *
 * PointVector also realizes the concept CLattice with an infimum
 * (meet, greatest lower bound) and a supremum (join, least upper
 * bound) operation.
 *
 * Usage example:
 * @code
 *
 * ...
 * typedef PointVector<5,double> VectorD5;
 * VectorD5 p, q, r;
 *
 * p.at(1) = 2.0;  // p = {0.0, 2.0, 0.0, 0.0, 0.0}
 * q.at(3) = -5.5   // q = {0.0, 0.0, 0.0, -5.5, 0.0}
 * r =  p + q ;   //  r = {0.0, 2.0, 0.0, -5.5, 0.0}
 *
 * d = r.norm( DGtal::PointVector::L_infty ); // d = 5.5
 * ...
 * @endcode
 *
 */

bool testComparison()
{
  const double t[ ] = { 3.5, 4.1, 2.2, 3.2 };
  PointVector<4,double> v ( t );
  PointVector<4,double> v2 ( t );
#ifdef CPP11_INITIALIZER_LIST
  PointVector<4,double> v3 ( { 3.5, 4.2, 2.2, 3.2 } );
#endif

  trace.beginBlock("Comparison of Points");
  if (v == v2)
    trace.info()<< "v == v2 (true)"<<std::endl;
  else
    trace.info()<< "v == v2 (false)"<<std::endl;

#ifdef CPP11_INITIALIZER_LIST
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


bool testMaxMin()
{

  const double t[ ] = { 3.5, 4.1, 2.2, 3.2 };
  PointVector<4,double> v ( t );

  trace.beginBlock("Testing max/min of a vector");
  trace.info() << " Vector: "<< v<<std::endl;
  trace.info() << "max val = "<< v.max() <<std::endl;
  trace.info()<< "min val = "<<v.min() << std::endl;
  trace.info() << "maxElement val = "<< *v.maxElement() <<std::endl;
  trace.info()<< "minElement val = "<<*v.minElement() << std::endl;
  trace.endBlock();
  return ((v.max() == 4.1) && (v.min()==2.2));
}

/**
 * Test instanciation of Points
 *
 **/
bool testSimplePoint()
{
  PointVector<3, int>  aPVInt3;

  int t[]={-3 ,4 ,4 ,0};
  PointVector<4,int> aPoint(t);
  PointVector<4,int> aFPoint;

  aPoint *= 5;

  cout << "aPoint=" << aPoint << endl;

  trace.beginBlock ( "Test point dimension" );
  trace.info() << "aPoint dimension="<<aPoint.dimension <<endl;
  trace.endBlock();

  if ( aPoint.dimension != 4 )
    return false;

  int tt[] = { 3, 4, 2, 2 };
  PointVector<4,int> v (tt);
  aPoint = aFPoint + v;
  trace.beginBlock ( "Test point addition with vector" );
  trace.info() << "aPoint = "<< aFPoint << " + " << v << endl;
  trace.info() << "aPoint = "<< aPoint << endl;
  trace.endBlock();

  return true;
}

bool testNorms()
{
  typedef PointVector<3, int> PointType;
  PointType aPoint;

  aPoint[ 2 ] =  2;
  aPoint[ 1 ] = -1;
  aPoint[ 0 ] =  3;

  trace.beginBlock ( "Test of Norms" );
  trace.info() << "aPoint l_2 norm="<<aPoint.norm() <<endl;
  trace.info() << "aPoint l_1 norm="<<aPoint.norm ( PointType::L_1 ) <<endl;
  trace.info() << "aPoint l_infty norm="<<aPoint.norm ( PointType::L_infty ) <<endl;

  trace.info() << "Normalization="<<aPoint.getNormalized () <<endl;

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
  PointVector<3, int>  aPVInt3;
  PointVector<4, int> aVector;
  PointVector<4, int> aFVector;

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
  PointVector<25,int> aPoint;
  PointVector<4, int> avector;

  trace.beginBlock("Point Iterator Test");

  for (unsigned int i=0;i<25;++i)
    aPoint[i] = i;
  trace.info() << "aPoint="<<aPoint<< std::endl;

  trace.info() << "With iterator: ";
  for (PointVector<25,int>::ConstIterator it = aPoint.begin() ;  it != aPoint.end(); ++it)
    trace.info() << (*it) <<" " ;

  trace.info() << std::endl;

  trace.endBlock();

  return true;
}

bool testOperators()
{
  unsigned int nb = 0;
  unsigned int nbok = 0;
  trace.beginBlock("Point Operators Test");

  DGtal::int32_t t1[] = {1,2,3,4};
  PointVector<4, DGtal::int32_t> p1( t1 );
  DGtal::int32_t t2[]= {5,4,3,2};
  PointVector<4,DGtal::int32_t> p2( t2 );

  trace.info() << "p1: "<<p1 <<", "<<"p2: "<<p2 <<std::endl;
  trace.info() << "p1+p2: "<<p1+p2 <<std::endl;
  trace.info() << "p1*2+p2: "<<p1*2+p2 <<std::endl;
  trace.info() << "p1-p2: "<<p1-p2 <<std::endl;
  trace.info() << "-p2: "<< -p2 <<std::endl;
  trace.info() << "inf(p1,p2): "<<p1.inf(p2) <<std::endl;
  trace.info() << "sup(p1,p2): "<<p1.sup(p2) <<std::endl;
  trace.info() << "p1 dot p2: "<<p1.dot(p2) <<std::endl;

  trace.endBlock();

  trace.beginBlock("Vector Operators Test");
  PointVector<4,DGtal::int32_t> p3 = -p1;
  PointVector<4,DGtal::int32_t> p4 = -p3;
  ++nb, nbok += ( p4 == p1 ) ? 1 : 0;
  p4 = 2*p1 + p3;
  trace.info() << "2*p1+p3: "<< p4 << " (==p1)" << std::endl;
  ++nb, nbok += ( p4 == p1 ) ? 1 : 0;
  trace.info() << "2*p1+3*p2: "<< 2*p1+3*p2 << std::endl;
  trace.endBlock();

  return nb == nbok;
}

bool testIntegerNorms()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  DGtal::int32_t t[]= {2,1,3,4};
  PointVector<4,DGtal::int32_t> p1(t);
  DGtal::int32_t t2[]= {4,5,3,2};
  PointVector<4,DGtal::int32_t> p2(t2);
  PointVector<4,DGtal::int32_t> p = p2 - p1;

  trace.beginBlock ( "Checking Integer norm1" );
  trace.info() << "p1: "<<p1 <<", "<<"p2: "<<p2 <<std::endl;
  nbok += p.norm1() == 8 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "L1(p2-p1): "<< p.norm1() << "( == 8 ?)" << std::endl;
  nbok += p.normInfinity() == 4 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "Linfty(p2-p1): "<< p.normInfinity()  << "( == 4 ?)"
         << std::endl;
  trace.endBlock();

  return nbok == nb;
}

int main()
{
  bool res;
  res =  testSimplePoint()
    && testSimpleVector()
    && testNorms()
    && testIterator()
    && testComparison()
    && testOperators()
    && testIntegerNorms()
    && testMaxMin();
  if (res)
    return 0;
  else
    return 1;
}

/** @ingroup Tests **/
