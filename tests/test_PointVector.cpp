/**
 * @file testr_measure.cpp
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
#include <assert.h>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/Point.h"
#include "DGtal/kernel/Vector.h"

using namespace DGtal;
using namespace std;



/**
 * Test instancition of Points 
 *
 **/
bool testSimplePoint()
{
  PointVector<int,3>  aPVInt3; //shouldn't be authorized
  Point<double,4> aPoint;
  Point<double,4> aFPoint;
  
  aPVInt3.zero();
  aPoint.zero();
  aFPoint.zero();
  
  aPoint.getSetVal(2) = 4.5;
  aPoint.getSetVal(1) = 4;
  aPoint.getSetVal(0) = -3;
  
  aPoint *= 5.6;
  
  trace.beginBlock("Test point dimension");
  trace.info()<< "aPoint dimension="<<aPoint.getDimension()<<endl;
  trace.endBlock();
  
  if (aPoint.getDimension() != 4)
      return false;
  
  
   aPoint += aFPoint;
  
  return true;
}

bool testNorms()
{
  typedef Point<double,3> PointType;
  PointType aPoint;
  
  aPoint.getSetVal(2) = 2;
  aPoint.getSetVal(1) = -1;
  aPoint.getSetVal(0) = 3;

  trace.beginBlock("Test of Norms");
  trace.info() << "aPoint l_2 norm="<<aPoint.norm()<<endl;
  trace.info() << "aPoint l_1 norm="<<aPoint.norm(PointType::L_1)<<endl;
  trace.info() << "aPoint l_infty norm="<<aPoint.norm(PointType::L_infty)<<endl;
  trace.endBlock();
  
  
  return ((aPoint.norm(PointType::L_1) == 6) && 
  (aPoint.norm(PointType::L_infty) == 3));
  
}

/**
* Test instancition of Vectors 
*
**/
bool testSimpleVector()
{
  Vector<int,3>  aPVInt3; //shouldn't be authorized
  Vector<double,4> aVector;
  Vector<double,4> aFVector;
  
  aPVInt3.zero();
  aVector.zero();
  aFVector.zero();
  
  trace.beginBlock("Test of Vector Dimension");
  trace.info() << "aVector dimension="<<aVector.getDimension()<<endl;
  trace.endBlock();
  
  if (aVector.getDimension() != 4)
    return false;
  
  aVector += aFVector;
  
  return true;
}


bool testPointTypeConversion()
{
  Point<int,3> aPointInt3;
  Point<int,3> aPointInt3b;
  Point<double,3> aPointInt3bb;
  
  aPointInt3b.getSetVal(2) = 4;
  aPointInt3 = aPointInt3b;

  aPointInt3bb.getSetVal(2) = 4.0;
  
  //This assignement does not compile
  //aPointInt3 = aPointInt3bb;
  
}


int main()
{
  
  if (testSimplePoint() && testSimpleVector() && testNorms() && testPointTypeConversion())
     return 0;
  else
    return 1;
}

