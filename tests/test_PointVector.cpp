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
  
  cout << "aPoint dimension="<<aPoint.getDimension()<<endl;
  if (aPoint.getDimension() != 4)
      return false;
  
   aPoint += aFPoint;
  
  return true;
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
  
  cout << "aVector dimension="<<aVector.getDimension()<<endl;
  if (aVector.getDimension() != 4)
    return false;
  
  aVector += aFVector;
  
  return true;
}





int main(int argc, char **argv)
{
  
  if (testSimplePoint() && testSimpleVector())
     return 0;
  else
    return 1;
}

