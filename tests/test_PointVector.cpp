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

using namespace DGtal;
using namespace std;



/**
 * Compute the measure of the unit square [0,1]x[0,1]
 *
 * Expected value : sqrt{2}/2
 **/
bool testSimple()
{
  PointVector<int,3>  aPVInt3;
  Point<double,4> aPoint;
  
  aPVInt3.zero();
  aPoint.zero();
  
  cout << "aPoint dimension="<<aPoint.getDimension()<<endl;
  if (aPoint.getDimension() != 4)
      return false;
  
  
  
  return true;
}





int main(int argc, char **argv)
{
  
  if (testSimple())
     return 0;
  else
    return 1;
}

