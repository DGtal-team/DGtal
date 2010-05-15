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
#include "DGtal/kernel/Space.h"


using namespace DGtal;
using namespace std;





/**
* Test instancition of Space 
*
**/
bool testSimpleSpace()
{
  
 Space<int,6> aSpace6;
 Space<double,2> aSpace2;
  
 return true;
}




int main()
{
  
  if (testSimpleSpace())
     return 0;
  else
    return 1;
}

