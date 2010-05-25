/**
 * @file test_Domain.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/21
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_Domain <p>
 * Aim: simple test of \ref Domain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/Domain.h"
#include "DGtal/kernel/Space.h"

using namespace DGtal;
using namespace std;


/**
* Test instancition of Domain 
*
**/
bool testSimpleDomain()
{
  
 typedef Space<int,6> Space6Type;
 typedef Domain<Space6Type> Domain6;
 typedef Domain6::PointType PointInDomain6Type;

 PointInDomain6Type myPoint;

 trace.info() << myPoint<< std::endl;
 trace.info() << "Dimension = "<< myPoint.dimension() << std::endl;
 
 return myPoint.isValid();
}




int main()
{
  
  if (testSimpleDomain())
     return 0;
  else
    return 1;
}

