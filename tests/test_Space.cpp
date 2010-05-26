/**
 * @file test_Space.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/03/03
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_Space <p>
 * Aim: simple test of \ref Space
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

 cout << "aSpace6 = " << aSpace6 << endl;
 cout << "aSpace2 = " << aSpace2 << endl;
 Space<int,6>::Subcospace<2>::Type aSpace4 = aSpace6.subcospace<2>();
 // won't work
 // aSpace6::SpaceType::Subcospace<2>::Type aSpace4 = aSpace6.getSubcospace<2>();
 cout << "aSpace6.subcospace<2> = " << aSpace4 << endl;

 return true;
}




int main()
{
  
  if (testSimpleSpace())
     return 0;
  else
    return 1;
}

