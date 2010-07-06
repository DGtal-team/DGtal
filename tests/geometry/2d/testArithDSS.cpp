/**
 * @file test_arithDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2010/07/02
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_arithDSS <p>
 * Aim: simple test of \ref arithDSS, \ref arithDSS4
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/ArithDSS4.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"


using namespace DGtal;
using namespace std;



int main(int argc, char **argv)
{


  typedef SpaceND<int,2> Space2Type;
  typedef HyperRectDomain<Space2Type> Domain2D;
  typedef Space2Type::Point Point;
  
  Point firstPoint ( { 0, 0 } );
  Point secondPoint ( { 1, 0 } );

    trace.info() << "Init of a DSS ";
		ArithDSS4<Domain2D> theDSS(firstPoint,secondPoint);		
    trace.emphase() <<" hop "<<endl;


    return 0;
}
