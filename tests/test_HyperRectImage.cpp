/**
 * @file test_HyperRectImage.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_HyperRectImage <p>
 * Aim: simple test of \ref HyperRectImage
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/Space.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/domains/HyperRectImage.h"


using namespace DGtal;
using namespace std;


/**
* Simple test of HyperRectImage construction.
*
**/
bool testSimpleHyperRectImage()
{

    typedef Space<int,4> Space4Type;
    typedef Space4Type::PointType PointType;
    typedef HyperRectDomain<Space4Type> TDomain;
    
    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    PointType a ( t );
    PointType b ( t2 );

    trace.beginBlock ( "HyperRectImage init" );

    ///Domain characterized by points a and b
    HyperRectImage<TDomain,double> myHyperRectImage (a,b );
    trace.info() << myHyperRectImage << std::endl;
 
    trace.endBlock();
    return myHyperRectImage.isValid();
}

int main()
{

    if ( testSimpleHyperRectImage()  )
        return 0;
    else
        return 1;
}

