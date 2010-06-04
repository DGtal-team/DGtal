/**
 * @file test_Domain1DIterator.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/30
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_Domain1DIterator <p>
 * Aim: simple test of \ref LineDomain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/Space.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"


using namespace DGtal;
using namespace std;


/**
* Simple test of 1D iterators on a 4D digital domain
*
**/
bool testIterator()
{

    trace.emphase() << "Iterator 4d: ";
    typedef Space<int,4> SpaceType4D;
    typedef SpaceType4D::PointType PointType4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,6,3,3};
    const int t4D3[ ] = { 1, 2,1,1};
    PointType4D a4D ( t4D );
    PointType4D b4D ( t4D2 );
    PointType4D c4D ( t4D3 );

    trace.beginBlock("1D Domain iterator test");
    ///Domain construction
    HyperRectDomain<SpaceType4D> my1D ( a4D,b4D );


    trace.emphase() << my1D <<std::endl;

    ///iterates from  {1, 2,1,1} to { 3,6,3,3} along the dimension 1
    for ( HyperRectDomain<SpaceType4D>::Const1DIterator it = my1D.begin ( c4D , 1);
            it != my1D.end ( 1 );
            ++it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();
    return my1D.isValid();
}


int main()
{

    if ( testIterator() )
        return 0;
    else
        return 1;
}


