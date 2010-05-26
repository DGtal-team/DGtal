/**
 * @file test_HyperRectDomain.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_HyperRectDomain <p>
 * Aim: simple test of \ref HyperRectDomain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/HyperRectDomain.h"
#include "DGtal/kernel/Space.h"

using namespace DGtal;
using namespace std;


/**
* Test instancition of HyperRectDomain
*
**/
bool testSimpleHyperRectDomain()
{

    typedef Space<int,4> Space4Type;
    typedef Space4Type::PointType PointType;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    PointType a( t );
    PointType b( t2 );

    trace.beginBlock("HyperRectDomain init");
    ///Empty domain using the default constructor
    HyperRectDomain<Space4Type> myEmptyDomain;
    trace.info() << "Empty Domain: "<<myEmptyDomain<< std::endl;

    ///Empty domain using the standard constructor
    HyperRectDomain<Space4Type> myHyperRectDomain(a,b);

    trace.info() << myHyperRectDomain << std::endl;

    trace.endBlock();
    return myHyperRectDomain.isValid();
}

bool testIterator()
{
    typedef Space<int,2> SpaceType;
    typedef SpaceType::PointType PointType;
    const int t[ ] = { 1, 1};
    const int t2[ ] = { 5, 5};
    PointType a( t );
    PointType b( t2 );

    trace.beginBlock("HyperRectDomain Iterator");

    ///Empty domain using the standard constructor
    HyperRectDomain<SpaceType> myHyperRectDomain(a,b);

    trace.info() << myHyperRectDomain << std::endl;

    trace.emphase() << "Iterator: ";
    for (HyperRectDomain<SpaceType>::ConstIterator it = myHyperRectDomain.begin();
            it != myHyperRectDomain.end();
            ++it)
        trace.info() << (*it) << std::endl;


    trace.endBlock();
    return myHyperRectDomain.isValid();
}



int main()
{

    if (testSimpleHyperRectDomain() && testIterator())
        return 0;
    else
        return 1;
}

