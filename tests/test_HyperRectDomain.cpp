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
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/Space.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"


using namespace DGtal;
using namespace std;


/**
* Simple test of HyperRectDomain construction.
*
**/
bool testSimpleHyperRectDomain()
{

    typedef Space<int,4> Space4Type;
    typedef Space4Type::PointType PointType;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    PointType a ( t );
    PointType b ( t2 );

    trace.beginBlock ( "HyperRectDomain init" );
    ///Empty domain using the default constructor
    HyperRectDomain<Space4Type> myEmptyDomain;
    trace.info() << "Empty Domain: "<<myEmptyDomain<< std::endl;

    ///Domain characterized by points a and b
    HyperRectDomain<Space4Type> myHyperRectDomain ( a,b );

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
    PointType a ( t );
    PointType b ( t2 );

    trace.beginBlock ( "HyperRectDomain Iterator" );
    HyperRectDomain<SpaceType> myHyperRectDomain ( a,b );

    trace.info() << myHyperRectDomain << std::endl;

    trace.emphase() << "Iterator 2d: ";
    for ( HyperRectDomain<SpaceType>::ConstIterator it = myHyperRectDomain.begin();
            it != myHyperRectDomain.end();
            ++it )
        trace.info() << ( *it ) << std::endl;


    trace.emphase() << "Iterator 4d: ";
    typedef Space<int,4> SpaceType4D;
    typedef SpaceType4D::PointType PointType4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,3,3,3};
    PointType4D a4D ( t4D );
    PointType4D b4D ( t4D2 );

    HyperRectDomain<SpaceType4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    for ( HyperRectDomain<SpaceType4D>::ConstIterator it = myHyperRectDomain4D.begin();
            it != myHyperRectDomain4D.end();
            ++it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();
    return myHyperRectDomain.isValid();
}


bool testReverseIterator()
{
    typedef Space<int,4> SpaceType4D;
    typedef SpaceType4D::PointType PointType4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,3,3,3};
    PointType4D a4D ( t4D );
    PointType4D b4D ( t4D2 );

    trace.beginBlock ( "Test reverse iterator" );

    HyperRectDomain<SpaceType4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    for ( HyperRectDomain<SpaceType4D>::ConstIterator it = myHyperRectDomain4D.end();
            it != myHyperRectDomain4D.begin();
            --it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();

    return myHyperRectDomain4D.isValid();
}


bool testSTLCompat()
{
    typedef Space<int,4> SpaceType4D;
    typedef SpaceType4D::PointType PointType4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,3,3,3};
    PointType4D a4D ( t4D );
    PointType4D b4D ( t4D2 );

    trace.beginBlock ( "TestSTL Compatibility" );

    HyperRectDomain<SpaceType4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

		std::copy ( myHyperRectDomain4D.begin(),
								myHyperRectDomain4D.end(),
                ostream_iterator<PointType4D> ( trace.info(), " " ) );

		trace.info() << std::endl;
    trace.endBlock();

    return myHyperRectDomain4D.isValid();
}


int main()
{

    if ( testSimpleHyperRectDomain() && testIterator() && testReverseIterator() && testSTLCompat() )
        return 0;
    else
        return 1;
}

