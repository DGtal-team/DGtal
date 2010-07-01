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
    typedef Space4Type::Point Point;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    Point a ( t );
    Point b ( t2 );

    trace.beginBlock ( "HyperRectDomain init" );
    ///Empty domain using the default constructor
    HyperRectDomain<Space4Type> myEmptyDomain;
    trace.info() << "Empty Domain: "<<myEmptyDomain<< std::endl;

    ///Domain characterized by points a and b
    HyperRectDomain<Space4Type> myHyperRectDomain ( a,b );

    trace.info() << myHyperRectDomain << std::endl;

    trace.info() << "Domain Extent= "<< myHyperRectDomain.extent()<<std::endl;
    
    
    trace.endBlock();
 
    
    trace.beginBlock("Test Copy Constructor");
    HyperRectDomain<Space4Type> myHyperRectDomainBis( myHyperRectDomain );
    trace.info() << "Domain Extent= "<< myHyperRectDomainBis.extent()<<std::endl;
    trace.endBlock();

    trace.beginBlock("Test Assignement");
    HyperRectDomain<Space4Type> myHyperRectDomainTer;

    myHyperRectDomainTer = myHyperRectDomain;
    
    trace.info() << "Domain Extent= "<< myHyperRectDomainTer.extent()<<std::endl;
    trace.endBlock();

    return myHyperRectDomain.isValid();
    
}

bool testIterator()
{
    typedef Space<int,2> TSpace;
    typedef TSpace::Point Point;
    const int t[ ] = { 1, 1};
    const int t2[ ] = { 5, 5};
    Point a ( t );
    Point b ( t2 );

    trace.beginBlock ( "HyperRectDomain Iterator" );
    HyperRectDomain<TSpace> myHyperRectDomain ( a,b );

    trace.info() << myHyperRectDomain << std::endl;

    trace.emphase() << "Iterator 2d: ";
    for ( HyperRectDomain<TSpace>::ConstIterator it = myHyperRectDomain.begin();
            it != myHyperRectDomain.end();
            ++it )
        trace.info() << ( *it ) << std::endl;


    trace.emphase() << "Iterator 4d: ";
    typedef Space<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    const int t4D[ ] = {1,1,1,1};
    const int t4D2[ ] = {3,3,3,3};
    Point4D a4D ( t4D );
    Point4D b4D ( t4D2 );

    HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    for ( HyperRectDomain<TSpace4D>::ConstIterator it = myHyperRectDomain4D.begin();
            it != myHyperRectDomain4D.end();
            ++it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();
    return myHyperRectDomain.isValid();
}


bool testReverseIterator()
{
    typedef Space<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,3,3,3};
    Point4D a4D ( t4D );
    Point4D b4D ( t4D2 );

    trace.beginBlock ( "Test reverse iterator" );

    HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    for ( HyperRectDomain<TSpace4D>::ConstIterator it = myHyperRectDomain4D.end();
            it != myHyperRectDomain4D.begin();
            --it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();

    return myHyperRectDomain4D.isValid();
}



bool testSTLCompat()
{
    typedef Space<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    const int t4D[ ] = { 1, 1,1,1};
    const int t4D2[ ] = { 3,3,3,3};
    Point4D a4D ( t4D );
    Point4D b4D ( t4D2 );

    trace.beginBlock ( "TestSTL Compatibility" );

    HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    std::copy ( myHyperRectDomain4D.begin(),
                myHyperRectDomain4D.end(),
                ostream_iterator<Point4D> ( trace.info(), " " ) );

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

