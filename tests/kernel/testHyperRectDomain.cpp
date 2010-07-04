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
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"


using namespace DGtal;
using namespace std;


/**
* Simple test of HyperRectDomain construction.
*
**/
bool testSimpleHyperRectDomain()
{

    typedef SpaceND<int,4> Space4Type;
    typedef Space4Type::Point Point;

    Point a ({ 1, 2, 3 ,4});
    Point b ({ 5, 5, 3 ,4} );

    trace.beginBlock ( "HyperRectDomain init" );
    // Checking that HyperRectDomain is a model of CDomain.
    typedef HyperRectDomain<Space4Type> HRDomain4;
    BOOST_CONCEPT_ASSERT(( CDomain< HRDomain4 > ));

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
    typedef SpaceND<int,2> TSpace;
    typedef TSpace::Point Point;
    Point a ({ 1, 1});
    Point b ({ 5, 5});

    trace.beginBlock ( "HyperRectDomain Iterator" );
    HyperRectDomain<TSpace> myHyperRectDomain ( a,b );

    trace.info() << myHyperRectDomain << std::endl;

    trace.emphase() << "Iterator 2d: ";
    for ( HyperRectDomain<TSpace>::ConstIterator it = myHyperRectDomain.begin();
            it != myHyperRectDomain.end(); ++it )
        trace.info() << ( *it ) << std::endl;


    trace.emphase() << "Iterator 4d: ";
    typedef SpaceND<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    Point4D a4D ( {1,1,1,1} );
    Point4D b4D ( {3,3,3,3} );

    HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    for ( HyperRectDomain<TSpace4D>::ConstIterator it = myHyperRectDomain4D.begin();
            it != myHyperRectDomain4D.end();
            ++it )
        trace.info() << ( *it ) << std::endl;

    trace.endBlock();

    trace.emphase() << "Iterator 4d by using order different from lexicographic: ";
    for ( HyperRectDomain<TSpace4D>::ConstIterator it = myHyperRectDomain4D.begin({3,2,1,0});
	  it != myHyperRectDomain4D.end({3,2,1,0}); ++it )
        trace.info() << ( *it ) << std::endl;

    trace.emphase() << "Decreasing Iterator 4d by using order different from lexicographic: ";
    HyperRectDomain<TSpace4D>::ConstIterator it1 = myHyperRectDomain4D.end({3,2,1,0});
    HyperRectDomain<TSpace4D>::ConstIterator it2 = myHyperRectDomain4D.begin({3,2,1,0});
    --it1; --it2;
    std::cout<<"BEGIN:"<<*it1<<" END:"<<*it2<<std::endl;
    for ( ; it1!=it2; --it1 )
      trace.info() << ( *it1 ) << std::endl;

    return myHyperRectDomain.isValid();
}


bool testReverseIterator()
{
    typedef SpaceND<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    Point4D a4D ({1,1,1,1});
    Point4D b4D ({3,3,3,3});

    trace.beginBlock ( "Test reverse iterator" );

    HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D,b4D );
    trace.emphase() << myHyperRectDomain4D<<std::endl;

    trace.emphase() << "Increasing order: ";
    
    HyperRectDomain<TSpace4D>::ConstIterator it = myHyperRectDomain4D.begin();
    for ( ; it != myHyperRectDomain4D.end(); ++it )
      trace.info() << ( *it ) << std::endl;

    trace.emphase() << "Now decreasing order: ";
    HyperRectDomain<TSpace4D>::ConstIterator it2 = myHyperRectDomain4D.begin();
    --it; --it2; 
    for ( ; it != it2; --it )
      trace.info() << ( *it ) << std::endl;

    trace.endBlock();

    return myHyperRectDomain4D.isValid();
}



bool testSTLCompat()
{
    typedef SpaceND<int,4> TSpace4D;
    typedef TSpace4D::Point Point4D;
    Point4D a4D ({1,1,1,1});
    Point4D b4D ({3,3,3,3});

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

/** @ingroup Tests **/
