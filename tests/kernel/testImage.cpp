/**
 * @file test_Image.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_Image <p>
 * Aim: simple test of \ref Image
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/ImageSelector.h"


using namespace DGtal;
using namespace std;


/**
* Simple test of Image construction.
*
**/
bool testSimpleImage()
{

    typedef SpaceND<4> Space4Type;
    typedef HyperRectDomain<Space4Type> TDomain;
    typedef TDomain::Point Point;

    //Default image selector = STLVector
    typedef ImageSelector<TDomain, int>::Type Image;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    Point a ( t );
    Point b ( t2 );

    trace.beginBlock ( "Image init" );

    ///Domain characterized by points a and b
    Image myImage ( a,b );
    trace.info() << myImage << std::endl;
    trace.endBlock();

    trace.beginBlock("Test of built-in iterators");


    trace.info() << " Vector iterator: ";
    for ( Image::ConstIterator it = myImage.begin();
            it != myImage.end();
            ++it)
        trace.info() << myImage(it) <<" ";

    trace.info()<<std::endl;
    trace.endBlock();
    
    return myImage.isValid();
}

/*
bool testImageContainer()
{
    typedef SpaceND<int,4> Space4Type;
    typedef Space4Type::Point Point;
    typedef HyperRectDomain<Space4Type> TDomain;
    typedef ImageContainerBySTLVector<Point, double> TContainerV;
    typedef ImageContainerBySTLMap<Point, double> TContainerM;

    bool res = true;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    const int t3[ ] = { 5, 3, 3 ,4};
    Point a ( t );
    Point b ( t2 );

    Point c ( t3 );

    trace.beginBlock ( "Image Container" );

    ///Domain characterized by points a and b
    Image<TDomain,double, TContainerV> myImageV ( a,b );
    trace.info() << "Vector container, value at c="<<myImageV( c )<< std::endl;

    ///Domain characterized by points a and b
    Image<TDomain,double, TContainerM> myImageM ( a,b );

    res = res && myImageM.isValid() && myImageV.isValid();

    //We revert the bool flag to catch the exception
    res = !res;

    try {
        trace.info() << "Map container, value at c="<<myImageM( c )<< std::endl;
    }
    catch (std::bad_alloc e)
    {
        trace.warning() << "Exception bad_alloc catched.. this is normal for the map container"<<std::endl;
        res = !res;
    }

    trace.endBlock();

    return res;
}
*/


int main()
{

  if ( testSimpleImage() )//&& testImageContainer() && testBuiltInIterators() && testConcepts() )
        return 0;
    else
        return 1;
}

