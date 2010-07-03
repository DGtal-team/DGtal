/**
* @file test_HyperRectDomain-snippet.cpp
* @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
*
*
* @date 2010/05/25
*
* This file is part of the DGtal library
*/

/**
* Description of test_HyperRectDomain-snippet <p>
* Aim: simple test of \ref HyperRectDomain based on the code snippet in the \ref HyperRectDomain class documentation.
*/

#include <algorithm> //for std::copy()
#include <iterator>
#include <iostream>
#include <DGtal/kernel/SpaceND.h>
#include <DGtal/kernel/domains/HyperRectDomain.h>


using namespace DGtal;

int main()
{
    //We create a digital Space based on 'int' integers and in dimension 4
    typedef DGtal::SpaceND<int,4> Space4DType;
    typedef Space4DType::Point Point4DType;

    const int rawA[ ] = { 1, 2, 3 ,4};
    const int rawB[ ] = { 4, 4, 5 ,5};
    Point4DType A ( rawA );
    Point4DType B ( rawB );

    //Domain construction from two points
    HyperRectDomain<Space4DType> myDomain ( A, B );

    //We just iterate on the Domain points and print out the point coordinates.
    std::copy ( myDomain.begin(),
                myDomain.end(),
                std::ostream_iterator<Point4DType> ( std::cout, " " ) );
}
/** @ingroup Tests **/
