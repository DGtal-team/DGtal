/**
 * @file test_BoxDomain.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_BoxDomain <p>
 * Aim: simple test of \ref BoxDomain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BoxDomain.h"
#include "DGtal/kernel/Space.h"

using namespace DGtal;
using namespace std;


/**
* Test instancition of BoxDomain
*
**/
bool testSimpleBoxDomain()
{

    typedef Space<int,4> Space4Type;
    typedef Space4Type::PointType PointType;

    const int t[ ] = { 1, 2, 3 ,4};
    const int t2[ ] = { 5, 5, 3 ,4};
    PointType a( t );
    PointType b( t2 );

    trace.beginBlock("BoxDomain init");
    ///Empty domain using the default constructor
    BoxDomain<Space4Type> myEmptyDomain;
    trace.info() << "Empty Domain: "<<myEmptyDomain<< std::endl;

    ///Empty domain using the standard constructor
    BoxDomain<Space4Type> myBoxDomain(a,b);

    trace.info() << myBoxDomain << std::endl;

    trace.endBlock();
    return myBoxDomain.isValid();
}




int main()
{

    if (testSimpleBoxDomain())
        return 0;
    else
        return 1;
}

