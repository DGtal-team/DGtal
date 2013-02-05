/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testTiledImage.cpp
 * @ingroup Tests
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/23
 *
 * Functions for testing class TiledImage*.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

//#define DEBUG_VERBOSE

#include "DGtal/images/TiledImageFromImage.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class TiledImage*.
///////////////////////////////////////////////////////////////////////////////
bool testSimple()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing simple TiledImage*");
    
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;

    VImage image(Z2i::Domain(Z2i::Point(0,0), Z2i::Point(3,3)));
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;

    trace.info() << "Original image: " << image << endl;

    Z2i::Domain domain1(Z2i::Point(0,0), Z2i::Point(1,1));    
    Z2i::Domain domain2(Z2i::Point(2,0), Z2i::Point(3,1));        
    Z2i::Domain domain3(Z2i::Point(0,2), Z2i::Point(1,3));        
    Z2i::Domain domain4(Z2i::Point(2,2), Z2i::Point(3,3));
    
    std::vector<Z2i::Domain> domains;
    domains.push_back(domain1);
    domains.push_back(domain2);
    domains.push_back(domain3);
    domains.push_back(domain4);
    
    typedef TiledImageFromImage<VImage > MyTiledImageFromImage;
    MyTiledImageFromImage tiledImageFromImage(image, domains);
    
    typedef MyTiledImageFromImage::OutputImage OutputImage;
    /*VImage*/OutputImage::Value aValue;
    
    trace.info() << "Read value for Point 2,2: " << tiledImageFromImage(Z2i::Point(2,2)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(2,2)) == 11) ? 1 : 0; 
    nb++;
    
    trace.info() << "Read value for Point 3,1: " << tiledImageFromImage(Z2i::Point(3,1)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(3,1)) == 8) ? 1 : 0; 
    nb++;
    
    aValue = 88; tiledImageFromImage.setValue(Z2i::Point(3,1), aValue);
    trace.info() << "Write value for Point 3,1: " << aValue << endl;
    nbok += (tiledImageFromImage(Z2i::Point(3,1)) == 88) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
    trace.beginBlock ( "Testing class TiledImage*" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << endl;

    bool res = testSimple(); // && ... other tests

    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
