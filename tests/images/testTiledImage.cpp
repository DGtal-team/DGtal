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
 * Functions for testing class TiledImage.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

//#define DEBUG_VERBOSE

#include "DGtal/images/ImageFactoryFromImage.h"
#include "DGtal/images/ImageCache.h"
#include "DGtal/images/TiledImage.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class TiledImage.
///////////////////////////////////////////////////////////////////////////////
bool testSimple()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing simple TiledImage");
    
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;

    VImage image(Z2i::Domain(Z2i::Point(0,0), Z2i::Point(3,3)));
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;

    trace.info() << "Original image: " << image << endl;

    // 1) ImageFactoryFromImage
    typedef ImageFactoryFromImage<VImage > MyImageFactoryFromImage;
    MyImageFactoryFromImage factImage(image);
    
    typedef MyImageFactoryFromImage::OutputImage OutputImage;
    
    Z2i::Domain domain1(Z2i::Point(0,0), Z2i::Point(1,1));
    OutputImage *image1 = factImage.requestImage(domain1);
    OutputImage::ConstRange r1 = image1->constRange();
    cout << "image1: "; std::copy( r1.begin(), r1.end(), std::ostream_iterator<int>(cout,", ") ); cout << endl;
    
    Z2i::Domain domain2(Z2i::Point(2,0), Z2i::Point(3,1));
    OutputImage *image2 = factImage.requestImage(domain2);
    OutputImage::ConstRange r2 = image2->constRange();
    cout << "image2: "; std::copy( r2.begin(), r2.end(), std::ostream_iterator<int>(cout,", ") ); cout << endl;
        
    Z2i::Domain domain3(Z2i::Point(0,2), Z2i::Point(1,3));
    OutputImage *image3 = factImage.requestImage(domain3);
    OutputImage::ConstRange r3 = image3->constRange();
    cout << "image3: "; std::copy( r3.begin(), r3.end(), std::ostream_iterator<int>(cout,", ") ); cout << endl;
        
    Z2i::Domain domain4(Z2i::Point(2,2), Z2i::Point(3,3));
    OutputImage *image4 = factImage.requestImage(domain4);
    OutputImage::ConstRange r4 = image4->constRange();
    cout << "image4: "; std::copy( r4.begin(), r4.end(), std::ostream_iterator<int>(cout,", ") ); cout << endl;
    
    // 2) ImageCache    
    typedef ImageCache<OutputImage > MyImageCache;
    MyImageCache imageCache(MyImageCache::LAST);
    /*VImage*/OutputImage::Value aValue;
    
    trace.info() << "Image cache: " << imageCache << endl;
    if (imageCache.read(Z2i::Point(2,2), aValue)) 
      trace.info() << "Point 2,2 is in an image from cache, value: " << aValue << endl;
    else
      trace.info() << "Point 2,2 is not in an image from cache." << endl;
    
    imageCache.update(image1);
    
    trace.info() << "Image cache: " << imageCache << endl;
    if (imageCache.read(Z2i::Point(2,2), aValue)) 
      trace.info() << "Point 2,2 is in an image from cache, value: " << aValue << endl;
    else
      trace.info() << "Point 2,2 is not in an image from cache." << endl;
    
    imageCache.update(image4);
    
    trace.info() << "Image cache: " << imageCache << endl;
    if (imageCache.read(Z2i::Point(2,2), aValue)) 
      trace.info() << "Point 2,2 is in an image from cache, value: " << aValue << endl;
    else
      trace.info() << "Point 2,2 is not in an image from cache." << endl;
    
    // 3) TiledImage    
    std::vector<Z2i::Domain> domains;
    domains.push_back(domain1);
    domains.push_back(domain2);
    domains.push_back(domain3);
    domains.push_back(domain4);
    
    typedef TiledImage<VImage > MyTiledImage;
    MyTiledImage tiledImage(image, domains);
    
    trace.info() << "Value for Point 2,2: " << tiledImage(Z2i::Point(2,2)) << endl;
    trace.info() << "Value for Point 3,1: " << tiledImage(Z2i::Point(3,1)) << endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
    trace.beginBlock ( "Testing class TiledImage" );
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
