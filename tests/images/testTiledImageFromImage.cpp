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
 * @file testTiledImageFromImage.cpp
 * @ingroup Tests
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/23
 *
 * @brief A test file for tiledImageFromImage.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/TiledImageFromImage.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class TiledImageFromImage.
///////////////////////////////////////////////////////////////////////////////
bool testSimple()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing simple TiledImageFromImage");
    
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;
    VImage image(Z2i::Domain(Z2i::Point(1,1), Z2i::Point(16,16)));
    
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;

    trace.info() << "ORIGINAL image: " << image << endl;
    
    typedef ImageFactoryFromImage<VImage> MyImageFactoryFromImage;
    typedef MyImageFactoryFromImage::OutputImage OutputImage;
    MyImageFactoryFromImage imageFactoryFromImage(image);
    
    typedef ImageCacheReadPolicyFIFO<OutputImage, MyImageFactoryFromImage> MyImageCacheReadPolicyFIFO;
    typedef ImageCacheWritePolicyWT<OutputImage, MyImageFactoryFromImage> MyImageCacheWritePolicyWT;
    MyImageCacheReadPolicyFIFO imageCacheReadPolicyFIFO(imageFactoryFromImage, 2);
    MyImageCacheWritePolicyWT imageCacheWritePolicyWT(imageFactoryFromImage);
    
    typedef TiledImageFromImage<VImage, MyImageFactoryFromImage, MyImageCacheReadPolicyFIFO, MyImageCacheWritePolicyWT> MyTiledImageFromImage;
    MyTiledImageFromImage tiledImageFromImage(image, imageFactoryFromImage, imageCacheReadPolicyFIFO, imageCacheWritePolicyWT, 4);
    
    typedef MyTiledImageFromImage::OutputImage OutputImage;
    /*VImage*/OutputImage::Value aValue;
    
    trace.info() << "Read value for Point 4,2: " << tiledImageFromImage(Z2i::Point(4,2)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(4,2)) == 20) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "Read value for Point 10,6: " << tiledImageFromImage(Z2i::Point(10,6)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(10,6)) == 90) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    aValue = 1; tiledImageFromImage.setValue(Z2i::Point(11,7), aValue);
    trace.info() << "Write value for Point 11,7: " << aValue << endl;
    nbok += (tiledImageFromImage(Z2i::Point(11,7)) == 1) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "Read value for Point 2,3: " << tiledImageFromImage(Z2i::Point(2,3)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(2,3)) == 34) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "Read value for Point 16,1: " << tiledImageFromImage(Z2i::Point(16,1)) << endl;
    nbok += (tiledImageFromImage(Z2i::Point(16,1)) == 16) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    aValue = 128; tiledImageFromImage.setValue(Z2i::Point(16,1), aValue);
    trace.info() << "Write value for Point 16,1: " << aValue << endl;
    nbok += (tiledImageFromImage(Z2i::Point(16,1)) == 128) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "  Point 16,1 on ORIGINAL image, value: " << image(Z2i::Point(16,1)) << endl;
    nbok += (image(Z2i::Point(16,1)) == 128) ? 1 : 0;
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

bool test3d()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing 3d TiledImageFromImage");
    
    typedef ImageContainerBySTLVector<Z3i::Domain, int> VImage;
    VImage image(Z3i::Domain(Z3i::Point(1,1,1), Z3i::Point(4,4,4)));
    
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;

    trace.info() << "ORIGINAL image: " << image << endl;
    
    typedef ImageFactoryFromImage<VImage> MyImageFactoryFromImage;
    typedef MyImageFactoryFromImage::OutputImage OutputImage;
    MyImageFactoryFromImage imageFactoryFromImage(image);
    
    typedef ImageCacheReadPolicyFIFO<OutputImage, MyImageFactoryFromImage> MyImageCacheReadPolicyFIFO;
    typedef ImageCacheWritePolicyWT<OutputImage, MyImageFactoryFromImage> MyImageCacheWritePolicyWT;
    MyImageCacheReadPolicyFIFO imageCacheReadPolicyFIFO(imageFactoryFromImage, 2);
    MyImageCacheWritePolicyWT imageCacheWritePolicyWT(imageFactoryFromImage);
    
    typedef TiledImageFromImage<VImage, MyImageFactoryFromImage, MyImageCacheReadPolicyFIFO, MyImageCacheWritePolicyWT> MyTiledImageFromImage;
    MyTiledImageFromImage tiledImageFromImage(image, imageFactoryFromImage, imageCacheReadPolicyFIFO, imageCacheWritePolicyWT, 4);
    
    typedef MyTiledImageFromImage::OutputImage OutputImage;
    
    trace.info() << "Read value for Point 1,1,1: " << tiledImageFromImage(Z3i::Point(1,1,1)) << endl;
    nbok += (tiledImageFromImage(Z3i::Point(1,1,1)) == 1) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "Read value for Point 4,4,4: " << tiledImageFromImage(Z3i::Point(4,4,4)) << endl;
    nbok += (tiledImageFromImage(Z3i::Point(4,4,4)) == 64) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.info() << "Read value for Point 4,3,2: " << tiledImageFromImage(Z3i::Point(4,3,2)) << endl;
    nbok += (tiledImageFromImage(Z3i::Point(4,3,2)) == 28) ? 1 : 0; 
    nb++;
    
    trace.info() << "(" << nbok << "/" << nb << ") " << endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
    trace.beginBlock ( "Testing class TiledImageFromImage" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << endl;

    bool res = testSimple() && test3d(); // && ... other tests

    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
