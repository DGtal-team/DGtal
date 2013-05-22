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
 * @file exampleTiledImageFromImage.cpp
 * @ingroup Examples
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/23
 *
 * @brief An example file for tiledImageFromImage.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/io/boards/Board2D.h"

//! [include]
#include "DGtal/io/colormaps/HueShadeColorMap.h"

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/TiledImageFromImage.h"
//! [include]

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

int main( int argc, char** argv )
{
    trace.beginBlock("ORIGINAL image");
    
    Board2D aBoard;
    
//! [def]
    typedef HueShadeColorMap<int> HueShade;     // a simple HueShadeColorMap varying on 'int' values
//! [def]
    
//! [image_creation]
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;
    VImage image(Z2i::Domain(Z2i::Point(1,1), Z2i::Point(16,16)));
//! [image_creation]
    
//! [image_filling]
    int i = 1;
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = i++;
//! [image_filling]
        
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, image, (int)0, (int)255);
    aBoard.saveSVG("tiledImageFromImage-image.svg");
#ifdef WITH_CAIRO
    aBoard.saveCairo("tiledImageFromImage-image.png", Board2D::CairoPNG);
#endif
    
    trace.info() << "ORIGINAL image: " << image << endl;
    
    trace.endBlock();
    
    // ---
    
    trace.beginBlock("tiledImageFromImage");
 
//! [TiledImageFromImage_creation]
    // here we create an image factory
    typedef ImageFactoryFromImage<VImage> MyImageFactoryFromImage;
    typedef MyImageFactoryFromImage::OutputImage OutputImage;
    MyImageFactoryFromImage imageFactoryFromImage(image);
    
    // here we create read and write policies
    typedef ImageCacheReadPolicyFIFO<OutputImage, MyImageFactoryFromImage> MyImageCacheReadPolicyFIFO;
    typedef ImageCacheWritePolicyWT<OutputImage, MyImageFactoryFromImage> MyImageCacheWritePolicyWT;
    MyImageCacheReadPolicyFIFO imageCacheReadPolicyFIFO(imageFactoryFromImage, 2);
    MyImageCacheWritePolicyWT imageCacheWritePolicyWT(imageFactoryFromImage);
    
    
    // here we create the TiledImageFromImage
    typedef TiledImageFromImage<VImage, MyImageFactoryFromImage, MyImageCacheReadPolicyFIFO, MyImageCacheWritePolicyWT> MyTiledImageFromImage;
    MyTiledImageFromImage tiledImageFromImage(image, imageFactoryFromImage, imageCacheReadPolicyFIFO, imageCacheWritePolicyWT, 4);
//! [TiledImageFromImage_creation]
    
    trace.info() << "tiledImageFromImage image: " << tiledImageFromImage << endl;
    
    typedef MyTiledImageFromImage::OutputImage OutputImage;
    /*VImage*/OutputImage::Value aValue;
    
    trace.endBlock();
    
    // ---

    trace.beginBlock("read and write on MyTiledImageFromImage");
    
//! [TiledImageFromImage_read42]
    trace.info() << "Read value for Point 4,2: " << tiledImageFromImage(Z2i::Point(4,2)) << endl;
//! [TiledImageFromImage_read42]    
    
//! [TiledImageFromImage_read106]
    trace.info() << "Read value for Point 10,6: " << tiledImageFromImage(Z2i::Point(10,6)) << endl;
//! [TiledImageFromImage_read106]
    
//! [TiledImageFromImage_write117]
    aValue = 1; tiledImageFromImage.setValue(Z2i::Point(11,7), aValue);
    trace.info() << "Write value for Point 11,7: " << aValue << endl;
//! [TiledImageFromImage_write117]
    
//! [TiledImageFromImage_read23]
    trace.info() << "Read value for Point 2,3: " << tiledImageFromImage(Z2i::Point(2,3)) << endl;
//! [TiledImageFromImage_read23]
    
//! [TiledImageFromImage_read161]
    trace.info() << "Read value for Point 16,1: " << tiledImageFromImage(Z2i::Point(16,1)) << endl;
//! [TiledImageFromImage_read161]
    
//! [TiledImageFromImage_write161]
    aValue = 128; tiledImageFromImage.setValue(Z2i::Point(16,1), aValue);
    trace.info() << "Write value for Point 16,1: " << aValue << endl;
//! [TiledImageFromImage_write161]
    
    trace.info() << "  Point 16,1 on ORIGINAL image, value: " << image(Z2i::Point(16,1)) << endl;
    
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, image, (int)0, (int)255);
    aBoard.saveSVG("tiledImageFromImage-image2.svg");
#ifdef WITH_CAIRO
    aBoard.saveCairo("tiledImageFromImage-image2.png", Board2D::CairoPNG);
#endif
    
    trace.endBlock();
    
    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
