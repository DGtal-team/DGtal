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
 * @file
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/14
 *
 * Functions for testing class STBReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/STBReader.h"
#include "DGtal/io/writers/STBWriter.h"
#include "DGtal/images/ImageSelector.h"

#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/writers/PPMWriter.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class STBReader.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing STBWriter" )
{
  SECTION("Testing feature io/readers of STBWriter (Grayscale PNG)")
  {
    std::string filename = testPath + "samples/contourS.png";
    typedef ImageSelector < Z2i::Domain, Color>::Type Image;
    Image image = STBReader<Image>::import( filename );
    CAPTURE(image);
    PPMWriter<Image>::exportPPM("contourS-export.ppm", image);

    STBWriter<Image>::exportPNG("contourS-export.png", image);
    CHECK( image.isValid());
  }

  SECTION("Testing feature io/readers of STBWriter (PNG)")
  {
    std::string filename = testPath + "samples/color64.png";
    typedef ImageSelector < Z2i::Domain, Color>::Type Image;
    Image image = STBReader<Image>::import( filename );
    CHECK( image.isValid());
    PPMWriter<Image>::exportPPM("color64-export.ppm", image);
    STBWriter<Image>::exportTGA("color64-export.tga", image);
    STBWriter<Image>::exportJPG("color64-export.jpg", image);
    STBWriter<Image>::exportBMP("color64-export.bmp", image);
  }

  SECTION("Testing scalar functor (PNG)")
  {
    typedef ImageSelector < Z2i::Domain, int>::Type Image;
    Image image(Z2i::Domain(Z2i::Point(0,0),  Z2i::Point(8,8)));

    image.setValue(Z2i::Point(3,3), 10);
    image.setValue(Z2i::Point(1,1), 1);
    image.setValue(Z2i::Point(7,7), 20);

    // Creating colormap.
    GradientColorMap<int> cmap_grad( 0, 30 );
    cmap_grad.addColor( Color( 50, 50, 255 ) );
    cmap_grad.addColor( Color( 255, 0, 0 ) );
    cmap_grad.addColor( Color( 255, 255, 10 ) );

    CHECK( image.isValid());
    STBWriter<Image,GradientColorMap<int>>::exportPNG("scalar-export.jpg", image, cmap_grad );
  }

}

/** @ingroup Tests **/
