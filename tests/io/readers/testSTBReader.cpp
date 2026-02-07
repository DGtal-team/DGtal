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
#include "DGtal/images/ImageSelector.h"

#include "DGtal/io/writers/PPMWriter.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class STBReader.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing STBReader" )
{
  SECTION("Testing feature io/readers of STBReader (Grayscale PNG)")
  {
    std::string filename = testPath + "samples/contourS.png";
    typedef ImageSelector < Z2i::Domain, Color>::Type Image;
    Image image = STBReader<Image>::import( filename );
    CAPTURE(image);

    PPMWriter<Image>::exportPPM("testwriter.ppm", image);
    CHECK( image.isValid());
  }

  SECTION("Testing feature io/readers of STBReader (PPM color)")
  {
    std::string filename = testPath + "samples/color64.";
    typedef ImageSelector < Z2i::Domain, Color>::Type Image;

    Image image = STBReader<Image>::import( filename+"ppm" );
    PPMWriter<Image>::exportPPM("testwriterColor.ppm", image);
    CHECK( image.isValid());
  }

  SECTION("Testing all file formats")
  {
    std::string filename = testPath + "samples/color64.";
    typedef ImageSelector < Z2i::Domain, Color>::Type Image;

    Image imagePPM = STBReader<Image>::import( filename+"ppm" );
    CHECK( imagePPM.isValid());
    Image imageJPG = STBReader<Image>::import( filename+"jpg" );
    CHECK( imageJPG.isValid());
    Image imageTGA = STBReader<Image>::import( filename+"tga" );
    CHECK( imageTGA.isValid());
    Image imageBMP = STBReader<Image>::import( filename+"bmp" );
    CHECK( imageBMP.isValid());
    Image imagePNG = STBReader<Image>::import( filename+"png" );
    CHECK( imagePNG.isValid());
    Image imageGIF = STBReader<Image>::import( filename+"gif" );
    CHECK( imageGIF.isValid());

    //For lossless compression formats
    CHECK( imageBMP == imagePPM );
    CHECK( imageTGA == imagePPM );
    CHECK( imagePNG == imagePPM );
  }
}

/** @ingroup Tests **/
