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
 * @file test3DImageView.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Functions for testing class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/Color.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/math/BasicMathFunctions.h"

#include "ConfigTest.h"

#include <limits>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

//////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, unsigned char>  imageNG;
  typedef DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, unsigned int>  imageCol;

  PolyscopeViewer<> viewer;

  trace.beginBlock("Testing Viewer with 3D Image View ");

  Point p1( 0, 0, 0 );
  Point p2( 125, 188, 0 );
  Point p3( 30, 30, 30 );

  std::string filename =  testPath + "samples/church-small.pgm";
  std::string filename3 =  testPath + "samples/color64.ppm";

  imageNG image = DGtal::PGMReader<imageNG>::importPGM(filename);
  imageNG image2 = DGtal::GenericReader<imageNG>::import(filename);
  imageCol image3 = DGtal::GenericReader<imageCol>::import(filename3);

  viewer.draw(image2, "Image 2");
  viewer.draw(image, "Image");
  viewer.draw(image3, "Image 3");

  viewer.data["Image 2"].transform.translate(Eigen::Vector3d{50, 50, 50});
  viewer.data["Image 3"].transform.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d{0, 1, 0}));

  viewer << p1 << p2 << p3;

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

