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
 * @file test3DImage3DView.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/04/29
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
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/math/BasicMathFunctions.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::ImageContainerBySTLVector< DGtal::Z3i::Domain, unsigned char>  Image3D;

  PolyscopeViewer viewer;
  trace.beginBlock("Testing Viewer with display of 3D Image  ");

  Point p1( 0, 0, 0 );
  Point p2( 125, 188, 0 );
  Point p3( 30, 30, 30 );

  std::string filename =  testPath + "samples/lobsterCroped.vol";

  Image3D image3d =  VolReader<Image3D>::importVol(filename);
  
  viewer << Color(255, 255, 200);
  viewer << image3d;
  // Extract some slice images:
  // Get the 2D domain of the slice:
  DGtal::functors::Projector<DGtal::Z2i::Space>  invFunctor; invFunctor.initRemoveOneDim(2);
  DGtal::Z2i::Domain domain2D(invFunctor(image3d.domain().lowerBound()),
         invFunctor(image3d.domain().upperBound()));

  typedef DGtal::ConstImageAdapter<Image3D, DGtal::Z2i::Domain,  DGtal::functors::Projector< Z3i::Space>,
                                Image3D::Value,  functors::Identity >  SliceImageAdapter;
  functors::Identity idV;
  functors::Projector<DGtal::Z3i::Space> aSliceFunctorZ(5); aSliceFunctorZ.initAddOneDim(2);

  SliceImageAdapter sliceImageZ(image3d, domain2D, aSliceFunctorZ, idV);

  std::string name = viewer.draw(sliceImageZ);
  viewer.data[name].transform.translate(Eigen::Vector3d(0, 0, -10));

  viewer << p1 << p2 << p3;

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
  return 0;


}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

