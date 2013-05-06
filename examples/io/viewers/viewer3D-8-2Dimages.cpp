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
 * @file viewer3D-7-2Dimages.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/04/29
 *
 * An example file named viewer3D-8-2Dimages.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageHelper.h"
#include "ConfigExamples.h"

#include <QtGui/qapplication.h>
#include "DGtal/io/viewers/Viewer3D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{

  typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  unsigned char > Image3D;
  typedef DGtal::ImageContainerBySTLVector<DGtal::Z2i::Domain,  unsigned char > Image2D;
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();
  std::string inputFilename = examplesPath + "samples/lobster.vol"; 
  Image3D imageVol = VolReader<Image3D>::importVol(inputFilename);
  

  //! [ExampleViewer3D2DImagesExtractImages]
  // Extracting the 2D images from the 3D one and from a given dimension.
  // First image  the teenth Z slice (dim=2) 
  typedef DGtal::ConstImageAdapter<Image3D, DGtal::Z2i::Domain, DGtal::AddOneDimensionDomainFunctor< DGtal::Z3i::Point>,
				   Image3D::Value,  DGtal::DefaultFunctor >  MySliceImageAdapter;

  // Define the functor to recover a 2D domain from the 3D one in the Z direction (2):
  DGtal::MinusOneDimensionDomainFunctor<DGtal::Z2i::Point>  transTo2DdomainFunctorZ(2);
  DGtal::Z2i::Domain domain2DZ((transTo2DdomainFunctorZ.operator()(imageVol.domain().lowerBound())), 
			       (transTo2DdomainFunctorZ.operator()(imageVol.domain().upperBound())));

  // Define the functor to associate 2D coordinates to the 3D one by giving the direction Z (2) and the slide numnber (10):
  DGtal::AddOneDimensionDomainFunctor<DGtal::Z3i::Point> aSliceFunctorZ(2, 10);

  // We can now obtain the slice image (a ConstImageAdapter):
  MySliceImageAdapter aSliceImageZ(imageVol, domain2DZ, aSliceFunctorZ, DGtal::DefaultFunctor() );

  // Second image  the fiftieth Y slice (dim=1) 
  // Define the functor to recover a 2D domain from the 3D one in the Y direction (1):
  DGtal::MinusOneDimensionDomainFunctor<DGtal::Z2i::Point>  transTo2DdomainFunctorY(1);
  DGtal::Z2i::Domain domain2DY((transTo2DdomainFunctorY.operator()(imageVol.domain().lowerBound())), 
			      (transTo2DdomainFunctorY.operator()(imageVol.domain().upperBound())));

  // Define the functor to associate 2D coordinates to the 3D one by giving the direction Y (1) and the slide numnber (50):
  DGtal::AddOneDimensionDomainFunctor<DGtal::Z3i::Point> aSliceFunctorY(1, 50);

  // We can now obtain the slice image (a ConstImageAdapter):
  MySliceImageAdapter aSliceImageY(imageVol, domain2DY, aSliceFunctorY, DGtal::DefaultFunctor() );
  //! [ExampleViewer3D2DImagesExtractImages]

  //! [ExampleViewer3D2DImagesDisplayImages]
  viewer <<  aSliceImageZ;
  viewer <<  aSliceImageY;
  //! [ExampleViewer3D2DImagesDisplayImages]

  //! [ExampleViewer3D2DModifImages]
  viewer <<  DGtal::UpdateImagePosition(1, DGtal::Display3D::yDirection, 0.0,  50.0, 0.0);
  viewer << DGtal::UpdateImageData<MySliceImageAdapter>(0, aSliceImageZ, 0, 0, 10);
  viewer << Viewer3D::updateDisplay;
 //! [ExampleViewer3D2DModifImages]

  
  
return application.exec();

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
