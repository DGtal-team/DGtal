/// @file io/display3DToOFF.cpp
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/Display3D.h"

#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"


using namespace std;
using namespace DGtal;
using namespace Z3i;


int main( int /*argc*/, char** /*argv*/ )
{
  std::string inputFilename = examplesPath + "samples/Al.100.vol";
  //! [ExampleDisplay3DToOFF]
  Display3D<Space, KSpace> viewer;
  typedef ImageSelector < Z3i::Domain, int>::Type Image;
  Image image = VolReader<Image>::importVol(inputFilename);
  Z3i::DigitalSet set3d (image.domain());
  SetFromImage<Z3i::DigitalSet>::append<Image>(set3d, image, 0,255);

  viewer << set3d ;
  viewer >> "exportMeshToOFF.off";
 //! [ExampleDisplay3DToOFF]

  return 0;
}
