#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer3D.h"

#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageHelper.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

typedef PolyscopeViewer3D<> MyViewer;

struct hueFct{
 inline
 unsigned int operator() (unsigned char aVal) const
  {
    return (aVal << 16) | (aVal << 8) | (aVal);
    HueShadeColorMap<unsigned char>  hueShade(0,255);
    Color col = hueShade((unsigned char)aVal);
    return  (((unsigned int) col.red()) <<  16)| (((unsigned int) col.green()) << 8)|((unsigned int) col.blue());
  }
};


int main( int argc, char** argv )
{
    MyViewer viewer;
 
    typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  unsigned char > Image3D;
    
    std::string inputFilename = "/home/bdoignies/Documents/DGtal/examples/samples/lobster.vol";
    Image3D imageVol = VolReader<Image3D>::importVol(inputFilename);

    Z3i::Point ptLow (100, 100, 20);
    Z3i::Point ptUpp (200, 200, 40);
    Z3i::Domain subDomain(ptLow, ptUpp);
  
    Z3i::Point ptLow2 (220, 50, 10);
    Z3i::Point ptUpp2 (260, 100, 20);
    Z3i::Domain subDomain2(ptLow2, ptUpp2);

    Image3D imageCrop(subDomain);
    Image3D imageCrop2(subDomain2);

    for(Z3i::Domain::ConstIterator it= imageVol.domain().begin(), itend = imageVol.domain().end(); it != itend; ++it){
      if(imageVol(*it)>140)
        viewer << *it;
      Z3i::Point pt = *it;
      if(pt[0]>=ptLow[0] && pt[1] >= ptLow[1] && pt[2] >= ptLow[2] &&
         pt[0]<=ptUpp[0] && pt[1] <= ptUpp[1] && pt[2] <= ptUpp[2]){
        imageCrop.setValue(*it, imageVol(*it));
      }

      if(pt[0]>=ptLow2[0] && pt[1] >= ptLow2[1] && pt[2] >= ptLow2[2] &&
         pt[0]<=ptUpp2[0] && pt[1] <= ptUpp2[1] && pt[2] <= ptUpp2[2]){
        imageCrop2.setValue(*it, imageVol(*it));
      }
    }
    viewer << imageCrop;
    viewer << SetMode3D(imageCrop.className(), "BoundingBox");
    //! [ExampleViewer3D3DImagesDisplayImagesColor]
    viewer << AddTextureImage3DWithFunctor<Image3D, hueFct, Z3i::Space, Z3i::KSpace> (imageCrop2, hueFct(), RGBMode);
    viewer << MyViewer::updateDisplay;
    //! [ExampleViewer3D3DImagesDisplayImagesColor]

    viewer << MyViewer::updateDisplay;
    viewer.show();
    return 0;
}
