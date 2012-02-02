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
 * @file testImagesSetsUtilities.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/26
 *
 * Functions for testing class ImagesSetsUtilities.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/images/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageSelector.h"

#include "DGtal/helpers/StdDefs.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ImagesSetsUtilities.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testImagesSetsUtilities()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing ImageFormSet ..." );
  
  Point a(0,0);
  Point b(23,435);
  Point c(12,12);

  typedef ImageContainerBySTLVector<Domain,int> Image;
  Image image(a,b);
  DigitalSet aSet(Domain(a,b));
  aSet.insert(c);
  
  //test of the append method
  ImageFromSet<Image>::append<DigitalSet>(image, aSet, 128);
  nbok += (image(c) == 128) ? 1 : 0; 
  nb++;

  //test of the create method
  Image imageBis =   ImageFromSet<Image>::create<DigitalSet>(aSet, 128);
  nbok += (imageBis(c) == 128) ? 1 : 0; 
  nb++;


  
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

bool testSetFromImage()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing SetFromImage ..." );
  
  Point a(0,0);
  Point b(23,43);
  Point c(12,12);

  typedef ImageContainerBySTLVector<Domain,int> Image;
  Image image(a,b);
  image.setValue(c,128);
  
  DigitalSet aSet(Domain(a,b));
  
  //test of the append method
  SetFromImage<DigitalSet>::append<Image>(aSet, image, 0,255);

  trace.info()<< "DigitalSet:= ";
  for(DigitalSet::ConstIterator it= aSet.begin(),itend=aSet.end();
      it != itend;
      ++it)
    trace.info() << *it<< " ";
  trace.info()<<endl;

  nbok += (aSet.find(c) != aSet.end() ) ? 1 : 0; 
  nb++;



  trace.beginBlock ( "Testing SetFromImage using pgm test file ..." );
  
  std::string filename = testPath + "samples/circleR10modif.pgm";
  Image image2 = PNMReader<Image>::importPGM( filename ); 
  Z2i::DigitalSet setFromImg (image2.domain());
  SetFromImage<Z2i::DigitalSet>::append<Image>(setFromImg, image2, 0, 255);

  
  for( Z2i::DigitalSet::Iterator it= setFromImg.begin(); 
       it!=setFromImg.end(); it++){
    trace.info()<< *it << endl;
  }
  trace.info()<< "Size=" << setFromImg.size();
  
  Image setImage = ImageFromSet<Image>::create(setFromImg, 1,
                 false, 
                  setFromImg.begin(), 
                  setFromImg.end());
  
  trace.info() << "Image at 0,12:" << setImage(Point(0,12))<< endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ImagesSetsUtilities" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testImagesSetsUtilities() && testSetFromImage(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
