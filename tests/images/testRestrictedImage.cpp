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
 * @file testRestrictedImage.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/07
 * 
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/09/04
 *
 * Functions for testing class RestrictedImage.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/RestrictedImage.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/io/boards/Board2D.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class RestrictedImage.
///////////////////////////////////////////////////////////////////////////////
bool testRestrictedImage()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing RestrictedImage Create ..." );
  
  typedef ImageContainerBySTLVector< Z2i::Domain, unsigned char> VImage;
  typedef GrayscaleColorMap<unsigned char> Gray;
  
  string filename = testPath + "samples/church-small.pgm";
  VImage image = PNMReader<VImage>::importPGM(filename); 
  trace.info() << "Imported image: " << image << endl;
  
  Board2D aBoard;
  
  Display2DFactory::drawImage<Gray>(aBoard, image, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("church.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("church.png", Board2D::CairoPNG);
#endif
  
  typedef RestrictedImage<Z2i::Domain, VImage > MyRestrictedImage;
  BOOST_CONCEPT_ASSERT(( CImage< MyRestrictedImage > ));
  
  nbok += true ? 1 : 0;
  nb++;
  
  // 1) bell_tower
  Z2i::Point p1( 43, 187-80/*10*/ );
  Z2i::Point p2( 73, 187-10/*80*/ );
  Z2i::Domain domain_bell_tower( p1, p2 );
  
  MyRestrictedImage bell_tower( domain_bell_tower, image );
  
  trace.info() << "RestrictedImage: " << bell_tower << endl;

  nbok += bell_tower.isValid() ? 1 : 0;
  nb++;
  
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, bell_tower, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("bell_tower.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("bell_tower.png", Board2D::CairoPNG);
#endif
  
  // 2) cars
  Z2i::Point p3( 0, 187-138/*115*/ );
  Z2i::Point p4( 58, 187-115/*138*/ );
  Z2i::Domain domain_cars( p3, p4 );
  
  MyRestrictedImage cars( domain_cars, image );
  
  trace.info() << "RestrictedImage: " << cars << endl;

  nbok += cars.isValid() ? 1 : 0;
  nb++;
  
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, cars, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("cars.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("cars.png", Board2D::CairoPNG);
#endif
  
  // 3) fill 255 for 'bell_tower' image
  typename VImage::Domain::ConstIterator bt_it = bell_tower.domain().begin();
  typename VImage::Domain::ConstIterator bt_itEnd = bell_tower.domain().end();
  for (; bt_it != bt_itEnd; ++bt_it)
  {
      bell_tower.setValue(*bt_it, 255);
  }
  
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, bell_tower, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("bell_tower_after_filling.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("bell_tower_after_filling.png", Board2D::CairoPNG);
#endif
  
  // 4) fill 55 for 'cars' image
  typename VImage::Domain::ConstIterator c_it = cars.domain().begin();
  typename VImage::Domain::ConstIterator c_itEnd = cars.domain().end();
  for (; c_it != c_itEnd; ++c_it)
  {
      cars.setValue(*c_it, 55);
  }
  
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, cars, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("cars_after_filling.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("cars_after_filling.png", Board2D::CairoPNG);
#endif
  
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, image, (unsigned char)0, (unsigned char)255);
  aBoard.saveSVG("church_after_filling.svg");
#ifdef WITH_CAIRO
  aBoard.saveCairo("church_after_filling.png", Board2D::CairoPNG);
#endif
  
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "true == true" << endl;
  trace.endBlock();

  return nbok == nb;
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class RestrictedImage" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testRestrictedImage(); // && ... other tests

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
