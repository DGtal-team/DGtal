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

//#define DEBUG_VERBOSE

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
bool testSimple()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock("Testing simple RestrictedImage");
    
    typedef ImageContainerBySTLVector<Z2i::Domain, int> VImage;

    VImage image(Z2i::Domain(Z2i::Point(0,0), Z2i::Point(10,10)));
    for (VImage::Iterator it = image.begin(); it != image.end(); ++it)
        *it = 10;

    trace.info() << "Original image: " << image << endl;

    Z2i::Domain domain(Z2i::Point(2,2), Z2i::Point(4,4));
    typedef RestrictedImage<Z2i::Domain, VImage> MyRestrictedImage;

    MyRestrictedImage restimage(domain, image);
    trace.info() << "Restricted Image: " << restimage << "  " << restimage.domain() << std::endl;

    nbok += (restimage(Z2i::Point(3,3)) == 10) ? 1 : 0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ") "
    << " read access" << endl;

    restimage.setValue(Z2i::Point(3,3), 5);
    nbok += (restimage(Z2i::Point(3,3)) == 5) ? 1 : 0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ") "
    << " write on restricted Image"  << endl;

    nbok += ((image)(Z2i::Point(3,3)) == 5) ? 1 : 0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ") "
    << " writed on original image" << endl;

    trace.warning()<< "Original image at (3,3) = "<< (image)(Z2i::Point(3,3)) <<std::endl;
    
    trace.endBlock();
    
    return nbok == nb;
}

bool testRestrictedImage()
{
    unsigned int nbok = 0;
    unsigned int nb = 0;

    trace.beginBlock ("Testing RestrictedImage");

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

    MyRestrictedImage bell_tower( domain_bell_tower, image);

    trace.info() << "RestrictedImage: " << bell_tower << "  " << bell_tower.domain() << std::endl;

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

    MyRestrictedImage cars( domain_cars, image);

    trace.info() << "RestrictedImage: " << cars << "  " << cars.domain() << std::endl;

    nbok += cars.isValid() ? 1 : 0;
    nb++;

    aBoard.clear();
    Display2DFactory::drawImage<Gray>(aBoard, cars, (unsigned char)0, (unsigned char)255);
    aBoard.saveSVG("cars.svg");
#ifdef WITH_CAIRO
    aBoard.saveCairo("cars.png", Board2D::CairoPNG);
#endif

    // 3) fill 255 for 'bell_tower' image
    typename MyRestrictedImage::Domain::ConstIterator bt_it = bell_tower.domain().begin();
    typename MyRestrictedImage::Domain::ConstIterator bt_itEnd = bell_tower.domain().end();
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
    typename MyRestrictedImage::Domain::ConstIterator c_it = cars.domain().begin();
    typename MyRestrictedImage::Domain::ConstIterator c_itEnd = cars.domain().end();
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
    
    // 5) fill 0 (only for one pixel on two) for on 'floor_lamp' image
    Z2i::Point p5( 56, 187-154/*108*/ );
    Z2i::Point p6( 68, 187-108/*154*/ );
    Z2i::Domain domain_floor_lamp( p5, p6 );

    // --- DigitalSetDomain
    typedef DigitalSetSelector < Z2i::Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type SpecificSet;
    BOOST_CONCEPT_ASSERT(( CDigitalSet< SpecificSet > ));

    SpecificSet mySet( domain_floor_lamp );

    unsigned int i = 0;
    for ( Z2i::Domain::ConstIterator it = domain_floor_lamp.begin() ; 
      it != domain_floor_lamp.end();
      ++it, ++i )
    {
      // insertNew is very important for vector container.
      if (i%2)
        mySet.insertNew( *it );
      i++;
    }
    
    DigitalSetDomain<SpecificSet> my_specific_domain_floor_lamp(mySet);
    // --- DigitalSetDomain
    
    MyRestrictedImage floor_lamp( domain_floor_lamp, image);
    //MyRestrictedImage floor_lamp( my_specific_domain_floor_lamp, image);

    trace.info() << "RestrictedImage: " << floor_lamp << "  " << floor_lamp.domain() << std::endl;

    nbok += floor_lamp.isValid() ? 1 : 0;
    nb++;
    
    typename MyRestrictedImage::Domain::ConstIterator f_it = floor_lamp.domain().begin();
    typename MyRestrictedImage::Domain::ConstIterator f_itEnd = floor_lamp.domain().end();
    for (; f_it != f_itEnd; ++f_it)
    {
        floor_lamp.setValue(*f_it, 0);
    }
    
    aBoard.clear();
    Display2DFactory::drawImage<Gray>(aBoard, floor_lamp, (unsigned char)0, (unsigned char)255);
    aBoard.saveSVG("floor_lamp.svg");
#ifdef WITH_CAIRO
    aBoard.saveCairo("floor_lamp.png", Board2D::CairoPNG);
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

    bool res = testSimple() && testRestrictedImage(); // && ... other tests

    trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    trace.endBlock();
    return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
