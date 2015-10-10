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
 * @file testMagickReader.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/30
 *
 * Functions for testing class MagickReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/ColorBrightnessColorMap.h"

#include "DGtal/io/readers/MagickReader.h"
#include "DGtal/io/boards/Board2D.h"
#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class MagickReader.
////////////////////////////////////////////////////////////////
///////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testMagickReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing MagickReader ..." );

  typedef SpaceND<2> Space2;
  typedef HyperRectDomain<Space2> TDomain;
  
  //Default image selector = STLVector
  typedef ImageSelector<TDomain, unsigned char>::Type Image;

  std::string filename = testPath + "samples/color64.png";

  trace.info()<<"Importing: "<<filename<<endl;

  MagickReader<Image> reader;
  Image img = reader.importImage( filename );

  nbok += img.isValid() ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "img.isValid() == true"
         << std::endl;

  nbok += img.extent() == Image::Vector( 64, 64 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "img.extent() = " << img.extent() 
         << "( == {64,64} )"
         << std::endl;

  Board2D board;
  typedef HueShadeColorMap<unsigned char,2> HueTwice;
  

  board.setUnit(LibBoard::Board::UCentimeter);

  Display2DFactory::drawImage<HueTwice>(board, img, (unsigned char)0, (unsigned char)255);
  board.saveSVG("testMagick-export.svg");
    
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class MagickReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testMagickReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
