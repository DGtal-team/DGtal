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
 * @file testPNMReader.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/04/29
 *
 * Functions for testing class PNMReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PNMReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testPNMReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;  
  trace.beginBlock ( "Testing pgm reader ..." );
  nbok += true ? 1 : 0; 
  nb++;
  std::string filename = testPath + "samples/circleR10.pgm";

  trace.info() << "Loading filename: "<< filename<<std::endl;

  typedef ImageSelector < Z2i::Domain, unsigned int>::Type Image;
  Image image = PNMReader<Image>::importPGM( filename ); 
  
  Z2i::DigitalSet set2d (image.domain());
  SetFromImage<Z2i::DigitalSet>::append<Image>(set2d, image, 0, 255);
   
  Board2D board;
  board << image.domain() << set2d; // display domain and set
  
  board.saveEPS( "testPNMReader.eps");
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();  
  return nbok == nb;
}
/**
 * Example of a test. To be completed.
 *
 */
bool testPNM3DReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;  
  trace.beginBlock ( "Testing pgm3D reader ..." );
  nbok += true ? 1 : 0; 
  nb++;
  std::string filename = testPath + "samples/simple.pgm3d";

  trace.info() << "Loading filename: "<< filename<<std::endl;

  typedef ImageSelector < Z3i::Domain, unsigned int>::Type Image;
  Image image = PNMReader<Image>::importPGM3D( filename ); 
  
  trace.info() << "Image 3D = "<<image<<std::endl;
  
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class PNMReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testPNMReader() && testPNM3DReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
