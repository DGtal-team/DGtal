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
 * @file testVolReader.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Functions for testing class VolReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/ColorBrightnessColorMap.h"
#include "DGtal/io/writers/VolWriter.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VolReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testVolReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing VolReader ..." );

  typedef SpaceND<3> Space4Type;
  typedef HyperRectDomain<Space4Type> TDomain;
  
  //Default image selector = STLVector
  typedef ImageSelector<TDomain, unsigned char>::Type Image;
  
  
  std::string filename = testPath + "samples/cat10.vol";
  Image image = VolReader<Image>::importVol( filename );
  
  trace.info() << image <<endl;
  
  nbok += true ? 1 : 0; 
  nb++;

  unsigned int nbval=0;
  for(Image::ConstIterator it=image.begin(), itend=image.end();
      it != itend;   ++it)
    if ( (*it) != 0)
      nbval++;
  
  trace.info() << "Number of points with (val!=0)  = "<<nbval<<endl;

  nbok += ( nbval == 8043)  ? 1 : 0; 
  nb++;

  VolWriter<Image>::exportVol("catenoid-export.vol",image);

  nbok += ( true )  ? 1 : 0; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}


bool testIOException()
{
   unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing VolReader ..." );

  typedef SpaceND<3> Space4Type;
  typedef HyperRectDomain<Space4Type> TDomain;
  
  //Default image selector = STLVector
  typedef ImageSelector<TDomain, unsigned char>::Type Image;
  
  
  std::string filename = testPath + "samples/null.vol";
  try
    {
      Image image = VolReader<Image>::importVol( filename );
    }
  catch(exception& e)
    {
      trace.info() << "Exception catched. Message : "<< e.what()<<endl;
    }
  
  
 
  nbok += ( true )  ? 1 : 0; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class VolReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testVolReader() && testIOException(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
