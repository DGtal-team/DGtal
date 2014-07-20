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
 * @file testRawReader.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/29
 *
 * Functions for testing class RawReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/writers/PGMWriter.h"
#include "DGtal/io/readers/RawReader.h"

#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class RawReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testRawReader2D()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing Raw reader ..." );
  
  typedef SpaceND<2> Space2;
  typedef HyperRectDomain<Space2> TDomain;
  typedef TDomain::Vector Vector;

  typedef SpaceND<3> Space3;
  typedef HyperRectDomain<Space3> TDomain3;
  typedef TDomain3::Vector Vector3;
  
  //Default image selector = STLVector
  typedef ImageSelector<TDomain, unsigned char>::Type Image;
  typedef ImageSelector<TDomain3, unsigned int>::Type Image32;
  
  std::string filename = testPath + "samples/raw2D-64x64.raw";
  
  Vector ext(16,16);
  
  Image image = RawReader<Image>::importRaw8( filename , ext);

  ///FIXME: check io errors
  trace.info() << image <<endl;

  //export
  PGMWriter<Image>::exportPGM("export-raw-reader.pgm",image);
  
  /// @todo re-import the PGM and compare with raw2D-64x64

  std::string filename2 = testPath + "samples/raw32bits5x5x5.raw";
  Vector3 ext2(5,5,5);
  Image32 image2 = RawReader<Image32>::importRaw32( filename2, ext2 );
  TDomain3::Point pointA(2,3,4);
  
  trace.info() << "Value of point " << pointA <<  " value :"  << image2(pointA) << std::endl;
  
  
  nbok += image2(pointA)== 250000*2*3*4 ? 1 : 0; 
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
  trace.beginBlock ( "Testing class RawReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testRawReader2D(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
