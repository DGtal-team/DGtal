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
 * @file testImageHelper.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/04/24
 *
 * Functions for testing class ImageHelper.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/writers/PGMWriter.h"
#include "DGtal/images/ImageSelector.h"

#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ImageHelper.
///////////////////////////////////////////////////////////////////////////////
/**
 * Test of the function extractLowerDimImage for the extraction of 2D images from 3D image.
 *
 */
bool testImageHelper()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  std::string filename = testPath + "samples/cat10.vol";

  trace.beginBlock ( "Testing block ..." );
  typedef ImageSelector < Z3i::Domain, unsigned char>::Type Image3D;
  typedef ImageSelector < Z2i::Domain, unsigned char>::Type Image2D;
  Image3D image = VolReader<Image3D>::importVol( filename ); 
  Image2D slice2D_X = DGtal::extractLowerDimImage<Image3D, Image2D> (image, 0, 20);
  Image2D slice2D_Y = DGtal::extractLowerDimImage<Image3D, Image2D> (image, 1, 20);
  Image2D slice2D_Z = DGtal::extractLowerDimImage<Image3D, Image2D> (image, 2, 20);  
  bool res= true;
  res &= PGMWriter<Image2D>::exportPGM("exportedSlice2DDimX.pgm",slice2D_X);
  res &= PGMWriter<Image2D>::exportPGM("exportedSlice2DDimY.pgm",slice2D_Y);
  res &= PGMWriter<Image2D>::exportPGM("exportedSlice2DDimZ.pgm",slice2D_Z);
  nbok += res ? 1 : 0; 
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
  trace.beginBlock ( "Testing class ImageHelper" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testImageHelper(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
