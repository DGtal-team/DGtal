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
 * @file testGenericWriter.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Functions for testing class GenericWriter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GenericWriter.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testGenericWriter()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain, unsigned char > Image3D;
  typedef DGtal::ImageContainerBySTLVector<DGtal::Z2i::Domain,  unsigned char > Image2D;
  trace.beginBlock ( "Testing block ..." );
  std::string filenameImage1 = testPath + "samples/cat10.pgm3d";    
  trace.info() << "Reading 3D image  ... ";  
  Image3D anImportedImage1 = DGtal::GenericReader<Image3D>::import(filenameImage1);  
  trace.info() <<"[done]"  << std::endl;
  trace.info() << "Testing writing PGM3D ... ";  
  bool ok1 = anImportedImage1>> "testGenericWriter.pgm3d";
  trace.info() <<"[done]"  << std::endl;
  
  bool okh5 = true;
  bool okh5bis = true;
#ifdef WITH_HDF5
  trace.info() << "Testing writing HDF5 3D ... ";
  okh5 = anImportedImage1 >> "testGenericWriter.h5";
  trace.info() <<"[done]"  << std::endl;
  trace.info() << "Testing writing HDF5 3D (bis) ... ";  
  okh5bis = DGtal::GenericWriter<Image3D>::exportFile("testGenericWriter_bis.h5", anImportedImage1, "/UInt8Array3D");
  trace.info() <<"[done]"  << std::endl;
#endif
  
  trace.info() << "Testing writing vol ... ";  
  bool ok2 = anImportedImage1 >> "testGenericWriter.vol";
  trace.info() <<"[done]"  << std::endl;
  trace.info() << "Testing writing raw ... ";  
  bool ok3 = anImportedImage1 >>"testGenericWriter.raw";
  trace.info() <<"[done]"  << std::endl;
  trace.info() << "Testing writing raw (bis) ... ";  
  bool ok3bis = DGtal::GenericWriter<Image3D>::exportFile( "testGenericWriter.raw", anImportedImage1);
  trace.info() <<"[done]"  << std::endl;

  std::string filenameImage2 = testPath + "samples/contourS.pgm";    

  trace.info() << "Reading 2D image  ... ";  
  Image2D anImportedImage2 = DGtal::GenericReader<Image2D>::import(filenameImage2);  
  trace.info() <<"[done]"  << std::endl; 
  trace.info() << "Testing writing pgm ... ";  
  bool ok4 = anImportedImage2 >> "testGenericWriter.pgm";
  trace.info() <<"[done]"  << std::endl;
  trace.info() << "Testing writing raw ... ";  
  bool ok5 = anImportedImage2 >> "testGenericWriter.raw";
  trace.info() <<"[done]"  << std::endl;



  nbok += ok1 && okh5 && okh5bis && ok2 && ok3 && ok3bis && ok4 & ok5  ? 1 : 0; 
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
  trace.beginBlock ( "Testing class GenericWriter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testGenericWriter(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
