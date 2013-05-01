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
 * @file testGenericReader.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Functions for testing class GenericReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGTal/io/readers/GenericReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigTest.h"





///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GenericReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testGenericReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  DGtal::trace.beginBlock ( "Testing 3D Image Reader" );
   
  typedef DGtal::ImageContainerBySTLMap<DGtal::Z3i::Domain, unsigned int> Image3D;
  typedef DGtal::ImageContainerBySTLMap<DGtal::Z2i::Domain, unsigned int> Image2D;
  std::string filenameImage1 = testPath + "samples/cat10.vol";    
  Image3D anImportedImage1= DGtal::GenericReader<Image3D>::import(filenameImage1);
  unsigned int size0Img1= anImportedImage1.domain().extent()[0];
  unsigned int size1Img1= anImportedImage1.domain().extent()[1];
  unsigned int size2Img1= anImportedImage1.domain().extent()[2];
  DGtal::trace.info()<<"Vol image read: size[0]:" << size0Img1  ;
  DGtal::trace.info()<<"size[1]:  " << size1Img1;
  DGtal::trace.info()<<"size[2]:  " << size2Img1 << std::endl;
  nbok += (size0Img1==40 && size1Img1==40 && size2Img1==40) ? 1 : 0; 
  nb++;
  std::string filenameImage2 = testPath + "samples/cat10.pgm3d";    
  Image3D anImportedImage2= DGtal::GenericReader<Image3D>::import(filenameImage2);
  unsigned int size0Img2= anImportedImage2.domain().extent()[0];
  unsigned int size1Img2= anImportedImage2.domain().extent()[1];
  unsigned int size2Img2= anImportedImage2.domain().extent()[2];
  DGtal::trace.info()<<"Pgm3D image read: size[0]:" << size0Img1  ;
  DGtal::trace.info()<<"size[1]:  " << size1Img1;
  DGtal::trace.info()<<"size[2]:  " << size2Img1 << std::endl;;
  nbok += (size0Img1==40 && size1Img1==40 && size2Img1==40) ? 1 : 0; 
  nb++;

  std::string filenameImage3 = testPath + "samples/contourS.pgm";    
  Image2D anImportedImage3= DGtal::GenericReader<Image2D>::import(filenameImage3);
  unsigned int size0Img3= anImportedImage3.domain().extent()[0];
  unsigned int size1Img3= anImportedImage3.domain().extent()[1];
  DGtal::trace.info()<<"Pgm image read: size[0]:" << size0Img3  ;
  DGtal::trace.info()<<"size[1]:  " << size1Img3     << std::endl;;
  nbok += (size0Img3==185 && size1Img3==85 ) ? 1 : 0; 
  nb++;



  DGtal::trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  DGtal::trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  DGtal::trace.beginBlock ( "Testing class GenericReader" );
  DGtal::trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    DGtal::trace.info() << " " << argv[ i ];
  DGtal::trace.info() << std::endl;

  bool res = testGenericReader(); // && ... other tests
  DGtal::trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
  DGtal::trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
