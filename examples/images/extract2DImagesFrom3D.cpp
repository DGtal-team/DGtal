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
 * @file extract2DImagesFrom3D.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/04/24
 *
 * An example file named extract2DImagesFrom3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <sstream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/writers/PGMWriter.h"
#include "DGtal/images/ImageSelector.h"
#include "ConfigExamples.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  typedef ImageSelector < Z3i::Domain, unsigned char>::Type Image3D;
  typedef ImageSelector < Z2i::Domain, unsigned char>::Type Image2D;
    
  trace.beginBlock ( "Example extract2DImagesFrom3D" );

  // Importing a 3D image 
  std::string filename = examplesPath + "samples/lobster.vol";
  Image3D image = VolReader<Image3D>::importVol( filename ); 

  // Extracting 2D slices ... and export them in the pgm format.
  for (unsigned int i=0; i<55; i+=10){
    std::stringstream name;
    name << "lobsterSliceZ_"  << i << ".pgm";
    Image2D slice2D_Z = DGtal::extractLowerDimImage<Image3D, Image2D> (image, 2, i);
    PGMWriter<Image2D>::exportPGM(name.str(), slice2D_Z);
  }

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
