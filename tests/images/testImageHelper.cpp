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
#include "DGtal/images/ImageContainerBySTLVector.h"


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
  typedef  DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain, unsigned char>  Image3D;
  typedef  DGtal::ImageContainerBySTLVector<DGtal::Z2i::Domain, unsigned char>  Image2D;

  typedef DGtal::ConstImageAdapter<Image3D, DGtal::Z2i::Domain, DGtal::AddOneDimensionDomainFunctor< DGtal::Z3i::Point>,
				   Image3D::Value,  DGtal::DefaultFunctor >  MySliceImageAdapter;
  DGtal::MinusOneDimensionDomainFunctor<DGtal::Z2i::Point>  invFunctor(0);

  bool res= true;
  Image3D image = VolReader<Image3D>::importVol( filename ); 
  DGtal::Z2i::Domain domain((invFunctor.operator()(image.domain().lowerBound())), 
			    (invFunctor.operator()(image.domain().upperBound())));
  DGtal::DefaultFunctor idV;
 
  DGtal::AddOneDimensionDomainFunctor<DGtal::Z3i::Point> aSliceFunctor(0, 20);
  MySliceImageAdapter sliceImageX(image, domain, aSliceFunctor, idV);
  res &= PGMWriter<MySliceImageAdapter>::exportPGM("exportedSlice2DDimX.pgm",sliceImageX);

  DGtal::AddOneDimensionDomainFunctor<DGtal::Z3i::Point> aSliceFunctor2(1, 20);
  MySliceImageAdapter sliceImageY(image, domain, aSliceFunctor2, idV);
  res &= PGMWriter<MySliceImageAdapter>::exportPGM("exportedSlice2DDimY.pgm",sliceImageX);

  DGtal::AddOneDimensionDomainFunctor<DGtal::Z3i::Point> aSliceFunctor3(2, 20);
  MySliceImageAdapter sliceImageZ(image, domain, aSliceFunctor3, idV);
  res &= PGMWriter<MySliceImageAdapter>::exportPGM("exportedSlice2DDimZ.pgm",sliceImageX);

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
