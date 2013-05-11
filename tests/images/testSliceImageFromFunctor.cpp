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
 * @file testSliceImageFromFunctor.cpp
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
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/writers/PGMWriter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"

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

bool testSliceImageFromFunctor()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  std::string filename = testPath + "samples/cat10.vol";
  trace.beginBlock ( "Testing block ..." );
  typedef  DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain, unsigned char>  Image3D;
  typedef  DGtal::ImageContainerBySTLVector<DGtal::Z2i::Domain, unsigned char>  Image2D;

  typedef DGtal::ConstImageAdapter<Image3D, DGtal::Z2i::Domain, DGtal::AddOneDimensionPointFunctor< DGtal::Z3i::Space>,
				   Image3D::Value,  DGtal::DefaultFunctor >  MySliceImageAdapter;
  DGtal::Projector<DGtal::Z2i::Space>  proj(0);

  bool res= true;
  Image3D image = VolReader<Image3D>::importVol( filename ); 
  DGtal::Z2i::Domain domain(proj(image.domain().lowerBound()), 
			    proj(image.domain().upperBound()));
 
  DGtal::AddOneDimensionPointFunctor<DGtal::Z3i::Space> aSliceFunctor(0, 20);
  MySliceImageAdapter sliceImageX(image, domain, aSliceFunctor, DGtal::DefaultFunctor());
  res &= PGMWriter<MySliceImageAdapter>::exportPGM("exportedSlice2DDimX.pgm",sliceImageX);

  DGtal::AddOneDimensionPointFunctor<DGtal::Z3i::Space> aSliceFunctor2(1, 20);
  MySliceImageAdapter sliceImageY(image, domain, aSliceFunctor2, DGtal::DefaultFunctor());
  res &= PGMWriter<MySliceImageAdapter>::exportPGM("exportedSlice2DDimY.pgm",sliceImageX);

  DGtal::AddOneDimensionPointFunctor<DGtal::Z3i::Space> aSliceFunctor3(2, 20);
  MySliceImageAdapter sliceImageZ(image, domain, aSliceFunctor3, DGtal::DefaultFunctor());
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

  bool res = testSliceImageFromFunctor(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
