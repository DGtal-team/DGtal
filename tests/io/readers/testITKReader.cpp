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
 * @file testITKReader.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Lorraine, France
 *
 * @date 2017/04/02
 *
 * Functions for testing class ITKReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/readers/ITKReader.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ITKReader.
///////////////////////////////////////////////////////////////////////////////


/**
   Check that an image loaded via DGtal ITKReader keeps the same spatial
   information than an image loaded via itk::ImageFileReader.

 */
template <typename PixelType>
void testSpatialInformation(const std::string &filename)
{
  typedef DGtal::ImageContainerByITKImage < DGtal::Z3i::Domain, PixelType > DGtalImage;

  DGtalImage img = DGtal::ITKReader<DGtalImage>::importITK(filename);

  typename DGtalImage::ITKImagePointer dgtal_itk = img.getITKImagePointer();


  typedef itk::Image<PixelType, 3> ItkImage;
  typedef itk::ImageFileReader<ItkImage> ReaderType;
  typename ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(filename);

  reader->Update();

  typename ItkImage::Pointer itk = reader->GetOutput();

  INFO( "Checking spacing" )
  REQUIRE( dgtal_itk->GetSpacing() == itk->GetSpacing() );
  INFO( "Checking origin" )
  REQUIRE( dgtal_itk->GetOrigin() == itk->GetOrigin() );
  INFO( "Checking direction" )
  REQUIRE( dgtal_itk->GetDirection() == itk->GetDirection() );
}



TEST_CASE( "Testing ITKReader" )
{
  // Default image selector = STLVector
  typedef ImageContainerBySTLVector<Z3i::Domain, unsigned char> Image3DUC;
  typedef ImageContainerBySTLVector<Z3i::Domain, uint16_t> Image3D16b;
  typedef ImageContainerBySTLVector<Z3i::Domain, double> Image3Dd;

  SECTION(
  "Testing feature io/readers of ITKReader with  16 bits (uint16) images" )
  {
    Image3D16b im = ITKReader<Image3D16b>::importITK(
    testPath + "samples/lobsterCroped16b.mhd" );
    REQUIRE( ( im( Z3i::Point( 35, 29, 3 ) ) == 60400 ) );
  }

  SECTION(
  "Testing feature io/readers of ITKReader with rescaled 16 bits (uint16) "
  "images" )
  {
    typedef DGtal::functors::Rescaling<uint16_t, unsigned char> RescalFCT;
    RescalFCT resc = RescalFCT( 0, 65535, 0, 255 );
    Image3DUC im   = ITKReader<Image3DUC>::importITK(
    testPath + "samples/lobsterCroped16b.mhd", resc );
    REQUIRE( ( im( Z3i::Point( 35, 29, 3 ) ) == resc( 60400 ) ) );
  }
  SECTION(
  "Testing behavior of ITKReader on non existent image file" )
  {
    bool caughtException = false;
    const std::string filename = testPath + "samples/null.mhd"; //non existent file
    try
    {
      Image3DUC im = ITKReader<Image3DUC>::importITK(filename);
    }
    catch(exception &)
    {
      caughtException = true;
      trace.info() <<"Exception was correctly caught" << std::endl;
    }
    REQUIRE( caughtException == true);
  }

  SECTION(
  "Checking spatial information when loading through DGtal" )
  {
    testSpatialInformation<int16_t>(testPath + "samples/lobsterCroped16b.mhd" );
    testSpatialInformation<int16_t>(testPath + "samples/lobsterCropedB16b.mhd" );
  }

  SECTION(
    "Checking import with/without shifted domain")

  {
    Image3Dd imShifted = ITKReader<Image3Dd>::importITK(testPath + "samples/lobsterCroped2.nii.gz",
                                                        ITKIOTrait<Image3Dd::Value>::DefaultReadFunctor(), true );

    Image3Dd imNonShifted = ITKReader<Image3Dd>::importITK(testPath + "samples/lobsterCroped2.nii.gz", false );

    REQUIRE( ( imShifted( Z3i::Point( 144, 100, 21 ) ) == 113 )  );
    REQUIRE( ( imNonShifted( Z3i::Point( 94, 50, 11 ) ) == 113 )  );
    Z3i::Point pTarget (95,76,7);
    auto i = 101*pTarget[1]+pTarget[0]+pTarget[2]*101*101;
    trace.info() << "value:"  << *(imNonShifted.range().begin()+i);
    REQUIRE( ( *(imNonShifted.range().begin()+i) == *(imShifted.range().begin()+i ) ));
    REQUIRE( ( *(imNonShifted.range().begin()+i) == 68  ));
  }


}

/** @ingroup Tests **/
