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
 * @file testITKDicomReader.cpp
 * @ingroup Tests
 * @author Boris Mansencal (\c boris.mansencal@labri.fr )
 * LaBRI (CNRS, UMR 5800, University of Bordeaux, Bordeaux-INP), France
 *
 * @date 2019/02/05
 *
 * Functions for testing class ITKDicomReader.
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
#include "DGtal/images/ImageContainerByITKImage.h"
#include "DGtal/io/readers/ITKDicomReader.h"
// Required ITK files to read serie DICOM files
// DGtal must be compiled with " -DDGTAL_WITH_ITK=true" option
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include <itkGDCMSeriesFileNames.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ITKReader.
///////////////////////////////////////////////////////////////////////////////

std::vector<std::string>
getFirstDicomSerieFileNames(const std::string &path)
{
  typedef itk::GDCMSeriesFileNames NamesGeneratorType;
  NamesGeneratorType::Pointer nameGenerator = NamesGeneratorType::New();
  nameGenerator->SetUseSeriesDetails( true );
  nameGenerator->SetDirectory( path );
  
  typedef itk::GDCMSeriesFileNames::SeriesUIDContainerType SeriesIdContainer;
  const SeriesIdContainer & seriesUID = nameGenerator->GetSeriesUIDs();
  
  if (! seriesUID.empty() )
  {
    return nameGenerator->GetFileNames( *(seriesUID.begin()) );
  }
  return std::vector<std::string>();
}


template <typename Image>
void
testImportDICOM()
{
  std::vector<std::string> fileNames = getFirstDicomSerieFileNames( testPath + "samples/dicomSample" );
      
  Image image = ITKDicomReader<Image>::importDICOM( fileNames );
  
  unsigned int nbVal=0, nbPos = 0;
  typename Image::ConstRange r = image.constRange();
  for ( typename Image::ConstRange::ConstIterator it=r.begin(), itend=r.end() ; it != itend ; ++it )
    {
      ++nbVal;
      if ( (*it) > 0 ) ++nbPos;
    }

  REQUIRE( ( nbVal==2130048 && nbPos==296030 ) );
}

template <typename PixelType>
void testSpatialInformation()
{
  std::vector<std::string> fileNames = getFirstDicomSerieFileNames( testPath + "samples/dicomSample" );
      
  typedef ImageContainerByITKImage<Z3i::Domain, PixelType> DGtalImage;
  DGtalImage img = ITKDicomReader<DGtalImage>::importDICOM( fileNames );
  typename DGtalImage::ITKImagePointer dgtal_itk = img.getITKImagePointer();

  typedef itk::Image<PixelType, 3> ItkImage;
  typedef itk::ImageSeriesReader<ItkImage> ItkReader;
  typename ItkReader::Pointer reader = ItkReader::New();
  reader->SetFileNames( fileNames );

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

  SECTION(
  "Testing ITKDicomReader with 8 bits ImageContainerBySTLVector images" )
  {
    typedef ImageContainerBySTLVector<Z3i::Domain, unsigned char> Image;
    testImportDICOM<Image>();
  }

  SECTION(
  "Testing ITKDicomReader with 8 bits ImageContainerByITKImage images" )
  {
    typedef ImageContainerByITKImage<Z3i::Domain, unsigned char> Image;
    testImportDICOM<Image>();
  }

  SECTION(
  "Testing ITKDicomReader with 16 bits ImageContainerBySTLVector images" )
  {
    typedef ImageContainerBySTLVector<Z3i::Domain, uint16_t> Image;
    testImportDICOM<Image>();
  }
  
  SECTION(
  "Testing ITKDicomReader with 16 bits ImageContainerByITKImage images" )
  {
    typedef ImageContainerByITKImage<Z3i::Domain, uint16_t> Image;
    testImportDICOM<Image>();
  }



  SECTION(
  "Testing behavior of ITKDicomReader on empty filenames vector" )
  {
    typedef ImageContainerBySTLVector<Z3i::Domain, unsigned char> Image;
    std::vector<std::string> filenames;
    bool caughtException = false;
    try
    {
      Image im = ITKDicomReader<Image>::importDICOM(filenames);
    }
    catch(exception &)
    {
      caughtException = true;
      trace.info() <<"Exception was correctly caught" << std::endl;
    }
    REQUIRE( caughtException == true);
  }

  SECTION(
  "Testing ITKDicomReader with 8 bits ImageContainerByITKImage images keeps spatial information" )
  {
    testSpatialInformation<unsigned char>();
  }

  SECTION(
  "Testing ITKDicomReader with 16 bits ImageContainerByITKImage images keeps spatial information" )
  {
    testSpatialInformation<int16_t>();
  }
 

}

/** @ingroup Tests **/
