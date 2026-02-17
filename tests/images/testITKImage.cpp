/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR a PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testITKImage.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/12/09
 *
 * Functions for testing class ITKImage.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerByITKImage.h"
#include <boost/foreach.hpp>

//specific itk method
#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include <itkExtractImageFilter.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ITKImage.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testITKImage()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "ITK Image init..." );

  typedef DGtal::int32_t Integer;
  typedef SpaceND<3, Integer > Space3Type;
  typedef HyperRectDomain<Space3Type> Domain;
  typedef Domain::Point Point;

  typedef ImageContainerByITKImage<Domain, Integer> Image;

  const Integer t[ ] = { 1, 1, 1};
  const Integer t2[ ] = { 5, 5, 5};
  const Integer t3[ ] = { 2, 2, 2};
  Point lowerBound ( t );
  Point upperBound ( t2 );
  Point c ( t3 );
  Integer val;

  Image myImage ( Domain(lowerBound, upperBound) );

  trace.info() << myImage << std::endl;
  trace.info() << "getvalue= " << myImage(c) << endl;
  trace.info() << "set value 23 " << endl;
  myImage.setValue( c, 23);

  val =  myImage(c);

  if (val == 23)
    nbok++;
  trace.info() << "getvalue= " << val << endl;
  nb++;

  //Iterator test
  trace.info() << "Simple Iterator=";
  for (Image::ConstIterator it = myImage.begin(), itend = myImage.end();
      it != itend;
      ++it)
    trace.warning() << myImage(it) << " ";
  trace.info() << endl;

  //We rewrite the image
  int nbVal = 0;
  for (Image::Iterator it = myImage.begin(), itend = myImage.end();
      it != itend;
      ++it)
    myImage.setValue(it, nbVal++);

  trace.info() << "Set Iterator=";
  for (Image::ConstIterator it = myImage.begin(), itend = myImage.end();
      it != itend;
      ++it)
    trace.warning() << myImage(it) << " ";
  trace.info() << endl;

  auto & container = myImage.container();
  (void)container;


  trace.info() << "(" << nbok << "/" << nb << ") "
  << "true == true" << std::endl;
  trace.endBlock();

  return nbok == nb;
}

bool testITKMethod()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Test the use of a pure ITK method..." );

  typedef DGtal::int32_t Integer;
  typedef SpaceND<2, Integer > Space2Type;
  typedef HyperRectDomain<Space2Type> Domain;
  typedef Domain::Point Point;


  typedef ImageContainerByITKImage<Domain, Integer> Image;

  Point lowerBound ( 0, 0 );
  Point upperBound ( 9, 9);
  Domain domain(lowerBound, upperBound);

  Image myImage(domain);
  trace.info() << myImage << std::endl;

  //We fill the image
  Integer nbVal = 0;
  for (Image::Iterator it = myImage.begin(), itend = myImage.end();
      it != itend;
      ++it)
    myImage.setValue(it, nbVal++);

  trace.info() << "Input image=";
  for (Image::ConstIterator it = myImage.begin(), itend = myImage.end();
      it != itend;
      ++it)
    trace.warning() << myImage(it) << " ";
  trace.info() << endl;


  // We define a cropFilter
  typedef itk::ExtractImageFilter< Image::ITKImage, Image::ITKImage > CropFilter;

  // Crop filter region
  Image::ITKImage::SizeType size;
  size[0] = 5;
  size[1] = 5;

  Image::ITKImage::IndexType index;
  index[0] = 2;
  index[1] = 2;

  Image::ITKImage::RegionType regionToExtract(index,size);

  // Crop filter process
  CropFilter::Pointer cropFilter = CropFilter::New();
  cropFilter->SetInput( myImage.getITKImagePointer() );
  cropFilter->SetExtractionRegion( regionToExtract  );
  cropFilter->Update();

  // Pointer to the filter output
  Image::ITKImagePointer handleOut = cropFilter->GetOutput();
  Image myImageOut ( handleOut );


  trace.info() << "Output image=";

  for (Image::ConstIterator it = myImageOut.begin(), itend = myImageOut.end();
       it != itend;
       ++it)
  {
    nbok += (it.Value() == (it.GetIndex()[1]*10 + it.GetIndex()[0]));
    nb++;
    trace.warning() << it.Value() << "(" << (it.GetIndex()[1]*10 + it.GetIndex()[0]) << ")" << " ";
  }
  trace.info() << endl;

  trace.info() << "(" << nbok << "/" << nb << ") " << "true == true" << std::endl;
  trace.endBlock();

  return nbok == 25 && nb == 25;
}

bool testITKImageWithMetadata()
{
  trace.beginBlock ( "ITK Image With Metadata init..." );

  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef DGtal::int32_t Integer;
  typedef SpaceND<3, Integer > Space3Type;
  typedef HyperRectDomain<Space3Type> Domain;
  typedef Domain::Point Point;

  typedef ImageContainerByITKImage<Domain, Integer> Image;

  // REMEMBER: Origin in ITK is the physical position of the index {0,0,...}
  // even when that zero-index is not included in the LargestPossibleRegion of the image.
  const Integer t[ ] = { 1, 1, 1};
  const Integer t2[ ] = { 5, 5, 5};
  const Integer t3[ ] = { 2, 2, 2};
  Point lowerBound ( t );
  Point upperBound ( t2 );
  Point c ( t3 );
  Integer val;

  Image myImage ( Domain(lowerBound, upperBound) );
  // Fill the image
  int nbVal = 0;
  for (Image::Iterator it = myImage.begin(), itend = myImage.end();
      it != itend; ++it) {
    myImage.setValue(it, nbVal++);
  }
  // Change the default metadata (physical properties) of an itk_image.
  auto itk_image = myImage.getITKImagePointer();
  Image::ITKImage::PointType origin;
  origin.Fill(10.0);
  Image::ITKImage::SpacingType spacing;
  spacing.Fill(2.0);
  Image::ITKImage::DirectionType direction;
  direction.SetIdentity();
  itk_image->SetOrigin(origin);
  itk_image->SetSpacing(spacing);
  itk_image->SetDirection(direction);

  // Check that the value of index points is not affected by a change of metadata.
  val = myImage.operator()(lowerBound);
  nbok += (val == 0); nb++;
  trace.info() << "Index: " << lowerBound << ". Value: " << val << ". Expected: " << 0 << std::endl;
  val = myImage.operator()(upperBound);
  nbok += (val == 124); nb++;
  trace.info() << "Index: " << upperBound << ". Value: " << val << ". Expected: " << 124 << std::endl;
  val = myImage.operator()(c);
  nbok += (val == 31); nb++;
  trace.info() << "Index: " << c << ". Value: " << val << ". Expected: " << 31 << std::endl;

  Image::PhysicalPoint physical_point;
  Image::PhysicalPoint expected_physical_point;
  Image::Point index_point;

  // when shiftDomain is zero, index points (ITK) and domain points (DGtal) are equal
  index_point = myImage.getIndexFromDomainPoint(lowerBound);
  nbok += (index_point == lowerBound); nb++;
  physical_point = myImage.getPhysicalPointFromDomainPoint(index_point);
  expected_physical_point = myImage.getLowerBoundAsPhysicalPoint();
  nbok += (physical_point[0] == 12.0); nb++;
  nbok += (physical_point == expected_physical_point); nb++;
  trace.info() << "Index: " << index_point << ". PhysicalPoint: " << physical_point << ". Expected: " << expected_physical_point << std::endl;

  index_point = myImage.getIndexFromDomainPoint(upperBound);
  nbok += (index_point == upperBound); nb++;
  physical_point = myImage.getPhysicalPointFromDomainPoint(index_point);
  expected_physical_point = myImage.getUpperBoundAsPhysicalPoint();
  nbok += (physical_point[0] == 20.0); nb++;
  nbok += (physical_point == expected_physical_point); nb++;
  trace.info() << "Index: " << index_point << ". PhysicalPoint: " << physical_point << ". Expected: " << expected_physical_point << std::endl;

  auto index_back = myImage.getDomainPointFromPhysicalPoint(physical_point);
  nbok += (index_back == upperBound); nb++;
  trace.info() << "PhysicalPoint: " << physical_point << ". Index (back): " << index_back << ". Expected: " << upperBound << std::endl;

  trace.endBlock();

  return nbok == 10 && nb == 10;
}

bool testITKImageWithShiftDomain()
{
  trace.beginBlock ( "ITK Image With ShiftDomain init..." );

  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef DGtal::int32_t Integer;
  typedef SpaceND<3, Integer > Space3Type;
  typedef HyperRectDomain<Space3Type> Domain;
  typedef Domain::Point Point;

  typedef ImageContainerByITKImage<Domain, Integer> Image;

  const Integer t[ ] = { 1, 1, 1};
  const Integer t2[ ] = { 5, 5, 5};
  const Integer t3[ ] = { 2, 2, 2};
  Point lowerBound ( t );
  Point upperBound ( t2 );
  Point c ( t3 );
  Integer val;

  Image myImage ( Domain(lowerBound, upperBound) );

  // Fill the image
  int nbVal = 0;
  for (Image::Iterator it = myImage.begin(), itend = myImage.end();
      it != itend; ++it) {
    myImage.setValue(it, nbVal++);
  }

  // Change the default metadata (physical properties) of an itk_image.
  auto itk_image = myImage.getITKImagePointer();
  Image::ITKImage::PointType origin;
  origin.Fill(10.0);
  Image::ITKImage::SpacingType spacing;
  spacing.Fill(2.0);
  Image::ITKImage::DirectionType direction;
  direction.SetIdentity();
  itk_image->SetOrigin(origin);
  itk_image->SetSpacing(spacing);
  itk_image->SetDirection(direction);


  // Apply a domainShift
  const Integer sd[ ] = { -20, -20, -20};
  Point domainShift(sd);
  myImage.updateDomain(domainShift);
  Point new_lowerBound = myImage.domain().lowerBound();
  Point new_upperBound = myImage.domain().upperBound();
  nbok += ( new_lowerBound == lowerBound + domainShift); nb++;
  trace.info() << "lowerBound: " << new_lowerBound << ". Expected: " << lowerBound + domainShift << std::endl;
  nbok += ( new_upperBound == upperBound + domainShift); nb++;
  trace.info() << "upperBound: " << new_upperBound << ". Expected: " << upperBound + domainShift << std::endl;

  // Check that the shifted domain points to the correct indices of the image.
  val = myImage.operator()(new_lowerBound);
  // It should have the same value than lowerBound had before applying the domainShift
  nbok += (val == 0); nb++;
  trace.info() << "Index: " << new_lowerBound << ". Value: " << val << ". Expected: " << 0 << std::endl;
  val = myImage.operator()(new_upperBound);
  nbok += (val == 124); nb++;
  trace.info() << "Index: " << new_upperBound << ". Value: " << val << ". Expected: " << 124 << std::endl;
  val = myImage.operator()(myImage.getDomainPointFromIndex(c));
  nbok += (val == 31); nb++;
  trace.info() << "Index: " << c << ". Value: " << val << ". Expected: " << 31 << std::endl;

  Image::PhysicalPoint physical_point;
  Image::PhysicalPoint expected_physical_point;
  Image::Point index_point;
  Image::Point domain_point;

  index_point = lowerBound;
  domain_point = new_lowerBound;
  physical_point = myImage.getPhysicalPointFromDomainPoint(domain_point);
  expected_physical_point = Image::PhysicalPoint(12.0, 12.0, 12.0);
  nbok += (physical_point == expected_physical_point); nb++;
  trace.info() << "Domain: " << domain_point <<
    ". Index: " << index_point <<
    ". PhysicalPoint: " << physical_point <<
    ". Expected: " << expected_physical_point << std::endl;

  index_point = myImage.getIndexFromDomainPoint(new_lowerBound);
  nbok += ( index_point == lowerBound ); nb++;
  trace.info() << "index_point: " << index_point << ". Expected: " << lowerBound << std::endl;
  physical_point = myImage.getPhysicalPointFromDomainPoint(new_lowerBound);
  expected_physical_point = myImage.getLowerBoundAsPhysicalPoint();
  nbok += (physical_point[0] == 12.0); nb++;
  nbok += (physical_point == expected_physical_point); nb++;
  trace.info() << "Domain: " << new_lowerBound <<
    ". Index: " << index_point <<
    ". PhysicalPoint: " << physical_point <<
    ". Expected: " << expected_physical_point << std::endl;

  index_point = upperBound;
  domain_point = new_upperBound;
  physical_point = myImage.getPhysicalPointFromDomainPoint(domain_point);
  expected_physical_point = myImage.getUpperBoundAsPhysicalPoint();
  nbok += (physical_point[0] == 20.0); nb++;
  nbok += (physical_point == expected_physical_point); nb++;
  trace.info() << "Domain: " << new_lowerBound <<
    ". Index: " << index_point <<
    ". PhysicalPoint: " << physical_point <<
    ". Expected: " << expected_physical_point << std::endl;

  return nbok == 11 && nb == 11;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ITKImage" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testITKImage() && testITKMethod() &&
    testITKImageWithMetadata() && testITKImageWithShiftDomain();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
