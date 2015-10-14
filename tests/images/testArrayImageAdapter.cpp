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
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 * LAboratory of MAthematics - LAMA (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/07/16
 *
 * This file is part of the DGtal library
 */

#if __cplusplus < 201103L
  #error This test file needs C++11 features.
#endif

#include <cstddef>
#include <iostream>
#include <new>
#include <cmath>

#include <DGtal/kernel/SpaceND.h>
#include <DGtal/kernel/domains/HyperRectDomain.h>
#include <DGtal/images/ImageContainerBySTLVector.h>
#include <DGtal/images/CConstImage.h>
#include <DGtal/images/CImage.h>

#include <DGtal/images/ArrayImageAdapter.h>

using namespace DGtal;
using namespace std;

template < typename TImage, typename TDomain >
void fillImageWithCounter ( TImage& anImage, TDomain const& aDomain )
{
  size_t cnt = 0;
  for ( auto const& point : aDomain )
    anImage.setValue( point, cnt++ );
}

template < typename TImage >
void fillImageWithCounter ( TImage& anImage )
{
  fillImageWithCounter( anImage, anImage.domain() );
}

template < typename TImage, typename TFunction, typename TDomain >
void fillImageWithPointFn ( TImage& anImage, TFunction const& aFunction, TDomain const& domain )
{
  using Image = TImage;
  using Value = typename Image::Value;
  for ( auto const& point : domain )
    {
      Value value = 0;
      for ( size_t i = 0; i < Image::dimension; ++i )
        value += aFunction( i, point[i] );

      anImage.setValue(point, value);
    }
}

template < typename TImage, typename TFunction >
void fillImageWithPointFn ( TImage& anImage, TFunction const& aFunction )
{
  fillImageWithPointFn ( anImage, aFunction, anImage.domain() );
}

template < typename TImage, typename TFunction, typename TDomain >
void incrementImageWithPointFn ( TImage& anImage, TFunction const& aFunction, TDomain const& domain )
{
  using Image = TImage;
  using Value = typename Image::Value;
  for ( auto const& point : domain )
    {
      Value value = anImage(point);
      for ( size_t i = 0; i < Image::dimension; ++i )
        value += aFunction( i, point[i] );

      anImage.setValue(point, value);
    }
}

template < typename TImage, typename TFunction >
void incrementImageWithPointFn ( TImage& anImage, TFunction const& aFunction )
{
  incrementImageWithPointFn ( anImage, aFunction, anImage.domain() );
}

template < typename TDomain, typename TValue, typename TFunction >
void fastFillImageWithPointFn ( ImageContainerBySTLVector<TDomain, TValue>& anImage, TFunction const& aFunction )
{
  using Image = ImageContainerBySTLVector<TDomain, TValue>;
  using Value = typename Image::Value;
  auto imgit = anImage.begin();
  for ( auto const& point : anImage.domain() )
    {
      Value value = 0;
      for ( size_t i = 0; i < Image::dimension; ++i )
        value += aFunction( i, point[i] );

      *(imgit++) = value;
    }
}

template < typename TIterator, typename TDomain, typename TFunction >
void fastFillImageWithPointFn ( ArrayImageAdapter<TIterator, TDomain>& anImage, TFunction const& aFunction )
{
  using Image = ArrayImageAdapter<TIterator, TDomain>;
  using Value = typename Image::Value;
  for ( auto imgit = anImage.begin(); imgit != anImage.end(); ++imgit )
    {
      Value value = 0;
      auto const point = imgit.getPoint();

      for ( size_t i = 0; i < Image::dimension; ++i )
        value += aFunction( i, point[i] );

      *imgit = value;
    }
}

template < typename TImage >
bool checkImage( TImage& anImage )
{
  using Image = TImage;
  using Value = typename Image::Value;
  using Domain = typename Image::Domain;
  using Point = typename Image::Point;
  using Dimension = typename Point::Dimension;
  using Coordinate = typename Point::Coordinate;
  using RefImage = ImageContainerBySTLVector<Domain, Value>;

  // Checks CImage concept.
  BOOST_CONCEPT_ASSERT( (DGtal::concepts::CImage<TImage>) );

  size_t nb = 0;
  size_t nbok = 0;

  // Full domain
  auto const domain = anImage.domain();

  // Sub domain
  Point lowerPt = domain.lowerBound();
  Point upperPt = domain.upperBound();
  for ( Dimension i = 0; i < Domain::dimension; ++i )
    {
      lowerPt[i] = std::min( upperPt[i]-1, lowerPt[i] + 1 + static_cast<Coordinate>(i) );
      upperPt[i] = std::max( lowerPt[i]+1, upperPt[i] - static_cast<Coordinate>(Domain::dimension - i) );
    }
  auto const sub_domain = Domain( lowerPt, upperPt );

  // Checks that sub domain is not empty and different of full domain
  nb++; nbok += ( !sub_domain.isEmpty() && sub_domain.size() != domain.size() ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") Checking sub-domain." << std::endl;

  // Reference image
  RefImage ref_image( domain );

  // The filling function
  auto const fn = [] (size_t i, Coordinate x) { return cos( static_cast<Value>(pow(100, i)*x ) ); };

  // Fill with function
  fillImageWithPointFn( ref_image, fn );
  fillImageWithPointFn( anImage, fn );
  nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") Filling with point dependant function." << std::endl;

  // Fill with counter
  fillImageWithCounter( ref_image );
  fillImageWithCounter( anImage );
  nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") Filling with counter." << std::endl;

  // Fast filling with function
  fastFillImageWithPointFn( ref_image, fn );
  fastFillImageWithPointFn( anImage, fn );
  nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") Fast filling with point dependant function." << std::endl;

  // Increment with function
  incrementImageWithPointFn( ref_image, fn );
  incrementImageWithPointFn( anImage, fn );
  nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") Incrementing with point dependant function." << std::endl;

  // Partial fill with counter
    {
      auto sub_image = makeArrayImageAdapterFromImage( anImage, sub_domain );
      fillImageWithCounter( ref_image, sub_domain );
      fillImageWithCounter( sub_image );
      nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
      trace.info() << "(" << nbok << "/" << nb << ") Partial filling with counter." << std::endl;
    }

  // Partial increment with function
    {
      auto sub_image = makeArrayImageAdapterFromImage( anImage, sub_domain );
      incrementImageWithPointFn( ref_image, fn, sub_domain );
      incrementImageWithPointFn( sub_image, fn );
      nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
      trace.info() << "(" << nbok << "/" << nb << ") Partial increment with point dependant function." << std::endl;
    }

  // Fast partial fill with function
    {
      auto sub_image = makeArrayImageAdapterFromImage( anImage, sub_domain );
      fillImageWithPointFn( ref_image, fn,  sub_domain );
      fastFillImageWithPointFn( sub_image, fn );
      nb++; nbok += std::equal( ref_image.begin(), ref_image.end(), anImage.begin() ) ? 1 : 0;
      trace.info() << "(" << nbok << "/" << nb << ") Fast partial filling with point dependant function." << std::endl;
    }

  return nb == nbok;
}

int main()
{
  using Space = SpaceND<3>;
  using Domain = HyperRectDomain<Space>;
  using Value = double;

  trace.beginBlock("Testing ArrayImageAdapter class");

  const Domain domain{ {0, 1, 2}, {12, 8, 11} };
  const Domain sub_domain{ {0, 2, 4}, {8, 7, 10} };

  size_t nb = 0;
  size_t nbok = 0;

  {
    trace.beginBlock("Checking ArrayImageAdapter with raw pointer");
    Value* data = new Value[domain.size()];
    auto image = makeArrayImageAdapterFromIterator( data, domain );
    nb++; nbok += checkImage(image) ? 1 : 0;
    delete[] data;
    trace.endBlock();
  }

  {
    trace.beginBlock("Checking ArrayImageAdapter with raw pointer on sub-domain");
    Value* data = new Value[domain.size()];
    auto image = makeArrayImageAdapterFromIterator( data, domain, sub_domain );
    nb++; nbok += checkImage(image) ? 1 : 0;
    delete[] data;
    trace.endBlock();
  }

  {
    trace.beginBlock("Checking ArrayImageAdapter with ImageContainerBySTLVector");
    ImageContainerBySTLVector<Domain, Value> image(domain);
    auto image_view = makeArrayImageAdapterFromImage( image );
    nb++; nbok += checkImage(image_view) ? 1 : 0;
    trace.endBlock();
  }

  {
    trace.beginBlock("Checking ArrayImageAdapter with ImageContainerBySTLVector on sub-domain");
    ImageContainerBySTLVector<Domain, Value> image(domain);
    auto image_view = makeArrayImageAdapterFromImage( image, sub_domain );
    nb++; nbok += checkImage(image_view) ? 1 : 0;
    trace.endBlock();
  }

  bool res = nb == nbok;

  trace.endBlock();
  trace.emphase() << ( res ? "Passed." : "Failed." ) << endl;
  return res ? 0 : 1;
}

/* GNU coding style */
/* vim: set ts=2 sw=2 expandtab cindent cinoptions=>4,n-2,{2,^-2,:2,=2,g0,h2,p5,t0,+2,(0,u0,w1,m1 : */
