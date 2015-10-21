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
 * @file testSimpleRandomAccessRangeFromPoint.cpp
 * @ingroup Tests
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 *
 * @date 2015/10/21
 *
 * This file is part of the DGtal library
 */

#include "DGtalCatch.h"

#include <DGtal/kernel/SpaceND.h>
#include <DGtal/kernel/domains/HyperRectDomain.h>
#include <DGtal/kernel/domains/Linearizer.h>
#include <DGtal/images/ImageContainerBySTLVector.h>

#include <algorithm>
#include <cstddef>

TEST_CASE( "Testing SimpleRandomAccess(Const)RangeFromPoint from ImageContainerBySTLVector", "" )
{
  using namespace DGtal;

  typedef double Value;
  typedef SpaceND<2> Space;
  typedef Space::Point Point;
  typedef HyperRectDomain<Space> Domain;
  typedef ImageContainerBySTLVector<Domain, Value> Image;
  typedef Image::Range Range;
  typedef Image::ConstRange ConstRange;
  typedef Linearizer<Domain, ColMajorStorage> Linearizer;

  const Domain domain( Point(1,2), Point(6,5) );
  const Point aPoint(3,4);
  REQUIRE( domain.isInside(aPoint) );

  Image image(domain);
  Image refImage(domain);

  // Initialization
  const double electricity = 1.21; // gigawatts. Great Scott!
  std::size_t cnt = 0;
  for ( Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end; ++it, ++cnt )
    {
      image.setValue(*it, cnt*electricity);
      refImage.setValue(*it, cnt*electricity);
    }

  SECTION( "Testing constant forward iterators" )
    {
      const Range range = image.range();
      REQUIRE( range.end() - range.begin() == domain.size() );
      REQUIRE( ( std::equal(range.begin(), range.end(), refImage.begin()) ) );

      ConstRange crange = image.constRange();
      REQUIRE( crange.end() - crange.begin() == domain.size() );
      REQUIRE( ( std::equal(crange.begin(), crange.end(), refImage.begin()) ) );
    }

  SECTION( "Testing constant forward iterators from a point" )
    {
      const Range range = image.range();
      REQUIRE( range.end() - range.begin(aPoint) == domain.size() - Linearizer::getIndex(aPoint, domain) );
      REQUIRE( std::equal( range.begin(aPoint), range.end(), refImage.begin() + Linearizer::getIndex(aPoint, domain) ) );
      
      ConstRange crange = image.constRange();
      REQUIRE( crange.end() - crange.begin(aPoint) == domain.size() - Linearizer::getIndex(aPoint, domain) );
      REQUIRE( std::equal( crange.begin(aPoint), crange.end(), refImage.begin() + Linearizer::getIndex(aPoint, domain) ) );
    }

  SECTION( "Testing mutable forward iterators" )
    {
      Range range = image.range();

      for ( Range::Iterator it = range.begin(), it_end = range.end(); it != it_end; ++it )
        {
          *it += 1;
        }

      for ( Domain::ConstIterator it = domain.begin(), it_end = domain.end(); it != it_end; ++it )
        {
          refImage.setValue( *it, refImage(*it)+1 );
        }

      REQUIRE( ( std::equal(range.begin(), range.end(), refImage.begin()) ) );
    }
  
  SECTION( "Testing mutable forward iterators from a point" )
    {
      Range range = image.range();

      for ( Range::Iterator it = range.begin(aPoint), it_end = range.end(); it != it_end; ++it )
        {
          *it += 1;
        }

      for ( Domain::ConstIterator it = domain.begin(aPoint), it_end = domain.end(); it != it_end; ++it )
        {
          refImage.setValue( *it, refImage(*it)+1 );
        }

      REQUIRE( ( std::equal(range.begin(), range.end(), refImage.begin()) ) );
    }

  SECTION( "Testing constant reverse iterators" )
    {
      const Range range = image.range();
      REQUIRE( range.rend() - range.rbegin() == domain.size() );
      REQUIRE( ( std::equal(range.rbegin(), range.rend(), refImage.rbegin()) ) );

      ConstRange crange = image.constRange();
      REQUIRE( crange.rend() - crange.rbegin() == domain.size() );
      REQUIRE( ( std::equal(crange.rbegin(), crange.rend(), refImage.rbegin()) ) );
    }

}

