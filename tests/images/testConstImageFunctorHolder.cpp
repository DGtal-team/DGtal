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
 * @file
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2018/07/10
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <cmath>


#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/CConstImage.h"

#include "DGtal/images/ConstImageFunctorHolder.h"

#include "DGtalCatch.h"

template < typename TPoint, typename TDomain >
inline
typename TPoint::Scalar unary_kernel( TPoint const& pt, TDomain const&, typename TPoint::Scalar cst )
{
  return cst * pt.norm();
}

template <typename TPoint, typename TDomain>
inline
typename TPoint::Scalar binary_kernel( TPoint const& pt, TDomain const& domain, typename TPoint::Scalar cst )
{
  return cst * (pt - domain.lowerBound()).norm();
}

template < DGtal::concepts::CConstImage TImage, typename TFunction >
void checkImage( TImage const& anImage, TFunction const& fn )
{
  using Image = TImage;
  using Domain = typename Image::Domain;

  // Image's domain
  Domain const& domain = anImage.domain();
  REQUIRE( !domain.isEmpty() );

  // Checking standard services
  std::cout << anImage << std::endl;
  REQUIRE( anImage.isValid() );

  // Checking operator()
  for ( auto const& pt : domain )
    REQUIRE( anImage(pt) == fn(pt, domain) );

  // Checking forward range
    {
      auto pt_it = domain.begin();
      auto im_it = anImage.constRange().begin();
      for ( ; pt_it != domain.end(); ++pt_it, ++im_it )
          REQUIRE( *im_it == fn(*pt_it, domain) );

      REQUIRE( im_it == anImage.constRange().end() );
    }

  // Checking reverse range
    {
      auto pt_it = domain.rbegin();
      auto im_it = anImage.constRange().rbegin();
      for ( ; pt_it != domain.rend(); ++pt_it, ++im_it )
          REQUIRE( *im_it == fn(*pt_it, domain) );

      REQUIRE( im_it == anImage.constRange().rend() );
    }
}

// Unary functor
struct UnaryFunctor
{
  double cst;
  explicit UnaryFunctor(double c) : cst(c) {}

  template <typename Point>
  double operator() (Point const& pt) const
    {
      return unary_kernel(pt, 0, cst);
    }
};

// Binary functor
struct BinaryFunctor
{
  double cst;
  explicit BinaryFunctor(double c) : cst(c) {}

  template <typename Point, typename Domain>
  double operator() (Point const& pt, Domain const &d) const
    {
      return binary_kernel(pt, d, cst);
    }
};

TEST_CASE( "2D Image from unary functor by rvalue", "[2D][functor][unary][rvalue]" )
{
  using namespace DGtal;
  using Space = SpaceND<2, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain(Point{-10, -15}, Point{20, 25});
  const double cst = 3.5;
  auto image = functors::holdConstImageFunctor<double>( domain, UnaryFunctor(cst) ); // Specifying explicitly the returned value type.
  checkImage(image, [&cst] (Point pt, Domain d) { return unary_kernel(pt, d, cst); });
}

TEST_CASE( "2D Image from binary functor by rvalue", "[2D][functor][binary][rvalue]" )
{
  using namespace DGtal;
  using Space = SpaceND<2, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain(Point{-10, -15}, Point{20, 25});
  const double cst = 3.5;
  auto image = functors::holdConstImageFunctor( domain, BinaryFunctor(cst) );
  checkImage(image, [&cst] (Point pt, Domain d) { return binary_kernel(pt, d, cst); });
}

TEST_CASE( "2D Image from binary functor by lvalue", "[2D][functor][binary][lvalue]" )
{
  using namespace DGtal;
  using Space = SpaceND<2, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain(Point{-10, -15}, Point{20, 25});
  const double cst = 3.5;
  const auto fn = BinaryFunctor(cst);
  auto image = functors::holdConstImageFunctor( domain, fn );
  checkImage(image, [&cst] (Point pt, Domain d) { return binary_kernel(pt, d, cst); });
}

TEST_CASE( "2D Image from binary lambda by rvalue", "[2D][lambda][binary][rvalue]" )
{
  using namespace DGtal;
  using Space = SpaceND<2, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain(Point{-10, -15}, Point{20, 25});
  const double cst = 3.5;
  auto image = functors::holdConstImageFunctor( domain, [cst] (Point const& pt, Domain const& d) { return binary_kernel(pt, d, cst); } );
  checkImage(image, [&cst] (Point pt, Domain d) { return binary_kernel(pt, d, cst); });
}

TEST_CASE( "2D Image from binary std::function by lvalue", "[2D][function][binary][lvalue]" )
{
  using namespace DGtal;
  using Space = SpaceND<2, int>;
  using Domain = HyperRectDomain<Space>;
  using Point = typename Domain::Point;

  const Domain domain(Point{-10, -15}, Point{20, 25});
  const double cst = 3.5;
  std::function<double(Point, Domain)> fn = [cst] (Point const& pt, Domain const& d) { return binary_kernel(pt, d, cst); };
  auto image = functors::holdConstImageFunctor( domain, fn );
  checkImage(image, [&cst] (Point pt, Domain d) { return binary_kernel(pt, d, cst); });
}
