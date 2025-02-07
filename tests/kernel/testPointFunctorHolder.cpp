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
 * @date 2018/07/17
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <cmath>

#include "DGtal/kernel/CPointFunctor.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/PointFunctorHolder.h"

#include "DGtalCatch.h"

template <typename T, typename Point>
T kernel(Point const& pt, Point const& center, T radius)
{
  return (pt - center).norm() - radius;
}

template <typename T, typename Point>
struct Functor
{
  Point center;
  T radius;

  Functor(Point const& c, T r)
    : center(c), radius(r)
  {}

  T operator() (Point const& pt) const
    {
      return kernel<T>(pt, center, radius);
    }
};

TEST_CASE( "2D PointFunctorHolder from functor by rvalue", "[2D][functor][rvalue]" )
{
  using namespace DGtal;
  using Point = PointVector<2, int>;
  using Value = double;

  auto fn = functors::holdPointFunctor<Point, Value>( Functor<Value, Point>( Point(1, 0), 1 ) );

  // Checks CPointFunctor concept.
  DGTAL_CONCEPT_CHECK( requires DGtal::concepts::CPointFunctor<decltype(fn)> );
  
  // Checking standard services
  std::cout << fn << std::endl;
  REQUIRE( fn.isValid() );

  // Checking operator()
  REQUIRE( fn(Point(2, 1)) == Approx( std::sqrt(2) - 1. ) );
}

TEST_CASE( "2D PointFunctorHolder from functor by lvalue", "[2D][functor][lvalue]" )
{
  using namespace DGtal;
  using Point = PointVector<2, int>;
  using Value = double;

  const auto functor = Functor<Value, Point>( Point(1, 0), 1 );
  auto fn = functors::holdPointFunctor<Point>( functor ); // auto deduction of the return type

  // Checks CPointFunctor concept.
  DGTAL_CONCEPT_CHECK( requires DGtal::concepts::CPointFunctor<decltype(fn)> );

  // Checking standard services
  std::cout << fn << std::endl;
  REQUIRE( fn.isValid() );

  // Checking operator()
  REQUIRE( fn(Point(2, 1)) == Approx( std::sqrt(2) - 1. ) );
}

TEST_CASE( "2D PointFunctorHolder from lambda by rvalue", "[2D][lambda][rvalue]" )
{
  using namespace DGtal;
  using Point = PointVector<2, int>;
  using Value = double;

  const Point center(1, 0);
  const Value radius = 1;

  auto fn = functors::holdPointFunctor<Point, Value>(
    [&center, &radius] (Point const& pt) { return kernel<Value>(pt, center, radius); }
  );

  // Checks CPointFunctor concept.
  DGTAL_CONCEPT_CHECK( requires DGtal::concepts::CPointFunctor<decltype(fn)> );

  // Checking standard services
  std::cout << fn << std::endl;
  REQUIRE( fn.isValid() );

  // Checking operator()
  REQUIRE( fn(Point(2, 1)) == Approx( std::sqrt(2) - 1. ) );
}

TEST_CASE( "2D PointFunctorHolder from lambda by lvalue", "[2D][lambda][lvalue]" )
{
  using namespace DGtal;
  using Point = PointVector<2, int>;
  using Value = double;

  const Point center(1, 0);
  const Value radius = 1;

  const auto lambda = [&center, &radius] (Point const& pt) { return kernel<Value>(pt, center, radius); };
  auto fn = functors::holdPointFunctor<Point, Value>( lambda );

  // Checks CPointFunctor concept.
  DGTAL_CONCEPT_CHECK( requires DGtal::concepts::CPointFunctor<decltype(fn)> );

  // Checking standard services
  std::cout << fn << std::endl;
  REQUIRE( fn.isValid() );

  // Checking operator()
  REQUIRE( fn(Point(2, 1)) == Approx( std::sqrt(2) - 1. ) );
}
