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
 * @date 2018/07/09
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <functional>

#include "DGtal/base/FunctorHolder.h"
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"

#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace DGtal::functors;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
// Functor and functions

struct Functor
{
  double cst;

  explicit Functor(double c) : cst(c) {}
  inline double operator() (double v) const { return v + cst; }
};

double add(double v)
{
  return v + 1.5;
}

///////////////////////////////////////////////////////////////////////////////
// Test cases

TEST_CASE( "Holding a lambda", "[lambda]" )
{
  double cst = 1.5;

  // Holding an unary lambda by mutable lvalue
  {
    auto fn = [&cst] (double v) { return v + cst; };
    auto holder = holdFunctor(fn);
    
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding an unary lambda by constant lvalue
  {
    const auto fn = [&cst] (double v) { return v + cst; };
    auto holder = holdFunctor(fn);
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding an unary lambda by rvalue
  {
    auto holder = holdFunctor( [&cst] (double v) { return v + cst; } );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding an unary mutable lambda by rvalue
  {
    auto holder = holdFunctor( [cst] (double v) mutable { return v + cst; } );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding a binary lambda by rvalue
  {
    auto holder = holdFunctor( [] (double v, double c) { return v + c; } );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    REQUIRE( holder(0.5, 1.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5, 1.5) == 2.0 );
  }
}

TEST_CASE( "Holding a functor", "[functor]" )
{
  double cst = 1.5;

  // Holding an unary functor by mutable lvalue
  {
    auto fn = Functor(cst);
    auto holder = holdFunctor(fn);
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding an unary functor by constant lvalue
  {
    const auto fn = Functor(cst);
    auto holder = holdFunctor(fn);
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding an unary functor by rvalue
  {
    auto holder = holdFunctor( Functor(cst) );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }
}

TEST_CASE( "Holding a function", "[function]" )
{
  // Holding a function by reference
  {
    auto holder = holdFunctor( add );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }

  // Holding a function by address
  {
    auto holder = holdFunctor( &add );
    BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
    DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
    REQUIRE( holder(0.5) == 2.0 );
    auto holder2 = holder;
    REQUIRE( holder2(0.5) == 2.0 );
  }
}

TEST_CASE( "Holding a std::function", "[std::function]" )
{
  double cst = 1.5;

  // Holding a std::function
  std::function<double(double)> fn = Functor(cst);
  auto holder = holdFunctor( fn );
  BOOST_CONCEPT_ASSERT(( boost::Assignable<decltype(holder)> ));
  DGTAL_CONCEPT_CHECK( requires concepts::CUnaryFunctor<decltype(holder), double, double> );
  REQUIRE( holder(0.5) == 2.0 );
  auto holder2 = holder;
  REQUIRE( holder2(0.5) == 2.0 );
}
