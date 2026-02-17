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
 * @file testAstroid2D.cpp
 * @ingroup Tests
 * @author Adrien Krähenbühl (\c krahenbuhl@unistra.fr )
 * Laboratoire ICube, UMR 7357, Université de Strasbourg, France
 *
 * @date 2019/07/18
 *
 * Functions for testing the methods of the Astroid2D shape.
 * In particular, test the parameter() method for cas leadind to
 * a division by 0.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <random>
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/parametric/Astroid2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z2i;


typedef Astroid2D<Space> Shape;

double notNullValue(
	std::uniform_real_distribution<double>& unif,
	std::default_random_engine& re )
{
  double res;
  do {
    res = unif(re);
  }
  while ( res == Approx(0.) );
  return res;
}

RealPoint notNullRealPoint(
	std::uniform_real_distribution<double>& unif,
	std::default_random_engine& re )
{
  return RealPoint(notNullValue(unif,re),notNullValue(unif,re));
}

TEST_CASE("Astroid2D")
{
  std::uniform_real_distribution<double> unif(-10000,10000);
  std::default_random_engine re;

  SECTION("center()")
    {
      const RealPoint center(unif(re),unif(re));
      Shape shape( center, unif(re), unif(re) );
	   REQUIRE( shape.center() == center );
  	}

  SECTION("Lower and upper bounds")
    {
      Shape shape( unif(re), unif(re), unif(re), unif(re) );
      REQUIRE( shape.getLowerBound() <= shape.getUpperBound() );
    }

  SECTION("parameter() with null x-radius -> division by 0.")
    {
  	  Shape shape( unif(re), unif(re), 0., notNullValue(unif,re) );
      REQUIRE_NOTHROW( shape.parameter(notNullRealPoint(unif,re)) );
    }

  SECTION("parameter() with null y-radius -> division by 0.")
    {
  	  Shape shape( unif(re), unif(re), notNullValue(unif,re), 0. );
      REQUIRE_NOTHROW( shape.parameter(notNullRealPoint(unif,re)) );
    }

  SECTION("parameter() with null x-radius and y-radius -> undefined behaviour.")
    {
  	  Shape shape( unif(re), unif(re), 0., 0. );
      REQUIRE_NOTHROW( shape.parameter(notNullRealPoint(unif,re)) );
    }

  SECTION("parameter() where (x,y) parameter with x == center -> pi/2 or 3*pi/2")
    {
      const double centerX = unif(re);
  	  Shape shape( centerX, unif(re), unif(re), unif(re) );
	    const RealPoint point( centerX, unif(re) );
      REQUIRE_NOTHROW( shape.parameter(point) );
      double res = shape.parameter(point);
      REQUIRE_THAT( res, Catch::WithinAbs(M_PI_2,DBL_EPSILON) || Catch::WithinAbs(3*M_PI_2,DBL_EPSILON) );
    }

  SECTION("parameter() with point parameter with null y -> 0. or pi")
    {
      const double centerY = unif(re);
  	  Shape shape( unif(re), centerY, unif(re), unif(re) );
	    const RealPoint point( unif(re), centerY );
      REQUIRE_NOTHROW( shape.parameter(point) );
      double res = shape.parameter(point);
      REQUIRE_THAT( res, Catch::WithinAbs(0.,DBL_EPSILON) || Catch::WithinAbs(M_PI,DBL_EPSILON) );
    }
}
