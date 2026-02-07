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
 * @file testLemniscate2D.cpp
 * @ingroup Tests
 * @author Adrien Krähenbühl (\c krahenbuhl@unistra.fr )
 * Laboratoire ICube, UMR 7357, Université de Strasbourg, France
 *
 * @date 2019/07/18
 *
 * Functions for testing the methods of the Lemniscate2D shape.
 * In particular, test the x(), xp() and xpp() methods for cases leadind to
 * a division by 0.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <random>
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/parametric/Lemniscate2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z2i;

typedef Lemniscate2D<Space> Shape;

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

TEST_CASE("Lemniscate2D")
{
  std::uniform_real_distribution<double> unif(-10000,10000);
  std::default_random_engine re;

  SECTION("center()")
    {
      const RealPoint center(unif(re),unif(re));
      Shape shape( center, unif(re) );
      REQUIRE( shape.center() == center );
    }

  SECTION("Lower and upper bounds")
    {
      Shape shape( unif(re), unif(re), unif(re) );
      REQUIRE( shape.getLowerBound() <= shape.getUpperBound() );
    }

  SECTION("x() with Pi parameter.")
    {
  	  Shape shape( unif(re), unif(re), unif(re) );
      REQUIRE_NOTHROW( shape.x(M_PI) );
    }

  SECTION("x() with Pi parameter -> division by 0.")
    {
  	  Shape shape( unif(re), unif(re), unif(re) );
      REQUIRE_NOTHROW( shape.x(M_PI) );
    }

  SECTION("xp() with Pi parameter -> division by 0.")
    {
  	  Shape shape( unif(re), unif(re), unif(re) );
      REQUIRE_NOTHROW( shape.xp(M_PI) );
    }

  SECTION("xpp() with Pi parameter -> division by 0.")
    {
  	  Shape shape( unif(re), unif(re), unif(re) );
      REQUIRE_NOTHROW( shape.xp(M_PI) );
    }
}
