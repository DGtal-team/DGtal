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
 * @file testShapeMoveCenter.cpp
 * @ingroup Tests
 * @author Adrien Krähenbühl (\c krahenbuhl@unistra.fr )
 * Laboratoire ICube, UMR 7357, Université de Strasbourg, France
 *
 * @date 2019/07/08
 *
 * Functions for testing the moveTo() method of star shapes.
 *
 * This file is part of the DGtal library.
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <random>
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/parametric/AccFlower2D.h"
#include "DGtal/shapes/parametric/Astroid2D.h"
#include "DGtal/shapes/parametric/Ball2D.h"
#include "DGtal/shapes/parametric/Ellipse2D.h"
#include "DGtal/shapes/parametric/Flower2D.h"
#include "DGtal/shapes/parametric/Lemniscate2D.h"
#include "DGtal/shapes/parametric/NGon2D.h"
#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/shapes/implicit/ImplicitHyperCube.h"
#include "DGtal/shapes/implicit/ImplicitNorm1Ball.h"
#include "DGtal/shapes/implicit/ImplicitRoundedHyperCube.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z2i;


typedef AccFlower2D<Space>  MyAccFlower;
typedef Astroid2D<Space>    MyAstroid;
typedef Ball2D<Space>       MyBall;
typedef Ellipse2D<Space>    MyEllipse;
typedef Flower2D<Space>     MyFlower;
typedef Lemniscate2D<Space> MyLemniscate;
typedef NGon2D<Space>       MyNGon;

typedef ImplicitBall<Space>             BallImplicit;
typedef ImplicitHyperCube<Space>        HyperCubeImplicit;
typedef ImplicitNorm1Ball<Space>        Norm1BallImplicit;
typedef ImplicitRoundedHyperCube<Space> RoundedHyperCubeImplicit;

template<typename Shape>
Shape createShape( const RealPoint& center, const RealPoint& radii );

template<>
MyAccFlower createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyAccFlower( center, radii[0], radii[1], 12, 2. );
}

template<>
MyAstroid createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyAstroid( center, radii[0], radii[1] );
}

template<>
MyBall createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyBall( center, radii[0] );
}

template<>
MyEllipse createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyEllipse( center, radii[0], radii[1], 2. );
}

template<>
MyFlower createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyFlower( center, radii[0], radii[1], 5, 2. );
}

template<>
MyLemniscate createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyLemniscate( center, radii[0] );
}

template<>
MyNGon createShape( const RealPoint& center, const RealPoint& radii )
{
  return MyNGon( center, radii[0], 20, 2. );
}

template<>
BallImplicit createShape( const RealPoint& center, const RealPoint& radii )
{
  return BallImplicit( center, radii[0] );
}

template<>
HyperCubeImplicit createShape( const RealPoint& center, const RealPoint& radii )
{
  return HyperCubeImplicit( center, radii[0] );
}

template<>
Norm1BallImplicit createShape( const RealPoint& center, const RealPoint& radii )
{
  return Norm1BallImplicit( center, radii[0] );
}

template<>
RoundedHyperCubeImplicit createShape( const RealPoint& center, const RealPoint& radii )
{
  return RoundedHyperCubeImplicit( center, radii[0], 3. );
}

std::uniform_real_distribution<double> unif(-1000000.,1000000.);
std::default_random_engine re;

TEMPLATE_TEST_CASE("Star shapes", "move() method",
                    MyAccFlower, MyAstroid, MyBall, MyEllipse, MyFlower, MyLemniscate, MyNGon,
                    BallImplicit, HyperCubeImplicit, Norm1BallImplicit, RoundedHyperCubeImplicit)
{
  const double centerX = unif(re);
  const double centerY = unif(re);
  const double radiusX = unif(re);
  const double radiusY = unif(re);

  TestType shape = createShape<TestType>( RealPoint(centerX, centerY), RealPoint(radiusX, radiusY) );

  SECTION("Center coordinates")
    {
      REQUIRE( shape.center() == RealPoint(centerX, centerY) );
    }

  SECTION("Change center position")
    {
      const double newCenterX = unif(re);
      const double newCenterY = unif(re);

      shape.moveTo( RealPoint( newCenterX, newCenterY ) );

      REQUIRE( shape.center() == RealPoint( newCenterX, newCenterY ) );
    }
}
#endif
