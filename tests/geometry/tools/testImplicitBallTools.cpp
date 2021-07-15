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
 * @ingroup Tests
 * @author
 *
 *
 * @date 30/06/2021
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testPointVector-catch' <p>
 * Aim: simple test of \ref ImplicitBallTools functions with Catch unit test framework.
 */

#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/geometry/tools/ImplicitBallTools.h"

#include "DGtalCatch.h"


using namespace DGtal;

TEST_CASE( "Testing ImplicitBallTools functions" ) {

  typedef ImplicitBall<SpaceND<2> > ImplicitBall2D;
  typedef ImplicitBall<SpaceND<3> > ImplicitBall3D;

  typedef Z2i::Point Point2D;
  typedef Z2i::RealPoint RealPoint2D;
  typedef Z3i::Point Point3D;
  typedef Z3i::RealPoint RealPoint3D;

  SECTION( "Ball From 2 Points Dimension 2 and 3" )
  {
    ImplicitBall2D ball = functions::ballFrom2Points(Point2D(0, 0), Point2D(5, 5));
    CHECK(ball.center() == RealPoint2D(2.5, 2.5));

    ImplicitBall2D ball2 = functions::ballFrom2Points(Point2D(4, 4), Point2D(8, 8));
    CHECK(ball2.center() == RealPoint2D(6, 6));

    ImplicitBall2D ball3 = functions::ballFrom2Points(Point2D(2, 2), Point2D(2, 2));
    CHECK(ball3.center() == RealPoint2D(2, 2));

    ImplicitBall3D ball3D = functions::ballFrom2Points(Point3D(0, 0, 0), Point3D(5, 5, 5));
    CHECK(ball3D.center() == RealPoint3D(2.5, 2.5, 2.5));

    ImplicitBall3D ball4 = functions::ballFrom2Points(Point3D(4, 5, 1), Point3D(2, 3, 7));
    CHECK(ball4.center() == RealPoint3D(3, 4, 4));

    ImplicitBall3D ball3D2 = functions::ballFrom2Points(Point3D(2, 2, 2), Point3D(2, 2, 2));
    CHECK(ball3D2.center() == RealPoint3D(2, 2, 2));
  }

  SECTION( "Ball From 3 Points Dimension 2 and 3" )
  {
    ImplicitBall2D ball = functions::ballFrom3Points(Point2D(0, 0), Point2D(1, 0), Point2D(0, 1));
    CHECK(ball.center() == RealPoint2D(0.5, 0.5));
    
    ImplicitBall3D ball3D3 = functions::ballFrom3Points(Point3D(0, 0, 0), Point3D(1, 0, 0), Point3D(0, 1, 0));
    CHECK(ball3D3.center() == RealPoint3D(0.5, 0.5, 0.5));
  }

  /*SECTION( "Ball From 4 Points")
  {
    ImplicitBall3D ball3D4 = functions::ballFrom4Points(Point3D(0, 0, 0), Point3D(1, 1, 1), Point3D(1, 0, 0), Point3D(0, 1, 0));
    CHECK(ball3D4.center() == RealPoint3D(0.5, 0.5, 0.5));
  }*/

  SECTION( "Trivial Circle" ) {
    
  }
}
