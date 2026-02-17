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
 * @file shapes/testWindingNumbersShape.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2023/06/14
 *
 * Functions for testing class WindingNumbersShape.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"

#include <DGtal/shapes/WindingNumbersShape.h>
#include <DGtal/shapes/GaussDigitizer.h>

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class WindingNumbersShape.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing WindingNumbersShape" )
{
  using WNShape = WindingNumbersShape<Z3i::Space>;

  SECTION("Checking concept")
    {
      BOOST_CONCEPT_ASSERT( (DGtal::concepts::CEuclideanOrientedShape<WNShape>) );
    }

  SECTION("Simple test of the API")
  {
    Eigen::MatrixXd points(4,3);
    points << 0,0,0,
              0,1,0,
              1,0,0,
              1,1,1;
    Eigen::MatrixXd normals(4,3);
    normals << 0,0,-1,
               0,0,-1,
               0,0,-1,
               0,0,1;

    WNShape wnshape(points,normals);

    RealPoint p(-2.0,-2.0,-2.0);
    DGtal::Orientation ori = wnshape.orientation(p);
    REQUIRE( ori == DGtal::OUTSIDE);

    p = RealPoint(2.0,2.0,2.0);
    ori = wnshape.orientation(p);
    REQUIRE( ori == DGtal::OUTSIDE);

    RealPoint q= RealPoint(.2,.2,.2);
    auto ori2 = wnshape.orientation(q);
    REQUIRE( ori2 == DGtal::INSIDE);
  }

 SECTION("Tesing with the GaussDigitizer")
 {
     Eigen::MatrixXd points(4,3);
     points << 0,0,0,
               0,1,0,
               1,0,0,
               1,1,1;
     Eigen::MatrixXd normals(4,3);
     normals << 0,0,-1,
                0,0,-1,
                0,0,-1,
                0,0,1;

     WNShape wnshape(points,normals);
     GaussDigitizer<Z3i::Space, WNShape> gauss;
     gauss.attach(wnshape);
     gauss.init(Z3i::RealPoint(0,0,0),Z3i::RealPoint(1.5,1.5,1.5), 0.5);
     auto cpt=0;
     for(auto p: gauss.getDomain())
         if (gauss.orientation(p) == DGtal::INSIDE)
             ++cpt;
     REQUIRE( cpt == 8);
 }
};

/** @ingroup Tests **/
