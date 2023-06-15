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
 * @file testlibigl.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2023/06/14
 *
 * Functions for testing class PolygonalCalculus.
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

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PolygonalCalculus.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing WindingNumbersShape" )
{
  using WNShape = WindingNumbersShape<Z3i::Space>;

  SECTION("Checking concept")
    {
      BOOST_CONCEPT_ASSERT( (DGtal::concepts::CEuclideanOrientedShape<WNShape>) );
    }
  
  SECTION("Simple testof the API")
  {
    Eigen::MatrixXd points(4,3);
    points << 0,0,0,
              1,1,0,
              0,1,0,
              1,1,1;
    Eigen::MatrixXd normals(4,3);
    normals << 0,0,-1,
               0,0,-1,
               0,0,-1,
               0,0,1;

    WNShape wnshape(points,normals);

    RealPoint p(1.0,1.0,1.0);
    DGtal::Orientation ori = wnshape.orientation(p);
    std::cout<<ori<<" "<<p<<std::endl;
  }
  
};

/** @ingroup Tests **/
