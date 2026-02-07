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
 * @author Baptiste GENEST (\c baptistegenest@gmail.com )
 * internship at Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/15
 *
 * Functions for testing class VectorsInHeat.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/dec/PolygonalCalculus.h"
#include "DGtal/dec/VectorsInHeat.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/MeshHelpers.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

TEST_CASE( "Testing VectorsInHeat" )
{
  typedef SurfaceMesh< RealPoint,RealPoint > Mesh;
  std::vector<RealPoint> positions = { RealPoint( 0, 0, 0 ) ,
    RealPoint( 1, 0, 0 ) ,
    RealPoint( 0, 1, 0 ) ,
    RealPoint( 1, 1, 0 ) ,
    RealPoint( 0, 0, 1 ) ,
    RealPoint( 1, 0, 1 ) ,
    RealPoint( 0, 1, 1 ) ,
    RealPoint( 1, 1, 1 ) ,
    RealPoint( 1, 0, 2 ) ,
    RealPoint( 0, 0, 2 ) };
  std::vector<Mesh::Vertices> faces = { { 1, 0, 2, 3 },
    { 0, 1, 5, 4 } ,
    { 1, 3, 7, 5 } ,
    { 3, 2, 6, 7 } ,
    { 2, 0, 4, 6 } ,
    { 4, 5, 8, 9 } };

  Mesh box(positions.cbegin(), positions.cend(),
           faces.cbegin(), faces.cend());

  PolygonalCalculus<RealPoint,RealVector> boxCalculus(box);


  SECTION("Construction and basic operators")
  {
    VectorsInHeat<PolygonalCalculus<RealPoint,RealVector>> heat(boxCalculus);
    REQUIRE( heat.isValid() == false );

    trace.beginBlock("init solvers");
    heat.init(0.1);
    trace.endBlock();
    REQUIRE( heat.isValid()  );

    heat.addSource(0,Eigen::Vector3d(0.1,0.2,0.3));
    std::vector<VectorsInHeat<PolygonalCalculus<RealPoint,RealVector>>::Vector> d = heat.compute();
    REQUIRE( d.size() == positions.size() );
    REQUIRE( d[5][0] == Approx(-0.111302) );

    heat.clearSource();
    VectorsInHeat<PolygonalCalculus<RealPoint,RealVector>>::Vector sources=heat.vectorSource();
    REQUIRE( sources.sum() == 0);
  }
}
/** @ingroup Tests **/
