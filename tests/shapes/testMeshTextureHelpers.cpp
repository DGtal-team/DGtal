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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/03/24
 *
 * Functions for testing class MeshTextureHelpers.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/TriangulatedSurface.h"
#include "DGtal/shapes/MeshTextureHelpers.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class MeshTextureHelpers.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing MeshTextureHelpers" )
{
  typedef PointVector<3,double>             RealPoint;
  typedef TriangulatedSurface< RealPoint >  TriMesh;
  TriMesh mesh;
  mesh.addVertex( RealPoint( 1, 0, 0 ) );
  mesh.addVertex( RealPoint( 0, 1, 0 ) );
  mesh.addVertex( RealPoint( 0, 0, 1 ) );
  mesh.addTriangle( 0, 1, 2 );
  mesh.build();
  
  SECTION("Testing Barycentric Coordinates")
  {
    //middle not in the plane
    RealPoint p(0.5,0.5,0.5);
    RealPoint lambda(0.333333,0.3333333,0.333333);
    RealPoint bary = MeshTextureHelpers<RealPoint>::getBarycentricCoordinatesInFace(mesh,0, p);
    INFO("Bary p"<<bary);
    REQUIRE(bary[0] == Approx(lambda[0]));
    REQUIRE(bary[1] == Approx(lambda[1]));
    REQUIRE(bary[2] == Approx(lambda[2]));
    
    //vertex
    RealPoint p2(1,0,0);
    bary = MeshTextureHelpers<RealPoint>::getBarycentricCoordinatesInFace(mesh,0, p2);
    RealPoint reconstruction = MeshTextureHelpers<RealPoint>::getPointFromBarycentricCoordinatesInFace(mesh,0, bary);
    INFO("Bary p2 "<<bary);
    INFO("p2 reco "<<reconstruction);
    REQUIRE(p2[0] == Approx(reconstruction[0]));
    REQUIRE(p2[1] == Approx(reconstruction[1]));
    REQUIRE(p2[2] == Approx(reconstruction[2]));
    
    //on edge
    RealPoint p3(.5,.5,0);
    bary = MeshTextureHelpers<RealPoint>::getBarycentricCoordinatesInFace(mesh,0, p3);
    reconstruction = MeshTextureHelpers<RealPoint>::getPointFromBarycentricCoordinatesInFace(mesh,0, bary);
    INFO("Bary "<<bary);
    INFO("p3 reco "<<reconstruction);
    REQUIRE(p3[0] == Approx(reconstruction[0]));
    REQUIRE(p3[1] == Approx(reconstruction[1]));
    REQUIRE(p3[2] == Approx(reconstruction[2]));
  }
  
  SECTION("Test OBJ loader")
  {
    TriMesh cube;
    MeshTextureHelpers<RealPoint>::UVMap texture;
    MeshTextureHelpers<RealPoint>::UVMesh uvMesh;
    std::tie( cube , uvMesh, texture) = MeshTextureHelpers<RealPoint>::loadOBJWithTextureCoord(testPath + "samples/cubetext.obj");
    REQUIRE(cube.nbVertices() == 8);
    REQUIRE(cube.nbFaces() == 12);
  }

}

/** @ingroup Tests **/
