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
 * @file testPolygonalCalculus.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/09/02
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

#include "DGtal/dec/PolygonalCalculus.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/MeshHelpers.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PolygonalCalculus.
///////////////////////////////////////////////////////////////////////////////

RealPoint vecToRealPoint(PolygonalCalculus<SurfaceMesh<RealPoint,RealPoint > >::Vector &v )
{
  return RealPoint(v(0),v(1),v(2));
}


TEST_CASE( "Testing PolygonalCalculus" )
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
  
  PolygonalCalculus<Mesh> boxCalculus(box);
  
  SECTION("Construction and basic operators")
    {
      REQUIRE( boxCalculus.isValid() );
      REQUIRE( boxCalculus.nbVertices() == positions.size() );

      PolygonalCalculus<Mesh>::Face f = 0;
      auto x = boxCalculus.X(f);
      auto d = boxCalculus.D(f);
      auto a = boxCalculus.A(f);
      
      //Checking X
      PolygonalCalculus<Mesh>::Vector vec = x.row(0);
      REQUIRE( vecToRealPoint(vec ) == positions[1]);
      vec = x.row(1);
      REQUIRE( vecToRealPoint(vec ) == positions[0]);
      vec = x.row(2);
      REQUIRE( vecToRealPoint(vec ) == positions[2]);
      vec = x.row(3);
      REQUIRE( vecToRealPoint(vec ) == positions[3]);

      //Some D and A
      REQUIRE( d(1,1) == -1 );
      REQUIRE( d(0,1) == 1 );
      REQUIRE( d(0,2) == 0 );

      REQUIRE( a(1,1) == 0.5 );
      REQUIRE( a(0,1) == 0.5 );
      REQUIRE( a(0,2) == 0 );
      
      auto vectorArea = boxCalculus.vectorArea(f);
      
      //Without correction, this should match
      for(auto f=0; f<6; ++f)
        REQUIRE( boxCalculus.faceArea(f) == box.faceArea(f) );
     
      box.computeFaceNormalsFromPositions();
      for(auto f=0; f<6; ++f)
      {
        auto cn = boxCalculus.faceNormalAsDGtalVector(f);
        auto n = box.faceNormal(f);
        REQUIRE(  cn == n  );
      }
      
      RealPoint c = boxCalculus.centroidAsDGtalPoint(f);
      RealPoint expected(0.5,0.5,0.0);
      REQUIRE(c == expected);
    }
  
  SECTION("Derivatives")
  {
    PolygonalCalculus<Mesh>::Face f = 0;
    auto d = boxCalculus.D(f);
    
    auto nf = boxCalculus.faceDegree(f);
    PolygonalCalculus<Mesh>::Vector phi(nf),expected(nf);
    phi << 1.0, 3.0, 2.0, 6.0;
    expected << 2,-1,4,-5;
    auto dphi = d*phi;  // n_f x 1 matrix
    REQUIRE(dphi == expected);
    
  }
  
  SECTION("Structural propertes")
  {
    PolygonalCalculus<Mesh>::Face f = 0;
    auto nf =  boxCalculus.faceDegree(f);
    PolygonalCalculus<Mesh>::Vector phi(nf);
    phi << 1.0, 3.0, 2.0, 6.0;
    
    auto G = boxCalculus.gradient(f);
    auto gphi = G*phi;
    auto coG = boxCalculus.coGradient(f);
    auto cogphi = coG*phi;
    
    // grad . cograd == 0
    REQUIRE( gphi.dot(cogphi) == 0.0);
        
    //    Gf = Uf Df
    REQUIRE( G == boxCalculus.U(f)*boxCalculus.D(f));
    
    //    UV = I - nn^t (lemma4)
    PolygonalCalculus<Mesh>::Vector n = boxCalculus.faceNormal(f);
    REQUIRE( boxCalculus.U(f)*boxCalculus.V(f) == PolygonalCalculus<Mesh>::DenseMatrix::Identity(3,3) - n*n.transpose() );
    
    //    P^2 = P (lemma6)
    auto P = boxCalculus.P(f);
    REQUIRE( P*P == P);
    
    //    PV=0 (lemma5)
    REQUIRE( (P*boxCalculus.V(f)).norm() == 0.0);
  }
  
  SECTION("Local Laplace-Beltrami")
  {
    PolygonalCalculus<Mesh>::Face f = 0;
    auto nf = box.incidentVertices(f).size();
    
    auto L = boxCalculus.LaplaceBeltrami(f);
    PolygonalCalculus<Mesh>::Vector phi(nf),expected(nf);
    phi << 1.0, 1.0, 1.0, 1.0;
    expected << 0,0,0,0;
    auto lphi = L*phi;
    REQUIRE( lphi == expected);
  }
  SECTION("Check lumped mass matrix")
  {
    PolygonalCalculus<Mesh>::SparseMatrix M = boxCalculus.globalLumpedMassMatrix();
    double a=0.0;
    for(auto v=0; v < box.nbVertices(); ++v )
      a += M.coeffRef(v,v);
    
    double fa=0.0;
    for(auto f=0; f < box.nbFaces(); ++f )
      fa += box.faceArea(f);
    REQUIRE( a == fa );
  }
  
  SECTION("Checking cache")
  {
    auto cacheU = boxCalculus.getOperatorCacheMatrix( [&](const PolygonalCalculus<Mesh>::Face f){ return boxCalculus.U(f);} );
    REQUIRE( cacheU.size() == 6 );
    
    auto cacheC = boxCalculus.getOperatorCacheVector( [&](const PolygonalCalculus<Mesh>::Face f){ return boxCalculus.centroid(f);} );
    REQUIRE( cacheC.size() == 6 );
  }
}

/** @ingroup Tests **/
