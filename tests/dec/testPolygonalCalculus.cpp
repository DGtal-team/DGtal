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
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
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
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/math/linalg/DirichletConditions.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PolygonalCalculus.
///////////////////////////////////////////////////////////////////////////////

RealPoint vecToRealPoint(PolygonalCalculus<RealPoint,RealPoint >::Vector &v )
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

  PolygonalCalculus< RealPoint,RealVector > boxCalculus(box);

  SECTION("Construction and basic operators")
    {
      REQUIRE( boxCalculus.isValid() );
      REQUIRE( boxCalculus.nbVertices() == positions.size() );

      PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
      auto x = boxCalculus.X(f);
      auto d = boxCalculus.D(f);
      auto a = boxCalculus.A(f);

      //Checking X
      PolygonalCalculus< RealPoint,RealVector >::Vector vec = x.row(0);
      REQUIRE( vecToRealPoint(vec ) == positions[1]);
      vec = x.row(1);
      REQUIRE( vecToRealPoint(vec ) == positions[0]);
      vec = x.row(2);
      REQUIRE( vecToRealPoint(vec ) == positions[2]);
      vec = x.row(3);
      REQUIRE( vecToRealPoint(vec ) == positions[3]);

      trace.info()<< boxCalculus <<std::endl;

      //Some D and A
      REQUIRE( d(1,1) == -1 );
      REQUIRE( d(0,1) == 1 );
      REQUIRE( d(0,2) == 0 );

      REQUIRE( a(1,1) == 0.5 );
      REQUIRE( a(0,1) == 0.5 );
      REQUIRE( a(0,2) == 0 );

      auto vectorArea = boxCalculus.vectorArea(f);

      //Without correction, this should match
      for(auto ff=0; ff<6; ++ff)
        REQUIRE( boxCalculus.faceArea(ff) == box.faceArea(ff) );

      box.computeFaceNormalsFromPositions();
      for(auto ff=0; ff<6; ++ff)
      {
        auto cn = boxCalculus.faceNormalAsDGtalVector(ff);
        auto n = box.faceNormal(ff);
        REQUIRE(  cn == n  );
      }

      RealPoint c = boxCalculus.centroidAsDGtalPoint(f);
      RealPoint expected(0.5,0.5,0.0);
      REQUIRE(c == expected);
    }

  SECTION("Derivatives")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto d = boxCalculus.D(f);

    auto nf = boxCalculus.faceDegree(f);
    PolygonalCalculus< RealPoint,RealVector >::Vector phi(nf),expected(nf);
    phi << 1.0, 3.0, 2.0, 6.0;
    expected << 2,-1,4,-5;
    auto dphi = d*phi;  // n_f x 1 matrix
    REQUIRE(dphi == expected);

  }

  SECTION("Structural properties")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto nf =  boxCalculus.faceDegree(f);
    PolygonalCalculus< RealPoint,RealVector >::Vector phi(nf);
    phi << 1.0, 3.0, 2.0, 6.0;

    auto G = boxCalculus.gradient(f);
    auto gphi = G*phi;
    auto coG = boxCalculus.coGradient(f);
    auto cogphi = coG*phi;

    // grad . cograd == 0
    REQUIRE( gphi.dot(cogphi) == 0.0);

    //    Gf = Uf Df
    REQUIRE( G == boxCalculus.sharp(f)*boxCalculus.D(f));

    //    UV = I - nn^t (lemma4)
    PolygonalCalculus< RealPoint,RealVector >::Vector n = boxCalculus.faceNormal(f);
    REQUIRE( boxCalculus.sharp(f)*boxCalculus.flat(f) == PolygonalCalculus< RealPoint,RealVector >::DenseMatrix::Identity(3,3) - n*n.transpose() );

    //    P^2 = P (lemma6)
    auto P = boxCalculus.P(f);
    REQUIRE( P*P == P);

    //    PV=0 (lemma5)
    REQUIRE( (P*boxCalculus.flat(f)).norm() == 0.0);
  }

  SECTION("Div / Curl")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto curl = boxCalculus.curl(f);
    //Not a great test BTW
    REQUIRE(curl.norm() == 2.0);
  }

  SECTION("Local Laplace-Beltrami")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto nf = box.incidentVertices(f).size();

    auto L = boxCalculus.laplaceBeltrami(f);
    PolygonalCalculus< RealPoint,RealVector >::Vector phi(nf),expected(nf);
    phi << 1.0, 1.0, 1.0, 1.0;
    expected << 0,0,0,0;
    auto lphi = L*phi;
    REQUIRE( lphi == expected);
  }

  SECTION("Local Connection-Laplace-Beltrami")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto nf = box.incidentVertices(f).size();

    auto L = boxCalculus.connectionLaplacian(f);
    PolygonalCalculus< RealPoint,RealVector >::Vector phi(2*nf);
    phi << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    //since connection laplacian transports the phi vectors to the face,
    //it's not expected to have 0 since these vectors aren't actually the same
    auto lphi = L*phi;
    //but we can still check that L is semi-definite
    double det = L.determinant()+1.;
    REQUIRE( det == Approx(1.0));
    REQUIRE( lphi[2] == Approx(-3.683));
  }

  SECTION("Covariant Operators")
  {
    PolygonalCalculus< RealPoint,RealVector >::Face f = 0;
    auto nf = box.incidentVertices(f).size();

    PolygonalCalculus< RealPoint,RealVector >::Vector phi(2*nf);
    phi << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    auto CG = boxCalculus.covariantGradient(f,phi);
    auto CP = boxCalculus.covariantProjection(f,phi);

    //check sizes
    REQUIRE( CG.rows() == 2);
    REQUIRE( CG.cols() == 2);
    REQUIRE( CP.rows() == (Eigen::Index)faces[f].size());
    REQUIRE( CP.cols() == 2);

    REQUIRE( CG(0,0) == Approx(0.707106));
    REQUIRE( CP(0,0) == Approx(1.224744));
  }
  SECTION("Check lumped mass matrix")
  {
    PolygonalCalculus< RealPoint,RealVector >::SparseMatrix M = boxCalculus.globalLumpedMassMatrix();
    double a=0.0;
    for( PolygonalCalculus< RealPoint,RealVector >::MySurfaceMesh::Index v=0; v < box.nbVertices(); ++v )
      a += M.coeffRef(v,v);

    double fa=0.0;
    for( PolygonalCalculus< RealPoint,RealVector >::MySurfaceMesh::Index f=0; f < box.nbFaces(); ++f )
      fa += box.faceArea(f);
    REQUIRE( a == fa );
  }

  SECTION("Checking cache")
  {
    auto cacheU = boxCalculus.getOperatorCacheMatrix( [&](const PolygonalCalculus< RealPoint,RealVector >::Face f){ return boxCalculus.sharp(f);} );
    REQUIRE( cacheU.size() == 6 );

    auto cacheC = boxCalculus.getOperatorCacheVector( [&](const PolygonalCalculus< RealPoint,RealVector >::Face f){ return boxCalculus.centroid(f);} );
    REQUIRE( cacheC.size() == 6 );
  }

  SECTION("Internal cache")
  {
    PolygonalCalculus< RealPoint,RealVector > boxCalculusCached(box,true);
    trace.info()<< boxCalculusCached <<std::endl;

    trace.beginBlock("Without cache");
    PolygonalCalculus< RealPoint,RealVector >::SparseMatrix L(box.nbVertices(),box.nbVertices());
    for(auto i=0; i < 1000 ; ++i)
      L += i*boxCalculus.globalLaplaceBeltrami();
    auto tps = trace.endBlock();

    trace.beginBlock("With cache");
    PolygonalCalculus< RealPoint,RealVector >::SparseMatrix LC(box.nbVertices(),box.nbVertices());
    for(auto i=0; i < 1000 ; ++i)
      LC += i*boxCalculusCached.globalLaplaceBeltrami();
    auto tpsC = trace.endBlock();
    REQUIRE(tpsC < tps);
    REQUIRE(L.norm() == Approx(LC.norm()));

  }

}

TEST_CASE( "Testing PolygonalCalculus and DirichletConditions" )
{
  typedef Shortcuts< KSpace >                SH3;
  typedef SurfaceMesh< RealPoint,RealPoint > Mesh;
  typedef Mesh::Index                        Index;
  typedef PolygonalCalculus< RealPoint,RealVector > PolyDEC;
  typedef DirichletConditions< EigenLinearAlgebraBackend > DC;

  // Build a more complex surface.
  auto params = SH3::defaultParameters();

  params( "polynomial", "0.1*y*y -0.1*x*x - 2.0*z" )( "gridstep", 2.0 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);

  std::vector<std::vector< Index > > faces;
  std::vector<RealPoint> positions = primalSurface->positions();
  for( PolygonalCalculus< RealPoint,RealVector >::MySurfaceMesh::Index face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));

  Mesh surfmesh = Mesh( positions.begin(), positions.end(),
                            faces.begin(),     faces.end() );
  auto boundaryEdges = surfmesh.computeManifoldBoundaryEdges();

  // Builds calculus and solve a Poisson problem with Dirichlet boundary conditions
  PolyDEC calculus( surfmesh );
  // Laplace operator
  PolyDEC::SparseMatrix L = calculus.globalLaplaceBeltrami();
  // value on boundary
  PolyDEC::Vector g = calculus.form0();
  // characteristic set of boundary
  DC::IntegerVector b = DC::IntegerVector::Zero( g.rows() );

  SECTION("Solve Poisson problem with boundary Dirichlet conditions")
    {
      for ( double scale = 0.1; scale < 2.0; scale *= 2.0 )
        {
          std::cout << "scale=" << scale << std::endl;
          auto phi = [&]( Index v)
          {
            return cos(scale*(surfmesh.position(v)[0]))
              * (scale*surfmesh.position(v)[1]);
          };

          for(auto &e: boundaryEdges)
            {
              auto adjVertices = surfmesh.edgeVertices(e);
              auto v1 = adjVertices.first;
              auto v2 = adjVertices.second;
              g(v1) = phi(v1);
              g(v2) = phi(v2);
              b(v1) = 1;
              b(v2) = 1;
            }
          // Solve Δu=0 with g as boundary conditions
          PolyDEC::Solver solver;
          PolyDEC::SparseMatrix L_dirichlet = DC::dirichletOperator( L, b );
          solver.compute( L_dirichlet );
          REQUIRE( solver.info() == Eigen::Success );
          PolyDEC::Vector g_dirichlet = DC::dirichletVector( L, g, b, g );
          PolyDEC::Vector x_dirichlet = solver.solve( g_dirichlet );
          REQUIRE( solver.info() == Eigen::Success );
          PolyDEC::Vector u = DC::dirichletSolution( x_dirichlet, b, g );
          double min_phi = 0.0;
          double max_phi = 0.0;
          double min_u   = 0.0;
          double max_u   = 0.0;
          double min_i_u = 0.0;
          double max_i_u = 0.0;
          for (  PolygonalCalculus< RealPoint,RealVector >::MySurfaceMesh::Index v = 0; v < surfmesh.nbVertices(); ++v )
            {
              min_phi = std::min( min_phi, phi( v ) );
              max_phi = std::max( max_phi, phi( v ) );
              min_u   = std::min( min_u  , u  ( v ) );
              max_u   = std::max( max_u  , u  ( v ) );
              if ( b( v ) == 0.0 )
                {
                  min_i_u = std::min( min_i_u, u  ( v ) );
                  max_i_u = std::max( max_i_u, u  ( v ) );
                }
            }
          REQUIRE( min_phi <= min_u );
          REQUIRE( max_phi >= max_u );
          REQUIRE( min_phi <  min_i_u );
          REQUIRE( max_phi >  max_i_u );
        } //   for ( double scale = 0.1; scale < 2.0; scale *= 2.0 )
    }
};

/** @ingroup Tests **/
