
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/dec/NormalCorrectedFEM.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/MeshHelpers.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/math/linalg/DirichletConditions.h"
#include "DGtal/math/linalg/EigenSupport.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class InterpolatedCorrectedCalculus.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing PolygonalCalculus" )
{
  typedef SurfaceMesh< RealPoint,RealPoint > Mesh;
  typedef NormalCorrectedFEM<EigenLinearAlgebraBackend, RealPoint,RealVector > CFEM;
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

  box.computeFaceNormalsFromPositions();
  CFEM boxCalculus(box);
  SECTION("Local Laplace-Beltrami")
  {
    CFEM::Face f = 0;
    auto nf = box.incidentVertices(f).size();

    auto L = boxCalculus.localL0(f);
    CFEM::LinearAlgebraBackend::DenseVector phi(nf),expected(nf);
    phi << 1.0, 1.0, 1.0, 1.0;
    expected << 0,0,0,0;
    auto lphi = L*phi;
    for(int i = 0; i < nf; i++) {
        REQUIRE(abs(lphi(i)) < 1e-15);
    }
  }

  /*
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
  */
}

TEST_CASE( "Testing PolygonalCalculus and DirichletConditions" )
{
  typedef Shortcuts< KSpace >                SH3;
  typedef SurfaceMesh< RealPoint,RealPoint > Mesh;
  typedef Mesh::Index                        Index;
  typedef NormalCorrectedFEM<EigenLinearAlgebraBackend, RealPoint,RealVector > CFEM;
  typedef CFEM::LinearAlgebraBackend::DenseVector DenseVector;
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
  for( Index face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));

  Mesh surfmesh = Mesh( positions.begin(), positions.end(),
                            faces.begin(),     faces.end() );
  auto boundaryEdges = surfmesh.computeManifoldBoundaryEdges();
  surfmesh.computeFaceNormalsFromPositions();

  SECTION("Check surface")
    {
      REQUIRE( surfmesh.nbVertices() == 1364 );
      REQUIRE( surfmesh.nbFaces() == 1279 );
      REQUIRE( boundaryEdges.size() == 168 );
    }

  // Builds calculus and solve a Poisson problem with Dirichlet boundary conditions
  CFEM calculus( surfmesh );
  // Laplace opeartor
  CFEM::LinearOperator L = calculus.L0();
  // value on boundary
  CFEM::LinearAlgebraBackend::DenseVector g = CFEM::LinearAlgebraBackend::DenseVector::Zero(surfmesh.nbVertices());
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
          EigenLinearAlgebraBackend::SolverSimplicialLDLT solver;
          CFEM::LinearOperator L_dirichlet = DC::dirichletOperator( L, b );
          solver.compute( L_dirichlet );
          REQUIRE( solver.info() == Eigen::Success );
          DenseVector g_dirichlet = DC::dirichletVector( L, g, b, g );
          DenseVector x_dirichlet = solver.solve( g_dirichlet );
          REQUIRE( solver.info() == Eigen::Success );
          DenseVector u = DC::dirichletSolution( x_dirichlet, b, g );
          double min_phi = 0.0;
          double max_phi = 0.0;
          double min_u   = 0.0;
          double max_u   = 0.0;
          double min_i_u = 0.0;
          double max_i_u = 0.0;
          for (  Index v = 0; v < surfmesh.nbVertices(); ++v )
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
