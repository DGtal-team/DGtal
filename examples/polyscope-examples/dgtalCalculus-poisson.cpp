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
 */
/**
 * @file
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/09/02
 *
 * This file is part of the DGtal library.
 */
#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>
#include <DGtal/dec/PolygonalCalculus.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>


#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PCalculus;
typedef PCalculus::SparseMatrix SparseMatrix;
typedef PCalculus::Vector       Form;
typedef PCalculus::Triplet      Triplet;
typedef std::size_t             Index;
//Polyscope global
polyscope::SurfaceMesh *psMesh;
SurfMesh surfmesh;
float scale = 0.1;

/// System of the form A x = b, where A is SDP, where you wish to
/// have Dirichlet boundary conditions u at some places p (`p=1`or `p=0`).
/// @pre `#row(A) = #col(A) = #col(u)`
/// @return the linear matrix A' to prefactor.
SparseMatrix dirichletOperator( const SparseMatrix& A,
                                  const Form& p )
{
  ASSERT( A.cols() == A.rows() );
  ASSERT( p.rows() == A.rows() );
  const auto n = p.rows();
  std::vector< Index > relabeling( n );
  Index j = 0;
  for ( Index i = 0; i < p.rows(); i++ )
    relabeling[ i ] = ( p[ i ] == 0.0 ) ? j++ : n;
  // Building matrix
  SparseMatrix Ap( j, j );
  std::vector< Triplet > triplets;
  for ( int k = 0; k < A.outerSize(); ++k )
    for ( typename SparseMatrix::InnerIterator it( A, k ); it; ++it )
      {
        if ( ( relabeling[ it.row() ] != n ) && ( relabeling[ it.col() ] != n ) )
          triplets.push_back( { relabeling[ it.row() ], relabeling[ it.col() ],
                it.value() } );
      }
  Ap.setFromTriplets( triplets.cbegin(), triplets.cend() );
  return Ap;
}

/// System of the form A x = b, where A is SDP, where you wish to
/// have Dirichlet boundary conditions u at some places p != 0.
/// @pre `#row(A) = #col(A) = #col(u)`
/// @return the form b' to solve for
Form dirichletVector( const SparseMatrix& A,
                      const Form& b,
                      const Form& p,
                      const Form& u ) 
{
  ASSERT( A.cols() == A.rows() );
  ASSERT( p.rows() == A.rows() );
  const auto n = p.rows();
  Form  up = p.array() * u.array();
  Form tmp = b.array() - (A * up).array();
  std::vector< Index > relabeling( n );
  Index j = 0;
  for ( Index i = 0; i < p.rows(); i++ )
    relabeling[ i ] = ( p[ i ] == 0.0 ) ? j++ : n;
  Form  bp( j );
  for ( Index i = 0; i < p.rows(); i++ )
    if ( p[ i ] == 0 ) bp[ relabeling[ i ] ] = tmp[ i ];
  return bp;
}

Form dirichletSolution( const Form& xp,
                        const Form& p,
                        const Form& u ) 
{
  Form  x = Form( p.rows() );
  Index j = 0;
  for ( Index i = 0; i < p.rows(); i++ )
    x[ i ] = ( p[ i ] == 0.0 ) ? xp[ j++ ] : u[ i ];
  return x;
}

void computeLaplace()
{
  //! [PolyDEC-init]
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector> calculus(surfmesh);
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::SparseMatrix L = calculus.globalLaplaceBeltrami();
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector g = PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector::Zero(surfmesh.nbVertices());
  Form b = g;
  
  //We set values on the boundary
  auto boundaryEdges = surfmesh.computeManifoldBoundaryEdges();
  std::cout<< "Number of boundary edges= "<<boundaryEdges.size()<<std::endl;
  
  auto pihVertex=[&](const SurfMesh::Vertex &v){return  cos(scale*(surfmesh.position(v)[0]))*(scale*surfmesh.position(v)[1]);};
  
  for(auto &e: boundaryEdges)
  {
    auto adjVertices = surfmesh.edgeVertices(e);
    g(adjVertices.first)  = pihVertex(adjVertices.first);
    g(adjVertices.second) = pihVertex(adjVertices.second);
    b(adjVertices.first)  = 1.0;
    b(adjVertices.second) = 1.0;
  }

  //Solve Δu=0 with g as boundary conditions
  PCalculus::Solver solver;
  SparseMatrix L_dirichlet = dirichletOperator( L, b );
  solver.compute( L_dirichlet );
  ASSERT(solver.info()==Eigen::Success);
  Form g_dirichlet = dirichletVector( L, g, b, g );
  Form x_dirichlet = solver.solve( g_dirichlet );
  Form u = dirichletSolution( x_dirichlet, b, g );

  // //Solve Δu=0 with g as boundary conditions
  // //(the operator constructon and its prefactorization could have been factorized)
  // PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Solver solver;
  // solver.compute(L);
  // ASSERT(solver.info()==Eigen::Success);
  
  // PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector u = solver.solve(g);
  // ASSERT(solver.info()==Eigen::Success);
  //! [PolyDEC-init]

  psMesh->addVertexScalarQuantity("g", g);
  psMesh->addVertexScalarQuantity("u", u);
}

void myCallback()
{
  ImGui::SliderFloat("Phi scale", &scale, 0., 10.);
  if(ImGui::Button("Compute Laplace problem"))
    computeLaplace();
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  
  auto h=.7   ; //gridstep
  params( "polynomial", "0.1*y*y -0.1*x*x - 2.0*z" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);
  
  //Need to convert the faces
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
  std::vector<RealPoint> positions = primalSurface->positions();
  for(auto face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));
  
  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  psMesh = polyscope::registerSurfaceMesh("digital surface", positions, faces);

  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
