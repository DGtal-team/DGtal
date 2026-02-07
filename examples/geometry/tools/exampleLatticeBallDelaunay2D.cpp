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
 * @file geometry/tools/exampleLatticeBallDelaunay2D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/01/01
 *
 * An example file named exampleLatticeBallDelaunay2D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the Delaunay complex of a set of lattice points in 2D by Quick Hull algorithm.

\verbatim
# 1000000 points in digital ball of radius 1000
./examples/geometry/tools/exampleLatticeBallDelaunay2D 1000000 1000
\endverbatim
outputs
\verbatim
#points=999908 #vertices=4480 #facets=8947
purge duplicates= 281 ms.
init simplex    = 24 ms.
quickhull core  = 336 ms.
compute vertices= 24 ms.
total time      = 665 ms.
\endverbatim

@see \ref moduleQuickHull


\example geometry/tools/exampleLatticeBallDelaunay2D.cpp
*/

#include "DGtal/base/Common.h"
//! [Delaunay2D-Includes]
#include "DGtal/geometry/tools/QuickHull.h"
//! [Delaunay2D-Includes]
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"

using namespace DGtal::Z2i;
int main( int argc, char* argv[] )
{
  int    nb = argc > 1 ? atoi( argv[ 1 ] ) : 100;  // nb points
  double dR = argc > 2 ? atof( argv[ 2 ] ) : 10.0; // radius of ball
  // (0) typedefs
  //! [Delaunay2D-Typedefs]
  typedef DGtal::DelaunayIntegralKernel< 2 > Kernel2D;
  typedef DGtal::QuickHull< Kernel2D >       Delaunay2D;
  //! [Delaunay2D-Typedefs]
  // (1) create range of random points in ball
  std::vector< Point > V;
  const double R2 = dR * dR;
  const int    R  = (int) ceil( dR );
  for ( int i = 0; i < nb; ) {
    Point p( rand() % (2*R+1) - R, rand() % (2*R+1) - R, rand() % (2*R+1) - R );
    if ( p.squaredNorm() < R2 ) { V.push_back( p ); i++; }
  }
  // (2) compute convex hull
  //! [Delaunay2D-Computation]
  Delaunay2D hull;
  hull.setInput( V );
  hull.computeConvexHull();
  std::cout << "#points="    << hull.nbPoints()
            << " #vertices=" << hull.nbVertices()
            << " #facets="   << hull.nbFacets() << std::endl;
  //! [Delaunay2D-Computation]
  //! [Delaunay2D-Timings]
  double total_time = 0;
  std::for_each( hull.timings.cbegin(), hull.timings.cend(),
                 [&total_time] ( double t ) { total_time += t; } );
  std::cout << "purge duplicates= " << round(hull.timings[ 0 ]) << " ms." << std::endl;
  std::cout << "init simplex    = " << round(hull.timings[ 1 ]) << " ms." << std::endl;
  std::cout << "quickhull core  = " << round(hull.timings[ 2 ]) << " ms." << std::endl;
  std::cout << "compute vertices= " << round(hull.timings[ 3 ]) << " ms." << std::endl;
  std::cout << "total time      = " << round(total_time) << " ms." << std::endl;
  //! [Delaunay2D-Timings]
  // (3) build mesh
  //! [Delaunay2D-BuildMesh]
  std::vector< RealPoint > positions;
  hull.getVertexPositions( positions );
  std::vector< std::vector< std::size_t > > facets;
  hull.getFacetVertices( facets );
  // Finite facets precede infinite facets => keep only finite facets
  facets.resize( hull.nbFiniteFacets() );
  // To visualize the result, we build a surface mesh in R3 lying in
  // the plane z=0 and composed of the Delaunay cells.
  typedef DGtal::SpaceND< 3, int > Z3;
  std::vector< Z3::RealPoint > positions3d;
  for ( auto p : positions )
    positions3d.push_back( Z3::RealPoint( p[ 0 ], p[ 1 ], 0.0  ) );
  typedef DGtal::SurfaceMesh< Z3::RealPoint, Z3::RealVector> SMesh;
  SMesh mesh( positions3d.cbegin(), positions3d.cend(),
              facets.cbegin(), facets.cend() );
  //! [Delaunay2D-BuildMesh]
  // (4) output result as OBJ file
  //! [Delaunay2D-OutputMesh]
  std::ofstream out( "delaunay.obj" );
  DGtal::SurfaceMeshWriter< Z3::RealPoint, Z3::RealVector >::writeOBJ( out, mesh );
  out.close();
  //! [Delaunay2D-OutputMesh]
  return 0;
}
