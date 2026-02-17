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
 * @file geometry/tools/exampleRationalBallQuickHull3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/01/01
 *
 * An example file named exampleRationalBallQuickHull3D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the convex hull of a set of real points in 3D by Quick Hull algorithm.

\verbatim
# 1000000 points in digital ball of radius 1000 with precision 1024
./examples/geometry/tools/exampleRationalBallQuickHull3D 1000000 1000 1024
\endverbatim
outputs
\verbatim
#points=1000000 #vertices=4490 #facets=8975
purge duplicates= 254 ms.
init simplex    = 29 ms.
quickhull core  = 326 ms.
compute vertices= 24 ms.
total time      = 634 ms.
\endverbatim

@see \ref moduleQuickHull


\example geometry/tools/exampleRationalBallQuickHull3D.cpp
*/

#include "DGtal/base/Common.h"
//! [QuickHull3D-Includes]
#include "DGtal/geometry/tools/QuickHull.h"
//! [QuickHull3D-Includes]
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"

double rand01() { return (double) rand() / (double) RAND_MAX; }

using namespace DGtal::Z3i;
int main( int argc, char* argv[] )
{
  int    nb = argc > 1 ? atoi( argv[ 1 ] ) : 100;  // nb points
  double  R = argc > 2 ? atof( argv[ 2 ] ) : 10.0; // radius of ball
  double precision = argc > 3 ? atof( argv[ 3 ] ) : 1024.0; // precision
  // (0) typedefs
  //! [QuickHull3D-Typedefs]
  typedef DGtal::ConvexHullRationalKernel< 3 > Kernel3D;
  typedef DGtal::QuickHull< Kernel3D >         QuickHull3D;
  //! [QuickHull3D-Typedefs]
  // (1) create range of random points in ball
  std::vector< RealPoint > V;
  const double R2 = R * R;
  for ( int i = 0; i < nb; ) {
    RealPoint p( rand01()*2.0*R - R, rand01()*2.0*R - R, rand01()*2.0*R - R );
    if ( p.squaredNorm() < R2 ) { V.push_back( p ); i++; }
  }
  // (2) compute convex hull
  //! [QuickHull3D-Computation]
  Kernel3D kernel( precision );
  QuickHull3D hull( kernel );
  hull.setInput( V );
  hull.computeConvexHull();
  std::cout << "#points="    << hull.nbPoints()
            << " #vertices=" << hull.nbVertices()
            << " #facets="   << hull.nbFacets() << std::endl;
  //! [QuickHull3D-Computation]
  //! [QuickHull3D-Timings]
  double total_time = 0;
  std::for_each( hull.timings.cbegin(), hull.timings.cend(),
                 [&total_time] ( double t ) { total_time += t; } );
  std::cout << "purge duplicates= " << round(hull.timings[ 0 ]) << " ms." << std::endl;
  std::cout << "init simplex    = " << round(hull.timings[ 1 ]) << " ms." << std::endl;
  std::cout << "quickhull core  = " << round(hull.timings[ 2 ]) << " ms." << std::endl;
  std::cout << "compute vertices= " << round(hull.timings[ 3 ]) << " ms." << std::endl;
  std::cout << "total time      = " << round(total_time) << " ms." << std::endl;
  //! [QuickHull3D-Timings]
  // (3) build mesh
  //! [QuickHull3D-BuildMesh]
  std::vector< RealPoint > positions;
  hull.getVertexPositions( positions );
  std::vector< std::vector< std::size_t > > facets;
  hull.getFacetVertices( facets );
  typedef DGtal::SurfaceMesh< RealPoint, RealVector> SMesh;
  SMesh mesh( positions.cbegin(), positions.cend(), facets.cbegin(), facets.cend() );
  //! [QuickHull3D-BuildMesh]
  // (4) output result as OBJ file
  //! [QuickHull3D-OutputMesh]
  std::ofstream out( "qhull.obj" );
  DGtal::SurfaceMeshWriter< RealPoint, RealVector >::writeOBJ( out, mesh );
  out.close();
  //! [QuickHull3D-OutputMesh]
  return 0;
}
