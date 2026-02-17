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
 * @file geometry/tools/exampleQuickHull3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/01/01
 *
 * An example file named exampleQuickHull3D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the convex hull of a list of 3D points by Quick Hull algorithm.

\verbatim
# Computes the convex hull of examples/samples/bunny.dat with precision 100
./examples/geometry/tools/exampleQuickHull3D
\endverbatim
outputs
\verbatim
Read 13230 3D points.
Domain l=[PointVector] {-95, -135, -68} u=[PointVector] {6114, 4743, 6186}
[BoundedRationalPolytope<3> A.rows=2104 valid_edge_constraints=0 denom=100]
  [ 100 0 0 ] . x <= 6114
  [ -100 0 0 ] . x <= 95
  [ 0 100 0 ] . x <= 4743
  [ 0 -100 0 ] . x <= 135
  [ 0 0 100 ] . x <= 6186
  [ 0 0 -100 ] . x <= 68
  [ -92731200 -234892700 -352348000 ] . x <= 264145202
  [ -200106200 -500176600 -423177500 ] . x <= -1778463203
  [ 1155400 -144600 -10046900 ] . x <= 43312434
...
  [ 346800 558800 229200 ] . x <= 43326624
  [ 1457700 2361700 949200 ] . x <= 181949097

[SurfaceMesh (OK) #V=1051 #VN=0 #E=3147 #F=2098 #FN=0 E[IF]=3 E[IV]=5.98858 E[IFE]=2]
[PolygonalSurface #V=1051 #E=3147 #F=2098 Chi=2]
[ConvexCellComplex<3> #C=1 #F=2098 #V=1051 ]
\endverbatim

<table>
<tr>
<td>
\image html exampleQuickHull3D-bunny-input.jpg "Coarse model of Stanford bunny"
</td><td>
\image html exampleQuickHull3D-bunny-both.jpg "Its convex hull with a precision 100"
</td>
</tr>
</table>

@see \ref moduleQuickHull

\example geometry/tools/exampleQuickHull3D.cpp
*/

#include "DGtal/base/Common.h"
//! [QuickHull3D-Includes]
#include "DGtal/geometry/volumes/ConvexityHelper.h"
//! [QuickHull3D-Includes]
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "ConfigExamples.h"

double rand01() { return (double) rand() / (double) RAND_MAX; }

using namespace DGtal;
using namespace DGtal::Z3i;
int main( int argc, char* argv[] )
{
  int precision = argc > 1 ? atoi( argv[ 1 ] ) : 100;  // precision
  std::string inputFilename = argc > 2
    ? std::string( argv[ 2 ] )
    : examplesPath + "samples/bunny.dat" ;
  std::vector< RealPoint > points;
  std::ifstream finput( inputFilename.c_str() );
  std::string linestr;
  while ( std::getline( finput, linestr ) )
    {
      std::istringstream iss( linestr );
      double a, b, c;
      if ( ! (iss >> a >> b >> c) ) break;
      points.push_back( RealPoint( a, b, c ) );
    }
  trace.info() << "Read " << points.size() << " 3D points." << std::endl;

  // Build rational polytope
  typedef ConvexityHelper< 3 > Helper;
  const auto polytope
    = Helper::computeRationalPolytope( points, precision );
  trace.info() << polytope << std::endl;

  // Build the boundary of the convex hull as a surface mesh
  SurfaceMesh< RealPoint, RealVector > mesh;
  bool ok = Helper::computeConvexHullBoundary( mesh, points, precision );
  trace.info() << mesh << std::endl;
  std::ofstream out( "qhull-mesh.obj" );
  SurfaceMeshWriter< RealPoint, RealVector >::writeOBJ( out, mesh );
  out.close();

  // Build the boundary of the convex hull as a polygonal surface
  PolygonalSurface< RealPoint > polysurf;
  bool ok2 = Helper::computeConvexHullBoundary( polysurf, points, precision );
  trace.info() << polysurf << std::endl;

  // Build the convex hull as a convex cell complex.
  ConvexCellComplex< RealPoint > cvx_complex;
  bool ok3 = Helper::computeConvexHullCellComplex( cvx_complex, points, precision );
  trace.info() << cvx_complex << std::endl;

  return ( ok && ok2 && ok3 ) ? 0 : 1;
}
