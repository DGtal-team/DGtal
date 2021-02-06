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
\endverbatim

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

  const auto polytope
    = ConvexityHelper< 3 >::computeRationalPolytope( points, precision );
  trace.info() << polytope << std::endl;
    // "Polytope has " << polytope.nbHalfSpaces() << " linear constraints."
  SurfaceMesh< RealPoint, RealVector > mesh;
  bool ok = ConvexityHelper< 3 >::computeConvexHullBoundary( mesh, points, precision );
  trace.info() << mesh << std::endl;
  
  std::ofstream out( "qhull.obj" );
  SurfaceMeshWriter< RealPoint, RealVector >::writeOBJ( out, mesh );
  out.close();
  
  return 0;
} 
  
