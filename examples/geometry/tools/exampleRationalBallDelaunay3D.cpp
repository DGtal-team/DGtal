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
 * @file geometry/tools/exampleLatticeBallDelaunay3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/02/06
 *
 * An example file named exampleRationalBallDelaunay3D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the Delaunay complex of a set of rational points in 3D by Quick Hull algorithm.

\verbatim
./examples/geometry/tools/exampleRationalBallDelaunay3D 100 10 0.25 100
\endverbatim
outputs
\verbatim
Compute convex hull in higher dimension
assign ridges/faces to cell and conversely
takes care of vertex positions
[ConvexCellComplex<3> #C=456 #F=955 #V=99 hasFaceGeometry ]
\endverbatim

\image html exampleRationalBallDelaunay3D-100.jpg "Delaunay cell decomposition of 100 randomly chosen points in a 3D ball with radius 10, with precision 100 and retraction 0.25"


@see \ref moduleQuickHull

\example geometry/tools/exampleRationalBallDelaunay3D.cpp
*/

#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/geometry/volumes/ConvexityHelper.h"

using namespace DGtal;
using namespace DGtal::Z3i;
int main( int argc, char* argv[] )
{
  int    nb        = argc > 1 ? atoi( argv[ 1 ] ) : 100; // nb points
  double dR        = argc > 2 ? atof( argv[ 2 ] ) : 10.; // radius of balla
  double eps       = argc > 3 ? atof( argv[ 3 ] ) : 0.1; // retraction
  double precision = argc > 4 ? atof( argv[ 4 ] ) : 100.0; // precision
  typedef ConvexCellComplex< RealPoint >::Index Index;
  ConvexCellComplex< RealPoint > dcomplex;
  // (1) create range of random points in ball
  std::vector< RealPoint > V;
  const auto R2 = dR * dR;
  for ( int i = 0; i < nb; ) {
    RealPoint p( ( rand() / (double) RAND_MAX * 2.0 - 1.0 ) * dR,
                 ( rand() / (double) RAND_MAX * 2.0 - 1.0 ) * dR,
                 ( rand() / (double) RAND_MAX * 2.0 - 1.0 ) * dR );
    if ( p.squaredNorm() <= R2 ) { V.push_back( p ); i++; }
  }
  // (2) compute convex hull
  bool ok =
    ConvexityHelper< 3 >::computeDelaunayCellComplex( dcomplex, V,
                                                      precision, true );
  if ( ! ok )
    {
      trace.error() << "Input set of points is not full dimensional." << std::endl;
      return 1;
    }
  dcomplex.requireFaceGeometry();
  std::cout << dcomplex << std::endl;
  // (3) build the mesh that is made of the exploded 3d cells
  std::vector< RealPoint > positions;
  std::vector< std::vector< Index > > facets;
  Index idxv = 0;
  for ( auto c = 0; c < dcomplex.nbCells(); ++c )
    {
      RealPoint b = dcomplex.cellBarycenter( c );
      auto c_vtcs = dcomplex.cellVertices( c );
      std::map< Index, Index > v2v;
      for ( auto v : c_vtcs ) {
        RealPoint x = dcomplex.position( v );
        v2v[ v ] = idxv++;
        positions.push_back( b + ( x - b ) * ( 1.0 - eps ) );
      }
      for ( const auto& f : dcomplex.cellFaces( c ) ) {
        auto f_vtcs = dcomplex.faceVertices( f );
        for ( auto& vertex : f_vtcs )
          vertex = v2v[ vertex ];
        facets.push_back( f_vtcs );
      }
    }
  typedef DGtal::SurfaceMesh< Z3::RealPoint, Z3::RealVector> SMesh;
  SMesh mesh( positions.cbegin(), positions.cend(),
              facets.cbegin(), facets.cend() );
  // (4) output result as OBJ file
  std::ofstream out( "delaunay3d.obj" );
  DGtal::SurfaceMeshWriter< Z3::RealPoint, Z3::RealVector >
    ::writeOBJ( out, mesh );
  out.close();
  return 0;
} 
  
