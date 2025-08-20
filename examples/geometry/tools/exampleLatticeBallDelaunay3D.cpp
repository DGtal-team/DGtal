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
 * An example file named exampleLatticeBallDelaunay3D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the Delaunay complex of a set of lattice points in 3D by Quick Hull algorithm.

\verbatim
./examples/geometry/tools/exampleLatticeBallDelaunay3D
\endverbatim
outputs
\verbatim
Compute convex hull in higher dimension
assign ridges/faces to cell and conversely
takes care of vertex positions
[ConvexCellComplex<3> #C=466 #F=976 #V=100 hasFaceGeometry ]
\endverbatim

\image html qhull-delaunay3d-ball-10.jpg "Delaunay cell decomposition of randomly chosen points in a 3D lattice ball with radius 10"


@see \ref moduleQuickHull

\example geometry/tools/exampleLatticeBallDelaunay3D.cpp
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
  int     nb = argc > 1 ? atoi( argv[ 1 ] ) : 100; // nb points
  double dR  = argc > 2 ? atof( argv[ 2 ] ) : 10.;  // radius of balla
  double eps = argc > 3 ? atof( argv[ 3 ] ) : 0.1; // retraction
  typedef ConvexCellComplex< Point >::Index Index;
  ConvexCellComplex< Point > dcomplex;
  // (1) create range of random points in ball
  std::vector< Point > V;
  const auto R2 = dR * dR;
  const int  R  = ceil( dR );
  for ( int i = 0; i < nb; ) {
    Point p( rand() % (2*R+1) - R, rand() % (2*R+1) - R, rand() % (2*R+1) - R );
    if ( p.squaredNorm() <= R2 ) { V.push_back( p ); i++; }
  }
  // (2) compute convex hull
  bool ok =
    ConvexityHelper< 3 >::computeDelaunayCellComplex( dcomplex, V, true );
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
  for ( size_t c = 0; c < dcomplex.nbCells(); ++c )
    {
      RealPoint b = dcomplex.cellBarycenter( c );
      auto c_vtcs = dcomplex.cellVertices( c );
      std::map< Index, Index > v2v;
      for ( auto v : c_vtcs ) {
        RealPoint x = dcomplex.toReal( dcomplex.position( v ) );
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
  
