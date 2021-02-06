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
  for ( auto c = 0; c < dcomplex.nbCells(); ++c )
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
  
