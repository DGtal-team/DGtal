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
 * @file geometry/volumes/fullConvexitySphereGeodesics.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/05/09
 *
 * An example file named fullConvexitySphereGeodesics
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to use tangency to compute exact or
   approximate geodesic shortest paths on 3D digital objects, here a
   unit sphere digitized at your chosen gridstep.
   
   @see \ref dgtal_dconvexityapp_sec25
   
   For instance, you may call it with a digitization gridstep 0.0625
   and parameter 1.8 (greater than sqrt(3), so guarantees exact
   shortest paths).

\verbatim
./examples/geometry/volumes/fullConvexitySphereGeodesics 0.0625 1.8
\endverbatim

and outputs

\verbatim
Usage: ./examples/geometry/volumes/fullConvexitySphereGeodesics <h> <opt>
	Computes shortest paths to a source point on a sphere digitized with gridstep <h>.
	- h [==1.0]: digitization gridstep
	- opt [==sqrt(3)]: >= sqrt(3): secure shortest paths, 0: fast
New Block [Building sphere1 shape ... ]
EndBlock [Building sphere1 shape ... ] (13.2603 ms)
New Block [Build mesh from primal surface]
  #surfels =4782
  #pointels=4784
  [SurfaceMesh (OK) #V=4784 #VN=0 #E=9564 #F=4782 #FN=0 E[IF]=4 E[IV]=3.99833 E[IFE]=2]
EndBlock [Build mesh from primal surface] (111.024 ms)
New Block [Compute geodesics]
EndBlock [Compute geodesics] (11958.8 ms)
Max distance is 3.02389
Comput. time is 11958.8
Last index is   3698
Uppest index is 3698
New Block [Output Corrected HD OBJ files]
  Max distance is 3.02389
EndBlock [Output Corrected HD OBJ files] (494.499 ms)
\endverbatim

It computes all shortest paths to the lowest point of the digitized
sphere, outputs the maximal distance (so less than pi), and checks
that the furthest point is indeed antipodal to the source point. Two
OBJ files named "sphere1-geodesics.obj "and
"sphere1-geodesics-iso.obj" are also outputed, and you may use them to
render the result like below.

<table>
<tr><td>
\image html sphere1-h0_0625-geodesics.jpg "Exact geodesic distances on unit sphere digitized at h=0.0625 (max d=3.02389)" width=90%
</td><td>
\image html sphere1-h0_01-geodesics.jpg "Approximate geodesic distances on unit sphere digitized at h=0.01 (max d=3.1269)" width=90%
</td></tr>
</table>

\example geometry/volumes/fullConvexitySphereGeodesics.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/geometry/volumes/TangencyComputer.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
typedef Z3i::Space          Space;
typedef Z3i::KSpace         KSpace;
typedef Z3i::Domain         Domain;
typedef Z3i::SCell          SCell;
typedef Shortcuts< KSpace > SH3;
typedef Space::Point        Point;
typedef Space::RealPoint    RealPoint;
typedef Space::RealVector   RealVector;
typedef Space::Vector       Vector;
typedef SurfaceMesh< RealPoint, RealVector > SMesh;
typedef SurfaceMeshWriter< RealPoint, RealVector > SMeshWriter;
typedef SMesh::Index        Index;
typedef SMesh::Vertices     Vertices;

void saveToObj( const std::string& output,
                const SMesh& surfmesh,
                const std::vector< double >& vvalues,
                int nb_isolines_per_unit = 10,
                const double thickness = 0.1 )
{
  std::string cobjname = output;
  std::string cisoname = output + "-iso";
  auto quantify = [] ( double v, double m, double nb )
    { return round( v/m*nb )*m/nb; };
  trace.beginBlock( "Output Corrected HD OBJ files" );
  const auto fvalues = surfmesh.computeFaceValuesFromVertexValues( vvalues );
  double maxValue = * ( std::max_element( vvalues.cbegin(), vvalues.cend() ) );
  double minValue = * ( std::min_element( vvalues.cbegin(), vvalues.cend() ) );
  double maxDist  = maxValue - minValue;
  trace.info() << "Max distance is " << maxDist << std::endl;
  auto cmap = SH3::getColorMap( 0.0, maxDist );
  std::vector< Color > fcolors( surfmesh.nbFaces() );
  for ( Index f = 0; f < fvalues.size(); ++f )
    fcolors[ f ] = cmap( quantify( fvalues[ f ] - minValue, maxDist, 50 ) );
  SMeshWriter::writeOBJ( cobjname, surfmesh, fcolors );
  double unit = pow( 10.0, floor( log( maxDist ) / log( 10.0 ) ) - 1.0 );
  const int N = 10 * nb_isolines_per_unit;
  std::vector< double > isolines( N );
  std::vector< Color >  isocolors( N );
  for ( int i = 0; i < N; i++ )
    {
      isolines [ i ] = (double) i * 10.0 * unit / (double) nb_isolines_per_unit
        + minValue;
      isocolors[ i ] = ( i % nb_isolines_per_unit == 0 )
        ? Color::Red : Color::Black;
    }
  SMeshWriter::writeIsoLinesOBJ( cisoname, surfmesh, fvalues, vvalues,
                                 isolines, thickness, isocolors );
  trace.endBlock();
}


int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <h> <opt>" << std::endl;
  trace.info() << "\tComputes shortest paths to a source point on a sphere digitized with gridstep <h>." << std::endl;
  trace.info() << "\t- h [==1.0]: digitization gridstep" << std::endl;
  trace.info() << "\t- opt [==sqrt(3)]: >= sqrt(3): secure shortest paths, 0: fast" << std::endl;
  double      h = argc > 1 ? atof( argv[ 1 ] ) : 0.0625; //< exact (sqrt(3)) or inexact (0) computations
  double    opt = argc > 2 ? atof( argv[ 2 ] ) : sqrt(3.0); //< exact (sqrt(3)) or inexact (0) computations

  // Domain creation from two bounding points.
  trace.beginBlock( "Building sphere1 shape ... " );
  auto   params  = SH3::defaultParameters();
  params( "polynomial", "sphere1" )( "gridstep",  h );
  params( "minAABB", -2)( "maxAABB", 2)( "offset", 1.0 )( "closed", 1 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K            = SH3::getKSpace( params );
  auto binary_image = SH3::makeBinaryImage(digitized_shape,
                                           SH3::Domain(K.lowerBound(),K.upperBound()),
                                           params );
  trace.endBlock();
  
  trace.beginBlock( "Build mesh from primal surface" );
  // Compute surface
  auto surface = SH3::makeDigitalSurface( binary_image, K, params );
  // Build a mesh
  SMesh smesh;
  auto embedder = SH3::getCellEmbedder( K );
  std::vector< Point >    lattice_points;
  SH3::RealPoints         vertices;
  std::vector< Vertices > faces;
  SH3::Cell2Index         c2i;
  auto pointels = SH3::getPointelRange( c2i, surface );
  for ( auto p : pointels ) lattice_points.push_back( K.uCoords( p ) );
  trace.info() << "#surfels =" << surface->size() << std::endl;
  trace.info() << "#pointels=" << pointels.size() << std::endl;
  vertices = SH3::RealPoints( pointels.size() );
  std::transform( pointels.cbegin(), pointels.cend(), vertices.begin(),
                  [&] (const SH3::Cell& c) { return h * embedder( c ); } ); 
  // Build faces
  for ( auto&& surfel : *surface )
    {
      const auto primal_surfel_vtcs = SH3::getPointelRange( K, surfel );
      std::vector< Index > face;	      
      for ( auto&& primal_vtx : primal_surfel_vtcs )
        face.push_back( c2i[ primal_vtx ] );
      faces.push_back( face );
    }
  smesh.init( vertices.cbegin(), vertices.cend(),
	      faces.cbegin(),    faces.cend() );
  trace.info() << smesh << std::endl;
  trace.endBlock();

  // Find lowest and uppest point.
  const Index nb = lattice_points.size();
  Index   lowest = 0;
  Index   uppest = 0;
  for ( Index i = 1; i < nb; i++ )
    {
      if ( lattice_points[ i ] < lattice_points[ lowest ] ) lowest = i;
      if ( lattice_points[ uppest ] < lattice_points[ i ] ) uppest = i;
    }
  
  // Extracts shortest paths to a target
  typedef TangencyComputer< KSpace >::Index tcIndex;
  trace.beginBlock( "Compute geodesics" );
  TangencyComputer< KSpace > TC( K );
  TC.init( lattice_points.cbegin(), lattice_points.cend() );
  auto SP = TC.makeShortestPaths( opt );
  SP.init( lowest ); //< set source
  double last_distance = 0.0;
  tcIndex  last = 0;
  while ( ! SP.finished() )
    {
      last = std::get<0>( SP.current() );
      last_distance = std::get<2>( SP.current() );
      SP.expand();
    }
  double time = trace.endBlock();
  std::cout << "Max distance is " << last_distance*h << std::endl;
  std::cout << "Comput. time is " << time << std::endl;
  std::cout << "Last index is   " << last << std::endl;
  std::cout << "Uppest index is " << uppest << std::endl;

  // Export surface for display
  std::vector<double> distances = SP.distances();
  for ( tcIndex i = 0; i < distances.size(); i++ )
    distances[ i ] *= h;
  saveToObj( "sphere1-geodesics", smesh, distances, 10, 0.1 );
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

