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
 * @file geometry/volumes/fullConvexityShortestPaths3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/20
 *
 * An example file named fullConvexityShortestPaths3D
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to use tangency to compute shortest paths on 3D digital objects
   
   @see \ref dgtal_dconvexityapp_sec2
   
   For instance, you may call it on object "cube+sphere" as

\verbatim
fullConvexityShortestPaths3D cps.vol 0 255 0.0
\endverbatim

   The user selects two surfels (with shift + left click), and then
   shortest paths are computed and displayed.

<table>
<tr><td>
\image html cps-geodesics-1.jpg "Geodesic distances and geodesics on cube+sphere shape" width=90%
</td><td>
\image html cps-geodesics-2.jpg "Geodesic distances on cube+sphere shape" width=90%
</td><td>
\image html cps-shortest-path.jpg "Shortest path between two points" width=90%
</td></tr>
</table>

 \example geometry/volumes/fullConvexityShortestPaths3D.cpp
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
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/SimpleDistanceColorMap.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
//! [Tangency3D-includes]
#include "DGtal/geometry/volumes/TangencyComputer.h"
//! [Tangency3D-includes]
#include "ConfigExamples.h"

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

int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <h> <opt>" << std::endl;
  trace.info() << "\tComputes shortest paths to a source point on a sphere digitized with gridstep <h>." << std::endl;
  trace.info() << "\t- h [==1.0]: digitization gridstep" << std::endl;
  trace.info() << "\t- opt [==sqrt(3)]: >= sqrt(3): secure shortest paths, 0: fast" << std::endl;
  string inputFilename = examplesPath + "samples/Al.100.vol";
  double      h = argc > 1 ? atof( argv[ 1 ] ) : 1.0; //< exact (sqrt(3)) or inexact (0) computations
  double    opt = argc > 2 ? atof( argv[ 2 ] ) : sqrt(3.0); //< exact (sqrt(3)) or inexact (0) computations

  // Domain creation from two bounding points.
  trace.info() << "Building sphere1 shape ... ";
  auto   params  = SH3::defaultParameters();
  params( "polynomial", "sphere1" )( "gridstep",  h );
  params( "minAABB", -2)( "maxAABB", 2)( "offset", 1.0 )( "closed", 1 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K            = SH3::getKSpace( params );
  auto binary_image = SH3::makeBinaryImage(digitized_shape,
                                           SH3::Domain(K.lowerBound(),K.upperBound()),
                                           params );
  trace.info() << "  [Done]" << std::endl;
  
  trace.beginBlock( "Build mesh from primal/dual surface" );
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
  
  // (I) Extracts shortest paths to a target
  typedef TangencyComputer< KSpace >::Index Index;
  //! [Tangency3D-shortest-paths]
  trace.beginBlock( "Compute geodesics" );
  TangencyComputer< KSpace > TC( K );
  TC.init( lattice_points.cbegin(), lattice_points.cend() );
  auto SP = TC.makeShortestPaths( opt );
  SP.init( lowest ); //< set source
  double last_distance = 0.0;
  Index  last = 0;
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
  //! [Tangency3D-shortest-paths]
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

