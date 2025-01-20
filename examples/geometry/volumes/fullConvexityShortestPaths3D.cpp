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
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/SimpleDistanceColorMap.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
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
typedef Space::Vector       Vector;

// Called when an user clicks on a surfel.
int reaction( void* viewer, DGtal::int32_t name, void* data )
{
  ((void) viewer);

  DGtal::int32_t* selected = (DGtal::int32_t*) data;
  *selected = name;
  std::cout << "Selected surfel=" << *selected << std::endl;
  return 0;
}

int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <input.vol> <m> <M> <opt>" << std::endl;
  trace.info() << "\tComputes shortest paths to a source point" << std::endl;
  trace.info() << "\t- input.vol: choose your favorite shape" << std::endl;
  trace.info() << "\t- m [==0], M [==255]: used to threshold input vol image" << std::endl;
  trace.info() << "\t- opt >= sqrt(3): secure shortest paths, 0: fast" << std::endl;
  string inputFilename = examplesPath + "samples/Al.100.vol";
  std::string fn= argc > 1 ? argv[ 1 ]         : inputFilename; //< vol filename
  int         m = argc > 2 ? atoi( argv[ 2 ] ) : 0;   //< low for thresholding
  int         M = argc > 3 ? atoi( argv[ 3 ] ) : 255; //< up for thresholding
  double    opt = argc > 4 ? atof( argv[ 4 ] ) : sqrt(3.0); //< exact (sqrt(3)) or inexact (0) computations

  QApplication application(argc,argv);
  Viewer3D<> viewer;
  viewer.setWindowTitle("fullConvexityShortestPaths3D");
  viewer.show();  

  // Set up shortcuts parameters.
  auto   params  = SH3::defaultParameters();
  params( "thresholdMin", m )( "thresholdMax", M );
  params( "surfaceComponents" , "All" );
  
  // Domain creation from two bounding points.
  trace.info() << "Building set or importing vol ... ";
  KSpace K;
  auto bimage = SH3::makeBinaryImage( fn, params );
  K = SH3::getKSpace( bimage );
  trace.info() << "  [Done]" << std::endl;
  
  // Compute surface
  auto surface = SH3::makeDigitalSurface( bimage, K, params );

  // Compute interior boundary points
  // They are less immediate interior points than surfels.
  std::vector< Point >   points;
  std::map< SCell, int > surfel2idx;
  std::map< Point, int > point2idx;
  int idx = 0;
  for ( auto s : (*surface) )
    {
      // get inside point on the border of the shape.
      Dimension k = K.sOrthDir( s );
      auto voxel  = K.sIncident( s, k, K.sDirect( s, k ) );
      Point p     = K.sCoords( voxel );
      auto it     = point2idx.find( p );
      if ( it == point2idx.end() )
        {
          points.push_back( p );
          surfel2idx[ s ] = idx;
          point2idx [ p ] = idx++;
        }
      else
        surfel2idx[ s ] = it->second;
    } 
  trace.info() << "Shape has " << points.size() << " interior boundary points"
               << std::endl;

  // Select a starting point.
  typedef Viewer3D<> MViewer3D;
  DGtal::int32_t selected_surfels[ 2 ] = { 0, 0 };
  auto surfels = SH3::getSurfelRange ( surface );
  for ( int i = 0;  i < 2; i++ )
    {
      MViewer3D viewerCore( K );
      viewerCore.show();
      Color colSurfel( 200, 200, 255, 255 );
      Color colStart( 255, 0, 0, 255 );
      DGtal::int32_t name = 0;
      viewerCore << SetMode3D( surfels[ 0 ].className(), "Basic");
      viewerCore.setFillColor( colSurfel );
      for ( auto && s : surfels ) viewerCore << SetName3D( name++ ) << s;
      viewerCore << SetSelectCallback3D( reaction, &selected_surfels[ i ],
                                         0, surfels.size() - 1 );
      viewerCore << MViewer3D::updateDisplay;
      application.exec();
    }
  
  // Get selected surfel/point
  const auto s0 = surfels[ selected_surfels[ 0 ] ];
  Dimension  k0 = K.sOrthDir( s0 );
  auto   voxel0 = K.sIncident( s0, k0, K.sDirect( s0, k0 ) );
  Point      p0 = K.sCoords( voxel0 );
  auto   start0 = point2idx[ p0 ];
  std::cout << "Start0 index is " << start0 << std::endl;
  const auto s1 = surfels[ selected_surfels[ 1 ] ];
  Dimension  k1 = K.sOrthDir( s1 );
  auto   voxel1 = K.sIncident( s1, k1, K.sDirect( s1, k1 ) );
  Point      p1 = K.sCoords( voxel1 );
  auto   start1 = point2idx[ p1 ];
  std::cout << "Start1 index is " << start1 << std::endl;

  // (I) Extracts shortest paths to a target

  typedef TangencyComputer< KSpace >::Index Index;
  //! [Tangency3D-shortest-paths]
  TangencyComputer< KSpace > TC( K );
  TC.init( points.cbegin(), points.cend() );
  auto SP = TC.makeShortestPaths( opt );
  SP.init( start0 ); //< set source
  double last_distance = 0.0;
  while ( ! SP.finished() )
    {
      last_distance = std::get<2>( SP.current() );
      SP.expand();
    }
  std::cout << "Max distance is " << last_distance << std::endl;
  //! [Tangency3D-shortest-paths]

  {
    const int nb_repetitions = 10;
    const double      period = last_distance / nb_repetitions;
    SimpleDistanceColorMap< double > cmap( 0.0, period, false );
    MViewer3D viewerCore;
    viewerCore.show();
    Color colSurfel( 200, 200, 255, 128 );
    Color colStart( 255, 0, 0, 128 );

    viewerCore.setUseGLPointForBalls(true);
    for ( auto i = 0; i < points.size(); ++i )
      {
        const double d_s = SP.distance( i );
        Color c_s        = cmap( fmod( d_s, period ) );
        viewerCore.setFillColor( c_s );
        viewerCore.addBall( RealPoint( points[ i ][ 0 ],
                                       points[ i ][ 1 ],
                                       points[ i ][ 2 ] ), 12.0 );
      }

    // JOL: Left if you wish to display it as a surface, and to display paths.
    
    // auto surfels = SH3::getSurfelRange ( surface );
    // viewerCore << SetMode3D( surfels[ 0 ].className(), "Basic");
    // for ( auto && s : surfels )
    //   {
    //     const double d_s = SP.distance( surfel2idx[ s ] );
    //     Color c_s        = cmap( fmod( d_s, period ) );
    //     viewerCore.setFillColor( c_s );
    //     viewerCore << s;
    //   }
    // viewerCore.setFillColor( colStart );
    // viewerCore.setLineColor( Color::Green );
    // viewerCore << s;
    // for ( Index i = 0; i < SP.size(); i++ ) {
    //   Point p1 = SP.point( i );
    //   Point p2 = SP.point( SP.ancestor( i ) );
    //   viewerCore.addLine( p1, p2, 1.0 );
    // }
    viewerCore << MViewer3D::updateDisplay;
    application.exec();
  }

  // (II) Extracts a shortest path between two points.

  //! [Tangency3D-shortest-path]
  auto SP0 = TC.makeShortestPaths( opt );
  auto SP1 = TC.makeShortestPaths( opt );
  SP0.init( start0 );     //< source point
  SP1.init( start1 );     //< target point
  std::vector< Index > Q; //< the output shortest path
  while ( ! SP0.finished() && ! SP1.finished() )
    {
      auto n0_ = SP0.current(); ((void) n0_);
      auto n1_ = SP1.current();
      // auto p0_ = std::get<0>( n0_ );
      auto p1_ = std::get<0>( n1_ );
      SP0.expand();
      SP1.expand();
      if ( SP0.isVisited( p1_ ) )
        {
          auto c0 = SP0.pathToSource( p1_ );
          auto c1 = SP1.pathToSource( p1_ );
          std::copy(c0.rbegin(), c0.rend(), std::back_inserter(Q));
          Q.pop_back();
          std::copy(c1.begin(), c1.end(), std::back_inserter(Q)); 
          break;
        }
    }
  // Q is empty if there is no path.
  //! [Tangency3D-shortest-path]

  // Display computed distances and shortest path
  {
    const int nb_repetitions = 10;
    const double      period = last_distance / nb_repetitions;
    SimpleDistanceColorMap< double > cmap( 0.0, period, false );
    MViewer3D viewerCore;
    viewerCore.show();
    Color colSurfel( 200, 200, 255, 128 );
    Color colStart( 255, 0, 0, 128 );
    viewerCore.setUseGLPointForBalls(true);
    for ( auto i = 0; i < points.size(); ++i )
      {
        const double d_s0 = SP0.isVisited( i ) ? SP0.distance( i ) : SP0.infinity();
        const double d_s1 = SP1.isVisited( i ) ? SP1.distance( i ) : SP1.infinity();
        const double d_s  = std::min( d_s0, d_s1 );
        Color c_s        = ( d_s != SP0.infinity() )
          ? cmap( fmod( d_s, period ) )
          : Color::Black;
        viewerCore.setFillColor( c_s );
        viewerCore.addBall( RealPoint( points[ i ][ 0 ],
                                       points[ i ][ 1 ],
                                       points[ i ][ 2 ] ), 12 );
      }
    viewerCore.setLineColor( Color::Green );
    for ( auto i = 1; i < Q.size(); i++ )
      {
        Point p1_ = TC.point( Q[ i-1 ] );
        Point p2_ = TC.point( Q[ i   ] );
        viewerCore.addLine( p1_, p2_, 18.0 );
      }
    viewerCore << MViewer3D::updateDisplay;
    application.exec();
  }

  // (III) Extracts multiple shortest paths between many sources and two targets.

  std::vector< Index > sources;
  std::vector< Index > dests;
  for ( int i = 0; i < 20; i++ )
    sources.push_back( rand() % TC.size() );
  dests.push_back( start0 );
  dests.push_back( start1 );
  auto paths = TC.shortestPaths( sources, dests, opt );

  // Display them.
  {
    MViewer3D viewerCore;
    viewerCore.show();
    Color colSurfel( 200, 200, 255, 128 );
    Color colStart( 255, 0, 0, 128 );
    viewerCore.setUseGLPointForBalls(true);
    for ( auto i = 0; i < points.size(); ++i )
      {
        viewerCore.setFillColor( Color( 150, 150, 150, 255 ) );
        viewerCore.addBall( RealPoint( points[ i ][ 0 ],
                                       points[ i ][ 1 ],
                                       points[ i ][ 2 ] ), 12 );
      }
    viewerCore.setLineColor( Color::Green );
    for ( auto path : paths )
      {
        for ( auto i = 1; i < path.size(); i++ )
          {
            Point p1_ = TC.point( path[ i-1 ] );
            Point p2_ = TC.point( path[ i   ] );
            viewerCore.addLine( p1_, p2_, 18.0 );
          }
        trace.info() << "length=" << TC.length( path ) << std::endl;
      }
    viewerCore << MViewer3D::updateDisplay;
    application.exec();
  }
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

