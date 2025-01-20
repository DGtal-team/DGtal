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
 * @file geometry/volumes/fullConvexityAnalysis3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/20
 *
 * An example file named fullConvexityAnalysis3D
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to analyze the local geometry of 3D digital
   sets with full convexity over cubical neighborhoods.
   
   @see \ref dgtal_dconvexityapp_sec1
   
   For instance, you may call it to analyse image Al.100.vol at scale 2 as

\verbatim
fullConvexityAnalysis3D 2 ${DGTAL}/examples/samples/Al.100.vol
\endverbatim

   Results are displayed, then saved in 'geom-cvx.obj'. You may also
   analyse the same shape in multiscale fashion with

\verbatim
fullConvexityAnalysis3D 2 ${DGTAL}/examples/samples/Al.100.vol
\endverbatim
 
   The result is saved in 'geom-scale-cvx.obj'. You will obtain images
   like below, where green means convex, blue means concave, white is
   planar and red is atypical (see \cite lachaud_dgmm_2021 for details).

<table>
<tr><td>
\image html al100-analysis-1.jpg "Full convexity analysis at scale 1" width=100%
</td><td>
\image html al100-analysis-2.jpg "Full convexity analysis at scale 2" width=100%
</td><td>
\image html al100-analysis-3.jpg "Full convexity analysis at scale 3" width=100%
</td><td>
\image html al100-smooth-analysis-1-5.jpg "Full convexity smooth multiscale analysis (scales 1-5)" width=100%
</td></tr>
</table>

 \example geometry/volumes/fullConvexityAnalysis3D.cpp
 */


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/geometry/volumes/NeighborhoodConvexityAnalyzer.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;
typedef Shortcuts< KSpace > SH3;

///////////////////////////////////////////////////////////////////////////////


template < typename KSpace, int N >
struct Analyzer {
  typedef typename KSpace::Point Point;
  typedef NeighborhoodConvexityAnalyzer< KSpace, N > NCA;

  template < typename ImagePtr >
  static
  std::vector<Point>
  debug_one( const KSpace& aK, Point p, ImagePtr bimage )
  {
    NCA nca( aK.lowerBound(), aK.upperBound(),
             KSpace::dimension <= 2 ? 0 : 10000*KSpace::dimension*N );
    auto& image = *bimage;
    int    geom = 0;
    nca.setCenter( p, image );
    bool  cvx = nca.isFullyConvex( true );
    bool ccvx = nca.isComplementaryFullyConvex( false );
    auto cfg  = nca.makeConfiguration( nca.configuration(), true, false );
    std::vector< Point > localCompX;
    nca.getLocalCompX( localCompX, false );
    std::cout << "InC=" << nca.configuration() << std::endl;
    std::cout << "Cfg=" << cfg << std::endl;
    for ( auto q : localCompX ) std::cout << q;
    std::cout << std::endl;
    geom = ( cvx ? 0x1 : 0x0 ) | ( ccvx ? 0x2 : 0x0 );
    std::cout << "cvx=" << cvx << " ccvx=" << ccvx << std::endl;
    std::cout << "geom=" << geom << std::endl;
    return localCompX;
  }

  template < typename ImagePtr >
  static
  std::vector<int>
  run( const KSpace& aK, std::vector<Point> pts, ImagePtr bimage )
  {
    NCA nca( aK.lowerBound(), aK.upperBound(), 0 );
    // KSpace::dimension <= 2 ? 0 : 10000*KSpace::dimension*N );
    auto& image = *bimage;
    std::vector<int> result;
    std::map< Point, int > computed;
    int geom;
    int i  = 0;
    int nb = pts.size();
    int  nb_cvx = 0;
    int nb_ccvx = 0;
    for ( auto p : pts )
      {
        if ( i % 100 == 0 ) trace.progressBar( i, nb );
        auto it = computed.find( p );
        if ( it == computed.end() )
          {
            nca.setCenter( p, image );
            bool  cvx = nca.isFullyConvex( true );
            bool ccvx = nca.isComplementaryFullyConvex( false );
            if ( cvx  ) nb_cvx  += 1;
            if ( ccvx ) nb_ccvx += 1;
            geom = ( cvx ? 0x1 : 0x0 ) | ( ccvx ? 0x2 : 0x0 );
            computed[ p ] = geom;
          }
        else geom = it->second;
        result.push_back( geom );
        i++; 
      }
    trace.info() << "nb_cvx=" << nb_cvx << " nb_ccvx=" << nb_ccvx << std::endl;
    return result;    
  }

  template < typename ImagePtr >
  static
  void
  run( std::vector<int> & to_update,
       const KSpace& aK, std::vector<Point> pts, ImagePtr bimage )
  {
    NCA nca( aK.lowerBound(), aK.upperBound() );
    // KSpace::dimension <= 2 ? 0 : 10000*KSpace::dimension*N );
    auto& image = *bimage;
    std::map< Point, int > computed;
    int geom;
    int i  = 0;
    int nb = pts.size();
    for ( auto p : pts )
      {
        if ( i % 100 == 0 ) trace.progressBar( i, nb );
        auto it = computed.find( p );
        if ( it == computed.end() )
          {
            nca.setCenter( p, image );
            bool  cvx = ( to_update[ i ] & 0x1 )
              ? nca.isFullyConvex( true )
              : false;
            bool ccvx = ( to_update[ i ] & 0x2 )
              ? nca.isComplementaryFullyConvex( false )
              : false;
            geom = ( cvx ? 0x1 : 0x0 ) | ( ccvx ? 0x2 : 0x0 );
            computed[ p ] = geom;
          }
        else geom = it->second;
        to_update[ i++ ] = geom;
      }
  }
};

template < typename KSpace, int N >
struct MultiScaleAnalyzer {
  typedef typename KSpace::Point Point;
  typedef NeighborhoodConvexityAnalyzer< KSpace, N > NCA;
  typedef std::pair< int, int > Geometry; // convex, concave

  template < typename ImagePtr >
  static
  std::vector< Geometry >
  multiscale_run( const KSpace& aK,
                  std::vector<Point> pts,
                  ImagePtr bimage )
  {
    auto prev_geometry
      = MultiScaleAnalyzer< KSpace, N-1>::multiscale_run( aK, pts, bimage );
    trace.info() << "------- Analyzing scale " << N << " --------" << std::endl;
    std::vector< int > geom( prev_geometry.size() );
    for ( int i = 0; i < geom.size(); i++ )
      geom[ i ] = ( prev_geometry[ i ].first == N-1 ? 0x1 : 0x0 )
        |  ( prev_geometry[ i ].second == N-1 ? 0x2 : 0x0 );
    Analyzer< KSpace, N>::run( geom, aK, pts, bimage );
    for ( int i = 0; i < geom.size(); i++ ) {
      prev_geometry[ i ].first  += ( geom[ i ] & 0x1 ) ? 1 : 0;
      prev_geometry[ i ].second += ( geom[ i ] & 0x2 ) ? 1 : 0;
    }
    return prev_geometry;
  }
};

/// Specialization
template < typename KSpace>
struct MultiScaleAnalyzer< KSpace, 0 > {
  typedef typename KSpace::Point Point;
  typedef std::pair< int, int > Geometry; // convex, concave

  template < typename ImagePtr >
  static
  std::vector< Geometry >
  multiscale_run( const KSpace& aK,
                  std::vector<Point> pts,
                  ImagePtr bimage )
  {
    ((void) aK);
    ((void) bimage);
    return std::vector< Geometry >( pts.size(), std::make_pair( 0, 0 ) );
  }
};

int main( int argc, char** argv )
{
  if ( argc <= 2 )
    {
      trace.info() << "Usage: " << argv[ 0 ] << " <K> <input.vol> <m> <M>" << std::endl;
      trace.info() << "\tAnalyze the  shape with local full convexity" << std::endl;
      trace.info() << "\t- 1 <= K <= 5: analysis at scale K" << std::endl;
      trace.info() << "\t- K == 0: multiscale analysis (using scales 1-5)" << std::endl;
      trace.info() << "\t- input.vol: choose your favorite shape" << std::endl;
      trace.info() << "\t- m [==0], M [==255]: used to threshold input vol image" << std::endl;
      return 1;
    }
  int         N = atoi( argv[ 1 ] );
  std::string fn= argv[ 2 ];
  int         m = argc > 3 ? atoi( argv[ 3 ] ) : 0;
  int         M = argc > 4 ? atoi( argv[ 4 ] ) : 255;

  QApplication application(argc,argv);

  auto   params  = SH3::defaultParameters();
  
  // Domain creation from two bounding points.
  trace.info() << "Building set or importing vol ... ";
  KSpace K;
  params( "thresholdMin", m );
  params( "thresholdMax", M );
  auto bimage = SH3::makeBinaryImage( fn, params );
  K = SH3::getKSpace( bimage );
  trace.info() << "  [Done]" << std::endl;
  // Compute surface
  params( "surfaceComponents" , "All" );
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
  if ( N != 0 )
    {
      std::vector< int > result;
      trace.beginBlock ( "Single scale analysis" );
      if ( N == 1 ) result = Analyzer< KSpace, 1 >::run( K, points, bimage );
      if ( N == 2 ) result = Analyzer< KSpace, 2 >::run( K, points, bimage );
      if ( N == 3 ) result = Analyzer< KSpace, 3 >::run( K, points, bimage );
      if ( N == 4 ) result = Analyzer< KSpace, 4 >::run( K, points, bimage );
      if ( N == 5 ) result = Analyzer< KSpace, 5 >::run( K, points, bimage );
      trace.endBlock();
      SCell dummy;
      Color colors[ 4 ] =
        { Color( 255, 0, 0, 255 ), Color( 0, 255, 0, 255 ),
          Color( 0, 0, 255, 255 ), Color( 255, 255, 255, 255 ) };
      auto surfels   = SH3::getSurfelRange( surface, params );
      SH3::Colors all_colors( surfels.size() );
      for ( int i = 0; i < surfels.size(); i++ )
        {
          const auto    j = surfel2idx[ surfels[ i ] ];
          all_colors[ i ] = colors[ result[ j ] ];
        }
      SH3::saveOBJ( surface, SH3::RealVectors(), all_colors, "geom-cvx.obj" );
      Viewer3D<> viewer;
      viewer.setWindowTitle("fullConvexityAnalysis3D");
      viewer.show();  
      int i = 0;
      viewer << SetMode3D( dummy.className(), "Basic" );
      for ( auto s : (*surface) )
        {
          viewer << CustomColors3D( all_colors[ i ], all_colors[ i ] )
                 << s;
          i++;
        }
      viewer<< Viewer3D<>::updateDisplay;
      application.exec();
    }
  else
    {
      trace.beginBlock ( "Multiscale analysis" );
      auto geometry =
        MultiScaleAnalyzer< KSpace, 5 >::multiscale_run( K, points, bimage );
      trace.endBlock();
      Color colors_planar[ 6 ] =
        { Color( 0, 255, 255, 255),
          Color( 50, 255, 255, 255), Color( 100, 255, 255, 255),
          Color( 150, 255, 255, 255), Color( 200, 255, 255, 255 ),
          Color( 255, 255, 255, 255 ) };
      Color color_atypical( 255, 0, 0, 255 ); 
      Color colors_cvx[ 5 ] =
        { Color( 0, 255, 0, 255 ), Color( 50, 255, 50, 255 ),
          Color( 100, 255, 100, 255 ), Color( 150, 255, 150, 255 ),
          Color( 200, 255, 200, 255 ) };
      Color colors_ccv[ 5 ] =
        { Color( 0, 0, 255, 255 ), Color( 50, 50, 255, 255 ),
          Color( 100, 100, 255, 255 ), Color( 150, 150, 255, 255 ),
          Color( 200, 200, 255, 255 ) };
      auto surfels   = SH3::getSurfelRange( surface, params );
      SH3::Colors all_colors( surfels.size() );
      for ( int i = 0; i < surfels.size(); i++ ) {
        const auto j = surfel2idx[ surfels[ i ] ];
        int m0 = std::min( geometry[ j ].first, geometry[ j ].second );
        int m1 = std::max( geometry[ j ].first, geometry[ j ].second );
        if ( m1 == 0 ) all_colors[ i ] = color_atypical;
        else if ( m0 == m1 ) all_colors[ i ] = colors_planar[ 5 ];
        else if ( geometry[ j ].first > geometry[ j ].second )
          all_colors[ i ] = colors_cvx[ 5 - abs( m0 - m1 ) ];
        else
          all_colors[ i ] = colors_ccv[ 5 - abs( m0 - m1 ) ];
      }
      SH3::saveOBJ( surface, SH3::RealVectors(), all_colors, "geom-scale-cvx.obj" );
      SCell dummy;
      int i = 0;
      Viewer3D<> viewer;
      viewer.setWindowTitle("fullConvexityAnalysis3D");
      viewer.show();  
      viewer << SetMode3D( dummy.className(), "Basic" );
      for ( auto s : (*surface) )
        {
          viewer << CustomColors3D( all_colors[ i ], all_colors[ i ] )
                 << s;
          i++;
        }
      viewer<< Viewer3D<>::updateDisplay;
      application.exec();
    }      
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

