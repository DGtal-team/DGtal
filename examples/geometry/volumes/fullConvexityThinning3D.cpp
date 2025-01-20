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
 * @file geometry/volumes/fullConvexityThinning3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/16
 *
 * An example file named fullConvexityThinning3D
 *
 * This file is part of the DGtal library.
 */


/**
 * A full convexity thinning is an iterative removal of fully convex collapsible points from a given digital object. This is an experimental test.
 *
 * \example geometry/volumes/fullConvexityThinning3D.cpp
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

typedef NeighborhoodConvexityAnalyzer< KSpace, 1 > NCA;

int main( int argc, char** argv )
{
  trace.info() << "Usage: " << argv[ 0 ] << " <thickness> <convexity> <input.vol> <m> <M>" << std::endl;
  trace.info() << "    - convexity in {0,1}: 0=0-convexity, 1=full-convexity"<< std::endl;

  int thickness = argc > 1 ? atoi( argv[ 1 ] ) : 2;
  bool full_cvx = argc > 2 ? atoi( argv[ 2 ] ) == 1 : false;
  std::string fn= argc > 3 ? argv[ 3 ] : "";
  int         m = argc > 4 ? atoi( argv[ 4 ] ) : 0;
  int         M = argc > 5 ? atoi( argv[ 5 ] ) : 255;
  trace.beginBlock ( "Example of 3D shape thinning with full convexity properties" );
  
  QApplication application(argc,argv);
  Viewer3D<> viewer;
  viewer.setWindowTitle("fullConvexityThinning3D");
  viewer.show();  

  auto   params  = SH3::defaultParameters();
  
  // Domain creation from two bounding points.
  trace.info() << "Building set or importing vol ... ";
  Point c( 0, 0, 0 );
  Point p1( -50, -50, -50 );
  Point p2( 50, 50, 50 );
  Domain domain( p1, p2 );
  KSpace K;
  std::set< Point > shape_set;
  CountedPtr< SH3::BinaryImage > bimage( new SH3::BinaryImage( domain ) );
  if ( fn == "" )
    {
      K.init( p1, p2, true );
      for (Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it )
        {
          Point p = *it;
          if ( ((p - c ).norm() <= 22+thickness ) && ((p - c ).norm() >= 20-thickness)
               && ( ((p[0] <= thickness)&& (p[0] >= -thickness))
                    || ((p[1] <= thickness)&& (p[1] >= -thickness))))
            {
              shape_set.insert( p );
              bimage->setValue( p, true );
            }
          else
            bimage->setValue( p, false );
        }
    }
  else
    {
      params( "thresholdMin", m );
      params( "thresholdMax", M );
      bimage = SH3::makeBinaryImage( fn, params );
      K = SH3::getKSpace( bimage );
      p1 = K.lowerBound();
      p2 = K.upperBound();
      domain = Domain( p1, p2 );
      for ( auto p : domain )
        if ( (*bimage)( p ) ) shape_set.insert( p );
    }
  std::set< Point > origin_set( shape_set );
  trace.info() << "  [Done]" << std::endl;

  {
    params( "surfaceComponents" , "All" );
    auto surface = SH3::makeDigitalSurface( bimage, K, params );
                   SH3::saveOBJ( surface, "source.obj" );
  }
  
  trace.beginBlock ( "Thinning" );
  SH3::BinaryImage& image = *bimage;
  NCA nca( p1, p2, 10000 );
  int nb_simple=0; 
  std::set< Point >::iterator it, itE;
  std::set< Point > to_process( shape_set );
  do 
    {
      std::set< Point > next_to_process;
      nb_simple = 0;
      trace.info() << "Pass #S=" << shape_set.size()
                   << " #Q=" << to_process.size() << std::endl; 
      for ( it  = to_process.begin(), itE = to_process.end(); it != itE; ++it )
        {
          Point p = *it;
          if ( ! image( p ) ) continue; // already removed
          nca.setCenter( p, image );
          if ( full_cvx
               ? nca.isFullyConvexCollapsible()
               : nca.is0ConvexCollapsible() )
            {
              std::vector< Point > neighbors;
              nca.getLocalX( neighbors, false );
              for ( auto q : neighbors ) next_to_process.insert( q );
              shape_set.erase( p );
              image.setValue( p, false );
              ++nb_simple;
            }
        }
      trace.info() << "  => nb_removed=" << nb_simple<< std::endl;
      if ( nb_simple != 0 )
        std::swap( to_process, next_to_process );
    }
  while ( nb_simple != 0 );
  trace.endBlock();

  {
    params( "surfaceComponents" , "All" );
    auto surface = SH3::makeDigitalSurface( bimage, K, params );
    SH3::saveOBJ( surface, "geom-thinned.obj" );
  }
    
  // Display by using two different list to manage OpenGL transparency.
  DigitalSet origin( domain );
  DigitalSet output( domain );
  for ( auto p : origin_set ) origin.insert( p );
  for ( auto p : shape_set ) output.insert( p );
  viewer << SetMode3D( output.className(), "Paving" );
  viewer << CustomColors3D(Color(25,25,255, 255), Color(25,25,255, 255));
  viewer << output;

  viewer << SetMode3D( origin.className(), "PavingTransp" );
  viewer << CustomColors3D(Color(250, 0,0, 25), Color(250, 0,0, 5));
  viewer << origin;

  viewer<< Viewer3D<>::updateDisplay;
   
  
  trace.endBlock();
  return application.exec();

}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

