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
 * @file topology/generateSimplicityTables2D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/06/22
 *
 * An example file named generateSimplicityTables2D. Creates precomputed
 * tables for determining whether some point is simple within an
 * object.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtal/geometry/volumes/NeighborhoodConvexityAnalyzer.h"
#include "DGtal/helpers/Shortcuts.h"
#include "ConfigExamples.h"

// Using standard 2D digital space.
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

typedef std::vector<bool> ConfigMap;
typedef DigitalConvexity< KSpace > DConv;
typedef Shortcuts<KSpace> SH2;
typedef NeighborhoodConvexityAnalyzer<KSpace,1> NCA1;

///////////////////////////////////////////////////////////////////////////////

void
displaySimplicityTable( Board2D & board,
			const ConfigMap & map )
{
  Point p1 = Point::diagonal( -1 );
  Point p2 = Point::diagonal(  1 );
  Point c = Point::diagonal( 0 );
  Domain domain( p1, p2 );

  Point q1 = Point::diagonal( -1 );
  Point q2 = Point::diagonal( 4*16-1 );
  Domain fullDomain( q1, q2 );
  board << SetMode( fullDomain.className(), "Paving" );
  unsigned int cfg = 0;
  for ( unsigned int y = 0; y < 16; ++y )
    for ( unsigned int x = 0; x < 16; ++x, ++cfg )
      {
	bool simple = map[ cfg ];
	Point base( x*4, y*4 );
	unsigned int mask = 1;
	for ( auto it = domain.begin();
	      it != domain.end(); ++it )
	  {
	    Point q = base + (*it);
	    if ( *it == c )
	      board << CustomStyle( q.className(),
                                    simple
				    ? new CustomColors( Color( 0, 0, 0 ),
                                                        Color( 30, 128, 30 ) )
				    : new CustomColors( Color( 0, 0, 0 ),
                                                        Color( 128, 30, 30 ) ) );
	    else
	      {
		if ( cfg & mask )
		  board <<
		    CustomStyle( q.className(),
				 simple
				 ? new CustomColors( Color( 0, 0, 0 ),
						     Color( 10, 255, 10 ) )
				 : new CustomColors( Color( 0, 0, 0 ),
						     Color( 255, 10, 10 ) ) );
		else
		  board <<
		    CustomStyle( q.className(),
				 simple
				 ? new CustomColors( Color( 0, 0, 0 ),
						     Color( 245, 255, 245 ) )
				 : new CustomColors( Color( 0, 0, 0 ),
						     Color( 255, 245, 245 ) ) );
		mask <<= 1;
	      }
	    board << q;
	  }
      }
}

/**
   Output simplicity configuration table as a C(++) array.
*/
template <typename Map>
void
outputTableAsArray( ostream & out,
		    const Map & map,
		    const string & tableName )
{
  typedef typename Map::const_iterator MapConstIterator;
  out << "const bool " << tableName << "[ " << map.size() << " ] = { ";
  for ( MapConstIterator it = map.begin(), it_end = map.end();
	it != it_end; )
    {
      out << *it;
      ++it;
      if ( it != it_end ) out << ", ";
    }
  out << " };" << std::endl;
}


int main( int argc, char** argv )
{
  double noise = argc > 1 ? atof( argv[ 1 ] ) : 0.1;
  DConv dconv( Point::diagonal( -5 ), Point::diagonal( 5 ) );
  
  trace.beginBlock ( "Generate 2d table" );
  ConfigMap table_with   ( 256, false );
  ConfigMap table_without( 256, false );
  ConfigMap table_both   ( 256, false );
  Point p1 = Point::diagonal( -1 );
  Point p2 = Point::diagonal(  1 );
  Point c = Point::diagonal( 0 );
  Domain domain( p1, p2 );
  unsigned int cfg = 0;
  for ( unsigned int y = 0; y < 16; ++y )
    for ( unsigned int x = 0; x < 16; ++x )
      {
        // Building a configuration.
        std::vector< Point > Xwith;
        std::vector< Point > Xwithout;
	Point base( x, y );
	unsigned int mask = 1;
	for ( auto it = domain.begin(); it != domain.end(); ++it )
	  {
            const Point p = *it;
            if ( p == c )
              Xwith.push_back( p );
            else
              {
                if ( cfg & mask )
                  {
                    Xwith.   push_back( p );
                    Xwithout.push_back( p );
                  }
                mask <<= 1;
              }
	  }
        // Checking full convexity.
        // for ( auto p : Xwith ) std::cout << p;
        bool full_with    = dconv.is0Convex( Xwith );
        // std::cout << "Xw full-convex=" << full_with << std::endl;
        bool full_without = dconv.is0Convex( Xwithout );
        // std::cout << "Xo full-convex=" << full_without << std::endl;
        table_with   [ cfg ] = full_with;
        table_without[ cfg ] = full_without;
        table_both   [ cfg ] = ( full_with && full_without );
        // std::cout  << "table_both[" << cfg << "]=" << table_both[ cfg ]
        //            << " #X=" << Xwith.size()
        //            << " #Xo=" << Xwithout.size()
        //            << std::endl;
        cfg += 1;
      }
  trace.endBlock();
  ConfigMap table_regular   ( 256, false );
  for ( cfg = 0; cfg < 256; cfg++ )
    table_regular[ cfg ] = table_with[ cfg ] && table_without[ 255 - cfg ];
  ConfigMap table_collapsible( 256, false );
  for ( cfg = 0; cfg < 256; cfg++ )
    table_collapsible[ cfg ] = table_with[ cfg ] && table_without[ cfg ] && ( cfg != 0 );
  // ultra-regular
  // table_regular[ cfg ] = table_with[ cfg ] && table_without[ 255 - cfg ]
  //   && table_without[ cfg ] && table_with[ 255 - cfg ];
  trace.beginBlock ( "Display 2d tables" );
  {
    Board2D board;
    displaySimplicityTable( board, table_with );
    board.saveEPS( "table-with.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_without );
    board.saveEPS( "table-without.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_both );
    board.saveEPS( "table-both.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_regular );
    board.saveEPS( "table-regular.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_collapsible );
    board.saveEPS( "table-collapsible.eps" );
  }
  trace.endBlock();

  auto params  = SH2::defaultParameters();
  params( "noise", noise )( "thresholdMin", 128 );
  auto g_image = SH2::makeGrayScaleImage( examplesPath + "samples/contourS.pgm" );
  auto b_image = SH2::makeBinaryImage   ( g_image, params );
  Board2D board;
  Domain image_domain = b_image->domain();
  NCA1 nca1( image_domain.lowerBound(), image_domain.upperBound() );
  for ( auto p : image_domain )
    {
      nca1.setCenter( p, (*b_image) );
      bool simple = nca1.isFullyConvexCollapsible();
      // Domain local_domain( p - Point::diagonal( 1 ), p + Point::diagonal( 1 ) );
      // unsigned int cfg = 0;
      // unsigned int mask = 1;
      // for ( auto q : local_domain )
      //   {
      //     if ( q != p ) {
      //       if ( image_domain.isInside( q ) && (*b_image)( q ) )
      //         cfg |= mask;
      //       mask <<= 1;
      //     }
      //   }
      // Detects nice pixels that can be added/removed while
      // guaranteeing full convexity before and after of background +
      // foreground
      // bool simple = (*b_image)( p )
      //   ? table_both[ cfg     ]
      //   : table_both[ 255-cfg ];
      
      // Detects foreground / background pixels that are full convex and 
      // bool simple = (*b_image)( p )
      //   ? table_with[ cfg ]
      //   : table_without[ cfg ];

      // Detects foreground / background pixels that should be flipped
      // bool simple = (*b_image)( p )
      //   ? ( ( ! table_with[ cfg ] || ! table_without[ 255-cfg ] )
      //       && table_with[ 255-cfg ] && table_without[ cfg ] )
      //   : ( ( ! table_with[ 255-cfg ] || ! table_without[ cfg ] )
      //       && table_with[ cfg ] ) && table_without[ 255-cfg ];
      if ( (*b_image)( p ) )
        board << CustomStyle( p.className(),
                              simple
                              ? new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 10, 255, 10 ) )
                              : new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 255, 10, 10 ) ) );
      else
        board << CustomStyle( p.className(),
                              simple
                              ? new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 180, 255, 180 ) )
                              : new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 255, 180, 180 ) ) );
      board << p;
    }
  board.saveEPS( "contour-both.eps" );
  return 0;
}
