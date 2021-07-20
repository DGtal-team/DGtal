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
			const ConfigMap & map,
                        bool complement,
                        bool with )
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
	    if ( *it == c ) {
              if ( with ) 
                board << CustomStyle( q.className(),
                                      simple
                                      ? new CustomColors( Color( 0, 0, 0 ),
                                                          Color( 30, 128, 30 ) )
                                      : new CustomColors( Color( 0, 0, 0 ),
                                                          Color( 128, 30, 30 ) ) );
              else
                board << CustomStyle( q.className(),
                                      simple
                                      ? new CustomColors( Color( 0, 0, 0 ),
                                                          Color( 200, 255, 200 ) )
                                      : new CustomColors( Color( 0, 0, 0 ),
                                                          Color( 255, 200, 200 ) ) );
                
            } else {
              bool in_cfg  = cfg & mask;
              bool display = complement ? ( ! in_cfg ) : in_cfg;
              if ( display )
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


int main( int argc, char** argv )
{
  DConv dconv( Point::diagonal( -5 ), Point::diagonal( 5 ) );
  
  trace.beginBlock ( "Generate 2d table" );
  ConfigMap table_with    ( 256, false );
  ConfigMap table_without ( 256, false );
  ConfigMap table_cwith   ( 256, false );
  ConfigMap table_cwithout( 256, false );
  Point p1 = Point::diagonal( -1 );
  Point p2 = Point::diagonal(  1 );
  Point c = Point::diagonal( 0 );
  Domain domain( p1, p2 );
  unsigned int cfg = 0;
  KSpace K;
  K.init( p1, p2, true );
  NeighborhoodConvexityAnalyzer< KSpace, 1  > LCA( K );
  ImageContainerBySTLVector< Domain, bool > image( domain );
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
            if ( p != c )
              {
                image.setValue( p, cfg & mask );
                mask <<= 1;
              }
	  }
        // Checking full convexity.
        LCA.setCenter( c, image );
        // for ( auto p : Xwith ) std::cout << p;
        bool full_with     = LCA.isFullyConvex( true );
        bool full_without  = LCA.isFullyConvex( false );
        bool full_cwith    = LCA.isComplementaryFullyConvex( true );
        bool full_cwithout = LCA.isComplementaryFullyConvex( false );
        table_with    [ cfg ] = full_with;
        table_without [ cfg ] = full_without;
        table_cwith   [ cfg ] = full_cwith;
        table_cwithout[ cfg ] = full_cwithout;
        cfg += 1;
      }
  trace.endBlock();
  trace.beginBlock ( "Display 2d tables" );
  {
    Board2D board;
    displaySimplicityTable( board, table_with, false, true );
    board.saveEPS( "table-with.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_without, false, false );
    board.saveEPS( "table-without.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_cwith, true, true );
    board.saveEPS( "table-cwith.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_cwithout, true, false );
    board.saveEPS( "table-cwithout.eps" );
  }
  trace.endBlock();

  return 0;
}
