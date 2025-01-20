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
 * @file geometry/volumes/fullConvexityLUT2D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/22
 *
 * An example file named fullConvexityLUT2D. 
 *
 * This file is part of the DGtal library.
 */

/** 
 * This example creates precomputed
 * tables for determining whether some 3x3 neighborhood of a point is
 * fully convex, collapsible, etc. More precisely it produces the
 * following tables, if the neighbor points are the possible 8 points
 * around the point of interest.
 *
 * - table-fcvx-with-center : 'true' iff the center point and its
 *   neighbor points are fully convex ;
 * - table-fcvx-without-center" : 'true' iff the neighbor points 
 *   without the center point are fully convex ;
 * - table-complementary-fcvx-with-center : 'true' iff the center point 
 *   and the complementary points of its neighbor points are fully convex ;
 * - table-complementary-fcvx-without-center : 'true' iff the complementary
 *   points of the neighbor points are full convex ;
 * - table-fcvx-regular : 'true' if the point is \b regular, meaning that 
 *   the center point and its neighbor points are fully convex, while the
 *   complementary points of the neighbor points are fully convex ;
 * - table-fcvx-collapsible : 'true' if the point is \b collapsible,
 *   meaning that the center and its neighbor points are fully convex,
 *   the neighbor points without the center point are also fully convex,
 *   and the center point is not isolated.
 *
 * @see \ref moduleDigitalConvexityApplications
 *
 * \example geometry/volumes/fullConvexityLUT2D.cpp
 */

///////////////////////////////////////////////////////////////////////////////
#include <vector>
#include <fstream>
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
/**
   Output simplicity configuration table as a C array.
*/
void
outputTableAsArray( ostream & out,
		    const string & tableName,
		    const ConfigMap & map )
{
  out << "const bool " << tableName << "[ " << map.size() << " ] = { ";
  for ( auto  it = map.cbegin(), it_end = map.cend();
	it != it_end; )
    {
      out << (int) *it;
      ++it;
      if ( it != it_end ) out << ", ";
    }
  out << " };" << std::endl;
}

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
  ((void) argc); ((void) argv);
  DConv dconv( Point::diagonal( -5 ), Point::diagonal( 5 ) );
  
  trace.beginBlock ( "Generate 2d tables" );
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
  trace.beginBlock ( "Computing topology-related tables" );
  ConfigMap table_regular   ( 256, false );
  for ( cfg = 0; cfg < 256; cfg++ )
    table_regular[ cfg ] = table_with[ cfg ] && table_without[ 255 - cfg ];
  ConfigMap table_collapsible( 256, false );
  for ( cfg = 0; cfg < 256; cfg++ )
    table_collapsible[ cfg ] = table_with[ cfg ] && table_without[ cfg ]
      && ( cfg != 0 );
  trace.endBlock();
  trace.beginBlock ( "Display 2d tables" );
  {
    Board2D board;
    displaySimplicityTable( board, table_with, false, true );
    board.saveEPS( "table-fcvx-with-center.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_without, false, false );
    board.saveEPS( "table-fcvx-without-center.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_cwith, true, true );
    board.saveEPS( "table-complementary-fcvx-with-center.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_cwithout, true, false );
    board.saveEPS( "table-complementary-fcvx-without-center.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_regular, false, true );
    board.saveEPS( "table-fcvx-regular.eps" );
  }
  {
    Board2D board;
    displaySimplicityTable( board, table_collapsible, false, true );
    board.saveEPS( "table-fcvx-collapsible.eps" );
  }
  trace.endBlock();
  trace.beginBlock ( "Output 2d tables as C arrays" );
  ofstream out( "table-fcvx.cpp" );
  outputTableAsArray( out, "table-fcvx-with-center",
                      table_with );
  outputTableAsArray( out, "table-fcvx-without-center",
                      table_without );
  outputTableAsArray( out, "table-complementary-fcvx-with-center",
                      table_cwith );
  outputTableAsArray( out, "table-complementary-fcvx-without-center",
                      table_cwithout );
  outputTableAsArray( out, "table-fcvx-regular",
                      table_regular );
  outputTableAsArray( out, "table-fcvx-collapsible",
                      table_collapsible );
  out.close();
  trace.endBlock();
  return 0;
}
