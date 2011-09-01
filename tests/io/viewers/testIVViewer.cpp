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
 * @file testIVViewer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/04
 *
 * Functions for testing class IVViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <Qt/qapplication.h>
#include <Inventor/nodes/SoCube.h>
#include "DGtal/io/viewers/DGtalInventor.h"
#include "DGtal/io/viewers/IVViewer.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IVViewer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testIVViewerSimpleWdw( int argc, char** argv )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing block ..." );
  IVViewer ivv( argc, argv );
  ivv.show();
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testIVViewer( int argc, char** argv )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  string s = "testIVViewer";
  for ( int i = 1; i < argc; ++i )
    s += " " + string( argv[ i ] );

  IVViewer ivv( argc, argv );
  //ivv.show();
  // Setting camera
  ivv.setCamera( 30.0, 25.0 );
  // Gives hand to Inventor
  ivv.setTitle( s.c_str() );

  DGtalInventor<Space> inventor;
  typedef DGtalInventor<Space>::Color Color;
  inventor.setDiffuseColor( Color( 1.0, 0.0, 0.0 ) );
  inventor.drawPoint( Point( 1, 0, 0 ) );
  inventor.setDiffuseColor( Color( 0.0, 1.0, 0.0 ) );
  inventor.drawPoint( Point( 0, 1, 0 ) );
  inventor.setDiffuseColor( Color( 0.0, 0.0, 1.0 ) );
  inventor.drawPoint( Point( 0, 0, 1 ) );

  Point p1( -4, -4, -4 );
  Point p2( 17, 17, 17 );
  Domain domain( p1, p2 );
  DigitalSet shape_set( domain );
  Shapes<Domain>::addNorm1Ball( shape_set, Point( 3, 13, 3 ), 7 );
  Shapes<Domain>::addNorm1Ball( shape_set, Point( 14, 5, 2 ), 12 );
  inventor.setDiffuseColor( Color( 0.7, 0.7, 0.7 ) );
  for ( DigitalSet::ConstIterator it = shape_set.begin(); 
  it != shape_set.end();
  ++it )
    {
      const Point & p = *it;
      if ( ( p[ 0 ] < 0 ) || ( p[ 1 ] < 0 ) ||( p[ 2 ] < 0 ) )
  inventor.setDiffuseColor( Color( 0.7, 0.7, 1.0 ) );
      else
  inventor.setDiffuseColor( Color( 1.0, 1.0, 0.7 ) );
      inventor.drawPoint( *it );
    }
  inventor.generate( ivv.root() );
  // ivv->addChild( node );
  ivv.show();
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  QApplication app( argc, argv );
  trace.beginBlock ( "Testing class IVViewer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = 
    // testIVViewerSimpleWdw( argc, argv ) &&
    testIVViewer( argc, argv );
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
