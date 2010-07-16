/**
 * @file testSimpleBoard.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/16
 *
 * Functions for testing class SimpleBoard.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "Board/Board.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;
  
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SimpleBoard.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleBoard()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board board;

  board.drawRectangle( -1, 1, 2.0, 2.0 );
  board.setPenColorRGBi( 0, 0, 255 );
  board.fillCircle( 0.0, 0.0, 0.05 );
  
  board.scale(100);

  board.saveSVG( "testsimpleboard.svg" );


  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SimpleBoard" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSimpleBoard(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
