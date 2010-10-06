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
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
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
  unsigned int nb = 2;
  
  Board board;

  board.setPenColorRGBi( 0, 0, 0);
  board.drawRectangle( -1, 1, 2.0, 2.0 );
  board.setPenColorRGBi( 0, 0, 255 );
  board.fillCircle( 2, 2, 1 );
  
  

  board.saveSVG( "simpleboard.svg" );
  board.saveFIG( "simpleboard.fig" );
  board.saveEPS( "simpleboard.eps" );
  nbok++;

  typedef  PointVector<2> Point2D;
  Point2D apoint, p2;
  apoint[0] = 5;
  p2[0] = 1;
  apoint[1] = 8;
  p2[1] = 1;

  board.setPenColorRGBi( 255, 0, 255 );
  apoint.selfDraw(board);
  board.setPenColorRGBi( 255, 0, 0 );
  apoint.selfDraw(board,p2);

  board.scale(10);

  board.saveSVG( "pointsimpleboard.svg" );
  board.saveFIG( "pointsimpleboard.fig" );
  board.saveEPS( "pointsimpleboard.eps" );
  nbok++;
  trace.endBlock();
  return nbok == nb;
}

bool testDomain()
{
    typedef SpaceND<2> TSpace;
    typedef TSpace::Point Point;
    Point a ( 1, 1);
    Point b ( 15, 15);

    trace.beginBlock ( "HyperRectDomain Iterator" );
    HyperRectDomain<TSpace> myDomain ( a,b );
    
    Board board;
    myDomain.selfDrawAsGrid(board);
    board.scale(10);
    board.saveSVG( "domain-grid.svg" );
    
    Board b2;
    myDomain.selfDrawAsPaving(b2);
    b2.scale(10);
    b2.saveSVG( "domain-paving.svg" );


    trace.endBlock();

    PointVector<3> pl;
    //An assert should be raised 
    //pl.selfDraw(b2);

    return true;
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

  bool res = testSimpleBoard() && testDomain(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
