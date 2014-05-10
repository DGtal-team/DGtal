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
 * @file testBoard3d.cpp
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@dbslabs.com.br )
 *
 * @date 2014/05/03
 *
 * Functions for testing class Board3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include "DGtal/io/boards/Board3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

bool testClearBoard3d()
{
  unsigned int nbok = 0;
  unsigned int nb = 4;
  
  KSpace K;
  Point plow(0,0,0);
  Point pup(20,20,20);
  Domain domain( plow, pup );
  K.init( plow, pup, true );
  Board3D<Space, KSpace> board(K);
  
  trace.beginBlock("Testing Board3D: SCells"); 
  SCell v = K.sSpel( Point( 0, 0, 0 ), KSpace::POS );
  SCell v2 = K.sSpel( Point( 1, 0, 0 ), KSpace::POS );
  SCell v3 = K.sSpel( Point( 0, 1, 0 ), KSpace::POS );

  board << v << v2 << v3;
  board.saveOBJ("board3d-cells.obj");
  board.clear();
  trace.info() << "File written as \"board3d-cells.obj\". All sent SCels were removed." << std::endl; 
  nbok++;
  trace.endBlock();
  
  trace.beginBlock("Testing Board3D: 3D points"); 
  Point p1( -3, -2, 0 );
  Point p2( 7, 3 , 6);
  Point p3( -1, -1, -1);
  
  board << p1 << p2 << p3;
  board.saveOBJ("board3D-points.obj");
  board.clear();  
  trace.info() << "File written as \"board3D-points.obj\". All sent points were removed." << std::endl;  
  nbok++;
  trace.endBlock();
  
  trace.beginBlock("Testing Board3D: Ball and clipping palnes"); 
  DigitalSet shape_set(domain);
  Shapes<Domain>::addNorm2Ball( shape_set, Point( 10, 10, 10 ), 7 );
  board << SetMode3D( shape_set.className(), "Both" );
  board << shape_set;
  board << CustomColors3D(Color(250, 200,0, 100),Color(250, 200,0, 20));
  board << SetMode3D( p1.className(), "Paving" );
  board << ClippingPlane(1,0,0,-4.9);
  board << ClippingPlane(0,1,0.3,-10);
  
  board.saveOBJ("board3d-clipped-ball.obj");
  board.clear();
  trace.info() << "File written as \"board3d-clipped-ball.obj\". All sent clipping palnes and ball were removed." << std::endl;  
  nbok++;
  trace.endBlock();
  
  trace.beginBlock("Testing Board3D: Empty file."); 
  board.saveOBJ("null.obj");
  trace.info() << "File written as \"null.obj\"." << std::endl;
  nbok++;
  trace.endBlock(); 
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////

int main(int, char**)
{

  trace.beginBlock ( "Testing class Board3D" );
  trace.info() << "Testing Display3D::clear()" << std::endl;
  bool res = testClearBoard3d();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}