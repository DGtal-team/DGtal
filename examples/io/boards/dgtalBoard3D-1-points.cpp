
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
 * @file dgtalBoard2D-1-points.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/26
 *
 * An example file named dgtalBoard2D-1-points.
 * 
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board3D.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  trace.beginBlock ( "Example dgtalBoard3D-1-points" );


  Point p1( -3, -2, 0 );
  Point p2( 7, 3 , 0);
  Point p3( 0, 0 , 0);
  Domain domain(p1, p2);


  
  Board3D board;

  board << domain;
  board << p1 << p2 << p3;

  board.saveOBJ("dgtalBoard3D-1-points.obj");


  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
