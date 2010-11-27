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
//LICENSE-END
/**
 * @file dgtalboard-3-custom-points.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/26
 *
 * An example file named dgtalboard-3-custom-points.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;

///////////////////////////////////////////////////////////////////////////////
struct MyDrawStyleCustomColor : public DrawableWithBoard
{
  DGtalBoard::Color myPenColor;
  DGtalBoard::Color myFillColor;
  MyDrawStyleCustomColor( const DGtalBoard::Color & penColor,
			  const DGtalBoard::Color & fillColor )
    : myPenColor( penColor ), myFillColor( fillColor )
  {}

  virtual void selfDraw( DGtalBoard & aboard) const
  {
    aboard.setFillColor( myFillColor);
    aboard.setPenColor( myPenColor );
  }
};

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example dgtalboard-3-custom-points" );

  Point p1( -3, -2 );
  Point p2( 7, 3 );
  Point p3( 0, 0 );
  Domain domain( p1, p2 );

  DGtalBoard::Color red( 255, 0, 0 );
  DGtalBoard::Color dred( 192, 0, 0 );
  DGtalBoard::Color green( 0, 255, 0 );
  DGtalBoard::Color dgreen( 0, 192, 0 );
  DGtalBoard::Color blue( 0, 0, 255 );
  DGtalBoard::Color dblue( 0, 0, 192 );
  
  DGtalBoard board;
  board << domain 
	<< CustomStyle( p1.styleName(), new MyDrawStyleCustomColor( red, dred ) )
	<< p1
	<< CustomStyle( p2.styleName(), new MyDrawStyleCustomColor( green, dgreen ) )
	<< p2
	<< CustomStyle( p3.styleName(), new MyDrawStyleCustomColor( blue, dblue ) )
	<< p3;
  board.saveSVG("dgtalboard-3-custom-points.svg");
  board.saveEPS("dgtalboard-3-custom-points.eps");
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
