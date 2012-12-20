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
 * @file logoDGtal.cpp
 * @ingroup Examples
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * CNRS, LIRIS UMR 5205, Université de Lyon, France
 *
 * @date 2012/04/19
 * 
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  trace.beginBlock ( "Generate DGtal logo (without drop shadow)" );

  //! [logoDGtal-main]
  Z2i::Point p1( 0,0 );
  Z2i::Point p2( 26, 8 );
  Domain domain( p1, p2 );
 
  Board2D board;
  board << SetMode( domain.className(), "Paving" ) << domain;
  board << CustomStyle( p1.className(), 
                        new CustomPen( Color(0,0,0), Color(180,180,180), 2.5, 
                                       Board2D::Shape::SolidStyle,
                                       Board2D::Shape::RoundCap,
                                       Board2D::Shape::RoundJoin ))
        << Z2i::Point(1,1) << Z2i::Point(1,2) << Z2i::Point(1,3)
        << Z2i::Point(1,4) << Z2i::Point(1,5) << Z2i::Point(1,6)
        << Z2i::Point(1,7) << Z2i::Point(2,1) << Z2i::Point(3,1) << Z2i::Point(4,1)
        << Z2i::Point(4,2) << Z2i::Point(5,2) << Z2i::Point(5,3)
        << Z2i::Point(5,4) << Z2i::Point(5,5) << Z2i::Point(5,6)
        << Z2i::Point(4,6) << Z2i::Point(4,7) << Z2i::Point(3,7) 
        << Z2i::Point(2,7) << Z2i::Point(9,1) << Z2i::Point(9,2)
        << Z2i::Point(8,2) << Z2i::Point(8,3)
        << Z2i::Point(8,4) << Z2i::Point(8,5) << Z2i::Point(8,6) 
        << Z2i::Point(9,6) << Z2i::Point(9,7) << Z2i::Point(10,7) 
        << Z2i::Point(11,7) << Z2i::Point(10,1) << Z2i::Point(11,1) << Z2i::Point(12,1)
        << Z2i::Point(12,2) << Z2i::Point(12,3) << Z2i::Point(12,4) << Z2i::Point(11,4);      
  
  board << CustomStyle( p1.className(), 
                        new CustomPen( Color(0,0,0), Color(0,0,0), 1.0, 
                                       Board2D::Shape::SolidStyle,
                                       Board2D::Shape::RoundCap,
                                       Board2D::Shape::RoundJoin ))
        << Z2i::Point(15,1) << Z2i::Point(16,1) << Z2i::Point(17,1)
        << Z2i::Point(15,2) << Z2i::Point(15,3) << Z2i::Point(15,4)
        << Z2i::Point(15,5) << Z2i::Point(16,4)
        << Z2i::Point(19,1) << Z2i::Point(21,1) << Z2i::Point(20,1) << Z2i::Point(22,1)
        << Z2i::Point(19,2) << Z2i::Point(21,2)
        << Z2i::Point(19,3) << Z2i::Point(20,3) << Z2i::Point(21,3)
        << Z2i::Point(24,1) << Z2i::Point(25,1)
        << Z2i::Point(24,2) << Z2i::Point(24,3) << Z2i::Point(24,4) << Z2i::Point(24,5);
  
  board.saveSVG("logoDGtal.svg");
  //! [logoDGtal-main]

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
