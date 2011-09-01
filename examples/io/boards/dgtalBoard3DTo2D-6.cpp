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
 * @file   dgtalCairo-6.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 25 mai 2011
 * 
 * @brief
 *
 * Simple example of class Board3DTo2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/io/boards/Board3DTo2D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  Board3DTo2D viewer;

  Point p1( 0, 0, 0 );
  Point p2( 20, 20, 20 );
  Domain domain(p1, p2);

  DigitalSet shape_set( domain );

  Shapes<Domain>::addNorm2Ball( shape_set, Point( 10, 10, 10 ), 7 );
  viewer << SetMode3D( shape_set.styleName(), "Both" );
  viewer << shape_set;
  viewer << CustomColors3D(Color(250, 200,0, 100),Color(250, 200,0, 20));
  viewer << SetMode3D( p1.styleName(), "Paving" );
  
  viewer << CameraPosition(10.000000, 10.000000, 41.682465)
   << CameraDirection(0.000000, 0.000000, -1.000000)
   << CameraUpVector(0.000000, 1.000000, 0.000000);
  
  //viewer << SetMode3DCairo(viewer.styleName(), "WireFrameMode");
  viewer.saveCairo("dgtalCairo-6.png", Board3DTo2D::CairoPNG, 600, 400);
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////




