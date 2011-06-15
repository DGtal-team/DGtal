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
 * Simple example of class DGtalCairo.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/io-viewers/CairoViewers/DGtalCairo.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shapes.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  DGtalCairo viewer;

  Point p1( 0, 0, 0 );
  Point p2( 20, 20, 20 );
  Domain domain(p1, p2);

  DigitalSet shape_set( domain );

  Shapes<Domain>::addNorm2Ball( shape_set, Point( 10, 10, 10 ), 7 );
  viewer << SetMode3DCairo( shape_set.styleName(), "Both" );
  viewer << shape_set;
  viewer << CustomColors3DCairo(QColor(250, 200,0, 100),QColor(250, 200,0, 20));
  viewer <<  SetMode3DCairo( p1.styleName(), "Paving" );

  viewer.setCameraPosition(10.000000, 10.000000, 41.682465);
  viewer.setCameraDirection(0.000000, 0.000000, -1.000000);
  viewer.setCameraUpVector(0.000000, 1.000000, 0.000000);
  
  //viewer.setWireFrame(true);
  viewer.saveCairo("dgtalCairo-6.png", DGtalCairo::CairoPNG, 600, 400);
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////




