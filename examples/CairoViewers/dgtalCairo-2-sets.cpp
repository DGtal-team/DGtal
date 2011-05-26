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
 * @file   dgtalCairo-2-sets.cpp
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
  Point p2( 10, 10 , 10 );
  Domain domain( p1, p2 );

  viewer << domain;

  DigitalSet shape_set( domain );
  Shapes<Domain>::addNorm1Ball( shape_set, Point( 5, 5, 5 ), 2 );
  Shapes<Domain>::addNorm2Ball( shape_set, Point( 3, 3, 3 ), 2 );

  shape_set.erase(Point(3,3,3));
  shape_set.erase(Point(6,6,6));
  viewer << shape_set;

  viewer.setCameraPosition(5.000000, 5.000000, 29.893368);
  viewer.setCameraDirection(0.000000, 0.000000, -1.000000);
  viewer.setCameraUpVector(0.000000, 1.000000, 0.000000);
  
  viewer.saveCairo("dgtalCairo-2-sets.png", DGtalCairo::CairoPNG, 600, 400);
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////




