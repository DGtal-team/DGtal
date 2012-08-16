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
 * @file testSetViewer.cpp
 * @ingroup Tests
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * INSA Lyon
 * @date 2015/08/13
 *
 * An example file named test-set-viewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/viewers/OGRE/ViewerOgre3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include <iostream>

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
//! [ImportSetfile]
  new ViewerOgre3D();
  ViewerOgre3D & viewer = ViewerOgre3D::getSingleton();
  Point p1( 0, 0, 0 );
  Point p2( 10, 10 , 10 );
  Domain domain( p1, p2 );
  DigitalSet shape_set( domain );
  Shapes<Domain>::addNorm1Ball( shape_set, Point( 5, 5, 5 ), 2 );
  Shapes<Domain>::addNorm2Ball( shape_set, Point( 3, 3, 3 ), 2 );
  shape_set.erase(Point(3,3,3));
  shape_set.erase(Point(6,6,6)); 
  viewer << shape_set;
  viewer.start(); 
//! [ImportSetfile]
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
