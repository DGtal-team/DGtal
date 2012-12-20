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
 * @file testSelectionViewer.cpp
 * @ingroup Tests
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * INSA Lyon
 * @date 2015/08/13
 *
 * An example file named test-selection-viewer.
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
//! [ImportSelectionfile]
  new ViewerOgre3D();
  DGtal::ViewerOgre3D & View = DGtal::ViewerOgre3D::getSingleton();

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );

  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );
  View << shape_set1;

  DGtal::Z3i::Point p1 ( 0, 0, 0 );

  View >> p1;
  shape_set1.erase ( p1 );

  View >> p1;
  shape_set1.erase ( p1 );

  View.clearScene();
  View << shape_set1;
  View.start();
//! [ImportSelectionfile]
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
