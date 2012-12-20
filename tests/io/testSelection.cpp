/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

/**
* @file testSelection.cpp
* @ingroup Tests
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* INSA Lyon
*
* @date 2012/06/13
*
* Functions for testing class testSelection.cpp
*
* This file is part of the DGtal library.
*/

///////////////////////////////////////////////////////////////////////////////
#include "../ViewerOgre3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include <iostream>
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ViewerOgre3D.
///////////////////////////////////////////////////////////////////////////////
/**
* Example of a test. To be completed.
*
*/
bool testSelection()
{
  new ViewerOgre3D();
  DGtal::ViewerOgre3D & View = DGtal::ViewerOgre3D::getSingleton();

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );

  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );
  View << shape_set1;

  DGtal::Z3i::Point p10 ( 30, 30 , 30 );
  View << p10;

  /*
  View << DGtal::ViewerClippingPlane ( 1, 0, 0, -4.9 );
  View << DGtal::ViewerClippingPlane ( 0, 1, 0.3, -10 );
  View << DGtal::ViewerCameraPosition ( 2.500000, 2.500000, 16.078199 )
  << DGtal::ViewerCameraDirection ( 0.000000, 0.000000, -1.000000 )
  << DGtal::ViewerCameraUpVector ( 0.000000, 1.000000, 0.000000 );
  View << DGtal::ViewerCameraZNearFar ( 0.1, 200 );
  */
  DGtal::Z3i::Point p1 ( 0, 0, 0 );
  DGtal::Z3i::Point p2 ( 0, 0 , 0 );


  /*
  cout<<" Begin Scene display "<<endl;
  View.sceneDisplay();
  cout<<" End scene display "<<endl;
  */


  View >> p1;

  View >> p2;

  shape_set1.erase ( p1 );
  shape_set1.erase ( p2 );
  View.clearScene();

// On met le nouvel objet
  View << shape_set1;
  View.start();

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing adding DGtalSet  for ViewerOgre3D" );
  bool res = testSelection(); 
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
