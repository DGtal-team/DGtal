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
* @file testDGtalSet.cpp
* @ingroup Tests
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* INSA Lyon
*
* @date 2012/06/13
*
* Functions for testing class testDGtalSet.
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
bool testDGtalSet()
{  
  new ViewerOgre3D();
  DGtal::ViewerOgre3D & View = DGtal::ViewerOgre3D::getSingleton();

  DGtal::Z3i::Point p1 ( 0, 0, 0 );
  DGtal::Z3i::Point p2 ( 5, 5 , 5 );
  DGtal::Z3i::Point p3 ( 2, 3, 4 );

  DGtal::Z3i::Point p4 ( 30, 30 , 30 );
  DGtal::Z3i::Point p5 ( -30, -30 , -30 );
  DGtal::Z3i::Domain domain ( p4, p5 );
    

  View << DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),
				DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),
				DGtal::Color(255,0,0,100),"unselected.jpg");


  View << DGtal::SetViewerMode3D(p1.className(),"Paving");

  View << p1;
  View << p2;
  View << p3;

  DGtal::Z3i::DigitalSet shape_set1 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set1, DGtal::Z3i::Point ( 7, 7, 7 ), 4 );

  View << DGtal::SetViewerMode3D(shape_set1.className(),"Grid");

  View << DGtal::CustomViewerColors3D(DGtal::Color(255,0,0,100),
				DGtal::Color(255,0,0,100),DGtal::Color(255,0,0,100),
				DGtal::Color(255,0,0,100),
				"unselected.jpg");
  View << shape_set1;


  DGtal::Z3i::DigitalSet shape_set2 ( domain );
  DGtal::Shapes<DGtal::Z3i::Domain>::addNorm1Ball ( shape_set2, DGtal::Z3i::Point ( -10, -10, -10 ), 2 );
  View << shape_set2;
  View.start();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing adding DGtalSet  for ViewerOgre3D" );
  bool res = testDGtalSet(); 
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
