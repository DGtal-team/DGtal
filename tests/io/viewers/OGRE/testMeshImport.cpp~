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
* @file testMeshImport.cpp
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
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ViewerOgre3D.
///////////////////////////////////////////////////////////////////////////////
/**
* Example of a test. To be completed.
*
*/
bool testMeshImport()
{  
  new ViewerOgre3D();
  DGtal::ViewerOgre3D & View = DGtal::ViewerOgre3D::getSingleton();

  MeshFromPoints<Point> aMesh(true);
  aMesh.addVertex(Point(0,0,0));
  aMesh.addVertex(Point(1,0,0));
  aMesh.addVertex(Point(1,1,0));

  aMesh.addVertex(Point(0,0,1));
  aMesh.addVertex(Point(1,0,1));
  aMesh.addVertex(Point(1,1,1));
  aMesh.addVertex(Point(0,1,1));

  aMesh.addVertex(Point(0,1,0));
  aMesh.addVertex(Point(0,2,0));
  aMesh.addVertex(Point(0,3,1));
  aMesh.addVertex(Point(0,2,2));
  aMesh.addVertex(Point(0,1,2));
  aMesh.addVertex(Point(0,0,1));
  
  aMesh.addTriangularFace(0, 1, 2, Color(150,0,150,104));
  aMesh.addQuadFace(6,5,4,3, Color::Blue);
  
  vector<unsigned int> listIndex;
  listIndex.push_back(7);
  listIndex.push_back(8);
  listIndex.push_back(9);
  listIndex.push_back(10);
  listIndex.push_back(11);  
  listIndex.push_back(12);  

  aMesh.addFace(listIndex, Color(150,150,0,54));
  View << aMesh;



  View.start();
  

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing adding DGtalSet  for ViewerOgre3D" );
  bool res = testMeshImport(); 
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
