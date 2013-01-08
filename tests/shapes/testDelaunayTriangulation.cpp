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
 * @file test3DMeshFromTriangles.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/07/02
 *
 * Functions for testing class 3DMeshFromTriangles.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/fromPoints/MeshFromTriangles.h"

#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;



///////////////////////////////////////////////////////////////////////////////
// Functions for testing class MeshFromTriangles.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testMeshFromTriangles()
{
  
  trace.beginBlock ( "Testing MeshFromTriangles ..." );


MeshFromTriangles<Point> aMesh(Point(0,0),Point(10,10));








aMesh.addPointInMesh(Point(2,4)); 
aMesh.addPointInMesh(Point(3,2));
aMesh.addPointInMesh(Point(7,2));
aMesh.addPointInMesh(Point(7,8));



Board2D aBoard; 
aBoard  << aMesh ;

Board2D aBoard2; 




  aBoard.saveEPS("displayDelaunayTest.eps");
  aBoard.saveFIG("displayDelaunayTest.fig");

int s=aMesh.getNumTriangles();

trace.info() << "num triangles =" << s << std::endl;



for(unsigned int i =3; i<s; i++){
trace.info() << "flipping i=" << i << std::endl;

aMesh.flipTriangleOnEdge(i, 1);
//aMesh.flipTriangleOnEdge(i, 2);
     }


aBoard2  << aMesh ;
 aBoard2.saveEPS("displayDelaunayTestTR.eps");
  aBoard2.saveFIG("displayDelaunayTestTR.fig");

return true;
}









///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class MeshFromPoints" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testMeshFromTriangles(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
