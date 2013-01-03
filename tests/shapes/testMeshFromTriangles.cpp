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


  MeshFromTriangles<Point> aMesh;
  Point p0=Point(0,0);
  Point p1=Point(0,1);
  Point p2=Point(1,2);

  Point p3=Point(3,2);
  Point p4=Point(3,3);
  Point p5=Point(4,4);

  
 

  // std::vector<Point> vectTrianglePoints = aMesh.getTrianglesFromVertex();
MeshFromTriangles<Point> aMesh2 (Point (0,0), Point(10,10));
aMesh2.addPointInMesh(Point(8,8));




aMesh2.addPointInMesh(Point(8,9));
aMesh2.addPointInMesh(Point(5,9));
DGtal::MeshFromTriangles<Point>::IndexOfCreatedTriangle indexNew = aMesh2.addPointInMesh(Point(3,4));
 aMesh2.addPointInMesh(Point(5,2));
aMesh2.addPointInMesh(Point(6,6));





//  DGtal::MeshFromTriangles<Point>::IndexOfCreatedTriangle indexNew =  aMesh2.addPointInMesh(Point(4,4));

Point adjPt1 = aMesh2.getAdjacentVertex(indexNew.indexTr3, 1);

std::vector<Point>  tr1 =  aMesh2.getTrianglePoints(indexNew.indexTr3);

std::vector<Point>  tr2 =  aMesh2.getTrianglePoints(aMesh2.getIndexAdjacentTriangle(indexNew.indexTr3, 1));




//std::vector<Point>  tr1 =  aMesh2.getTrianglePoints(11);
//std::vector<Point>  tr2 =  aMesh2.getTrianglePoints(aMesh2.getIndexAdjacentTriangle(9,2));


//std::vector<Point>  tr1 =  aMesh2.getTrianglePointsAdj(indexNew.indexTr3, 1);





Board2D aBoardTrans; 

Board2D aBoard; 
  aBoard  << aMesh2 ;
  aBoard.setPenColor(DGtal::Color(200,20,20));


  aBoard.setPenColor(DGtal::Color(200,200,20));
  aBoard.fillTriangle(LibBoard::Point(tr1.at(0)[0],tr1.at(0)[1]), 
			LibBoard::Point(tr1.at(1)[0],tr1.at(1)[1]),
			LibBoard::Point(tr1.at(2)[0],tr1.at(2)[1]));
  aBoard.setPenColor(DGtal::Color(100,100,100));
  aBoard.fillTriangle(LibBoard::Point(tr2.at(0)[0],tr2.at(0)[1]), 
			LibBoard::Point(tr2.at(1)[0],tr2.at(1)[1]),
			LibBoard::Point(tr2.at(2)[0],tr2.at(2)[1]));
  aBoard.setLineWidth(2);
  aBoard.setPenColor(DGtal::Color(20,200,20));
  aBoard.drawLine(adjPt1[0], adjPt1[1],3, 4);  

  aBoard.saveEPS("displayMeshFromTriangles.eps");



  bool isIn = aMesh2.isInTriangle(1,p3); 
  bool isIn2 = aMesh2.isInTriangle(1,Point(-1,-1)); 

  trace.info() << "pt is in triangle 1:" << isIn<< endl;
  trace.info() << "pt -1,-1 is in triangle 2:" << isIn2<< endl;
  trace.info() << "index of triangle containing pt (8,8):" << aMesh2.getTriangleIndexInclosing(Point(8,8)) << endl;
  trace.info() << "index of triangle containing pt (1,1):" << aMesh2.getTriangleIndexInclosing(Point(1,1)) << endl;
  trace.info() << "is inCircle 6,6 :" << aMesh2.isInCircle(1,Point(15,10)) << endl;







 



  aMesh2.flipTriangleOnEdge(indexNew.indexTr3, 1);
  std::vector<Point>  tr1s =  aMesh2.getTrianglePoints(aMesh2.getIndexAdjacentTriangle(22, 3));
  std::vector<Point>  tr1sA =  aMesh2.getTrianglePoints(aMesh2.getIndexAdjacentTriangle(aMesh2.getIndexAdjacentTriangle(22, 3),3));

  aBoardTrans.setLineWidth(0.4);
  aBoardTrans.setPenColor(DGtal::Color(200,200,20));
  aBoardTrans.fillTriangle(LibBoard::Point(tr1s.at(0)[0],tr1s.at(0)[1]), 
		      LibBoard::Point(tr1s.at(1)[0],tr1s.at(1)[1]),
		      LibBoard::Point(tr1s.at(2)[0],tr1s.at(2)[1]));
  
  aBoardTrans.setPenColor(DGtal::Color(100,100,100));
  aBoardTrans.fillTriangle(LibBoard::Point(tr1sA.at(0)[0],tr1sA.at(0)[1]), 
		      LibBoard::Point(tr1sA.at(1)[0],tr1sA.at(1)[1]),
		      LibBoard::Point(tr1sA.at(2)[0],tr1sA.at(2)[1]));
  

  aBoardTrans << aMesh2;
  aBoardTrans.saveEPS("displayMeshFromTrianglesTrans.eps");


  // Point p0f0 = vectTrianglePoints.at(0);
  // Point p1f0 = vectTrianglePoints.at(1);
  // Point p2f0 = vectTrianglePoints.at(2);

  // Point p0f1 = vectTrianglePoints.at(3);
  // Point p1f1 = vectTrianglePoints.at(4);
  // Point p2f1 = vectTrianglePoints.at(5);
  
  // trace.info() << "Set of points" << endl;
  // trace.info() << p0 << p1 << p2 << endl;
  // trace.info() << p3 << p4 << p5 << endl;
  
  // trace.info() << "Face1 points " << endl;
  // trace.info() << p0f0 << p1f0 << p2f0<< endl;

  // trace.info() << "Face2 points " << endl;
  // trace.info() << p0f1 << p1f1 << p2f1<< endl;


  

  // return (p0==p0f0) && (p1==p1f0) && (p2==p2f0) && 
  //   (p3==p0f1) && (p4==p1f1) && (p5==p2f1) ;
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
