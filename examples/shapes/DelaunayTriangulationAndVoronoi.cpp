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
 * @file DelaunayTriangulationAndVoronoi.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/01/13
 *
 * Example of MeshFromPoints construction and visualisation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [TriangularMeshFrom2DPointsINC]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/fromPoints/TriangularMeshFrom2DPoints.h"
#include "DGtal/io/boards/Board2D.h"

//! [TriangularMeshFrom2DPointsINC]
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;


int main( int argc, char** argv )
{

  //! [TriangularMeshFrom2DPointsINIT] 
  
  // The flag of the last argument is set to true to save the map
  // associating a Point to a Triangle (needed for the voronoi diagram)
  
  TriangularMeshFrom2DPoints<Point> aMesh (Point (0,0), Point(100,100), true);  
  
  //! [TriangularMeshFrom2DPointsINIT] 

    
  srand ( time(NULL) );
  for (unsigned int i =0; i < 1000 ; i++){

    unsigned int x = 10+rand()%80;
    unsigned int y = 10+rand()%80;
    Point p(x,y);
    double norme = sqrt((x-50)*(x-50)+(y-50)*(y-50));
  
    //! [TriangularMeshFrom2DPointsADDPOINT] 
    aMesh.addPointInsideDelaunayMesh(p);
    //! [TriangularMeshFrom2DPointsADDPOINT] 
      
    
  } 
  
  aMesh.removeTrianglesOfBoundingVertex();
  //! [TriangularMeshFrom2DPointsDISPLAYRES] 
  Board2D aBoard;
  aBoard << aMesh;    
  aBoard << Point(0,0) << Point(100,100);
  aBoard.saveEPS("exampleDelaunayTriangulation.eps");
  //! [TriangularMeshFrom2DPointsDISPLAYRES] 

  Board2D aBoard2;
  aBoard2.setPenColor(DGtal::Color(20,20,20));
  std::vector< std::vector<Point> > vectPolygons = aMesh.getVoronoiDiagram();
  for(unsigned int i=0;i<vectPolygons.size();  i++){
    std::vector< Point> aPolygon = vectPolygons.at(i);
    std::vector< LibBoard::Point > tmpCnt;
    for(unsigned int j=0; j< aPolygon.size(); j++){
      tmpCnt.push_back( LibBoard::Point(aPolygon.at(j)[0], aPolygon.at(j)[1]));
    }
    aBoard2.drawPolyline(tmpCnt);
  }

  aBoard2 << Point(0,0) << Point(100,100);
  aBoard2.saveEPS("exampleVoronoi.eps");
  trace.endBlock();
  return true;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

