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
 * @file Mesh3DConstructionAndVisualisation.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/01/10
 *
 * Example of MeshFromPoints construction and visualisation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [MeshFromPointsUseInclude]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include <QtGui/qapplication.h>
//! [MeshFromPointsUseInclude]
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;


int main( int argc, char** argv )
{

  //! [MeshFromPointsUseInitDisplay]
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();     
  //! [MeshFromPointsUseInitDisplay]


  //! [MeshFromPointsUseMeshCreation]
  // A mesh is constructed and faces are added from the vertex set. 
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
  //! [MeshFromPointsUseMeshCreation]
  //! [MeshFromPointsUseDisplay]
  viewer.setLineColor(Color(150,0,0,254));
  viewer << aMesh;
  viewer << Viewer3D::updateDisplay;
  bool res = application.exec();
  //! [MeshFromPointsUseDisplay]
  return true;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

