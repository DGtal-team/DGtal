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
 * @file testMeshFromPointsDisplay.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/07/07
 *
 * Functions for testing class MeshFromPointsDisplay.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
#include <QtGui/qapplication.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class MeshFromPointsDisplay.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :


int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class MeshFromPointsDisplay" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();     

  MeshFromPoints<Point> aMesh;

  aMesh.addVertex(Point(0,0,0));
  aMesh.addVertex(Point(1,0,0));
  aMesh.addVertex(Point(1,1,0));

  aMesh.addVertex(Point(0,0,1));
  aMesh.addVertex(Point(1,0,1));
  aMesh.addVertex(Point(1,1,1));
  aMesh.addVertex(Point(0,1,1));


  aMesh.addVertex(Point(0,0,0));
  aMesh.addVertex(Point(0,2,0));
  aMesh.addVertex(Point(0,2,1));
  aMesh.addVertex(Point(0,0.5,1));
  aMesh.addVertex(Point(0,0.0,1));


  aMesh.addTriangularFace(0, 1, 2);
  aMesh.addQuadFace(3, 4, 5, 6);
  vector<unsigned int> listIndex;
  listIndex.push_back(7);
  listIndex.push_back(8);
  listIndex.push_back(9);
  listIndex.push_back(10);
  listIndex.push_back(11);
  


  aMesh.addFace(listIndex);
  
  viewer.setFillColor(DGtal::Color(240,240,240,150));
  viewer.setLineColor(DGtal::Color(150,0,0,254));
  viewer << aMesh;
  viewer << Viewer3D::updateDisplay;

  bool res = application.exec();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
