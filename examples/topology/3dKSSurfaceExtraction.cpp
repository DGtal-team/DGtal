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
 * @file 3dKSSurfaceExtraction.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/06/27
 *
 * An example file named 3dKSSurfaceExtraction.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include <QtGui/qapplication.h>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/3dViewers/DGtalQGLViewer.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/helpers/Surfaces.h"

#include "DGtal/topology/KhalimskySpaceND.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  
  Point p1( -10, -10, -10 );
  Point p2( 10, 10, 10 );
  Point c( 0, 0, 0 );
  Domain domain( p1, p2 );
  
  DigitalSet diamond_set( domain );
  
  for ( Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it ){
    if ( (*it - c  ).norm1() <= 6 && (*it - c  ).norm1() >= 4 ) 
      diamond_set.insertNew( *it );
    if ( (*it - c +Point(7,7,7) ).norm1() <= 2 ) 
      diamond_set.insertNew( *it );
    if ( (*it - c +Point(-5,-5,-5) ).norm1() <= 7 ) 
      diamond_set.insertNew( *it );

  }
  
  KSpace K;
  K.init(p1, p2, true);
  
    
  SurfelAdjacency<3> SAdj( true );
  vector<vector<SCell> > vectConnectedSCell;
  Surfaces<KSpace>::extractAllConnectedSCell(vectConnectedSCell,K, SAdj, diamond_set, true);
  
  QApplication application(argc,argv);
  DGtalQGLViewer viewer;
  viewer.show(); 
   
  //viewer << SetMode3D( vectConnectedSCell.at(0).at(0).styleName(), "Basic" );
  for(uint i=0; i< vectConnectedSCell.size();i++){
    switch (i){
    case 0:
      viewer << CustomColors3D(QColor(250, 0,0),QColor(250, 200,100));
      break;
    case 1:
      viewer << CustomColors3D(QColor(250, 0,0),QColor(250, 20,100));
      break;
    case 2:
      viewer << CustomColors3D(QColor(250, 0,0),QColor(20, 200,100));
      break;
    case 3:
      viewer << CustomColors3D(QColor(250, 0,0),QColor(20, 20,200));
      break;
    }
    for(uint j=0; j< vectConnectedSCell.at(i).size();j++){
      viewer << vectConnectedSCell.at(i).at(j);
    }    
  }

  
  viewer << CustomColors3D(QColor(250, 0,0),QColor(250, 200,200, 250));
  viewer << diamond_set;
  //viewer << ClippingPlane(0,1,0.0,-2);
  viewer << DGtalQGLViewer::updateDisplay;
  trace.endBlock();
  return application.exec();
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
