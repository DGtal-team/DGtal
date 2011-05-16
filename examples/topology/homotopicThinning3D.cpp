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
 * @file qglViewer.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/04
 *
 * An example file named qglViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include <QImageReader>
#include <QtGui/qapplication.h>
#include "DGtal/io-viewers/3dViewers/DGtalQGLViewer.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////



int main( int argc, char** argv )
{

  trace.beginBlock ( "Example simple example of DGtalQGLviewer" );
  
  QApplication application(argc,argv);
  DGtalQGLViewer viewer;
  viewer.setWindowTitle("simpleExampleDGtalQGLviewer");
  viewer.show();  
  
  // Domain cretation from two bounding points.
  Point c( 0, 0, 0 );
  Point p1( -50, -50, -50 );
  Point p2( 50, 50, 50 );
  Domain domain( p1, p2 );
  
  trace.warning() << "Constructing a ring DigitalSet  ... ";
  DigitalSet shape_set( domain );
  for ( Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it )
    {
      if ( ((*it - c ).norm() <= 25) && ((*it - c ).norm() >= 18)
	   && ( (((*it)[0] <= 3)&& ((*it)[0] >= -3))|| (((*it)[1] <= 3)&& ((*it)[1] >= -3)))){
	shape_set.insertNew( *it );
      }
    }
  trace.warning() << "  [Done]";
  

  Object6_26 shape( dt6_26, shape_set );
  int nb_simple=0; 
  int layer = 0;
  do 
    {
      DigitalSet & S = shape.pointSet();
      std::queue<DigitalSet::Iterator> Q;
      for ( DigitalSet::Iterator it = S.begin(); it != S.end(); ++it )
	if ( shape.isSimple( *it ) )
	  Q.push( it );
      nb_simple = 0;
      while ( ! Q.empty() )
	{
	  DigitalSet::Iterator it = Q.front();
	  Q.pop();
	  if ( shape.isSimple( *it ) )
	    {
	      cerr << "point simple " << (*it) << endl; 
	      S.erase( *it );
	      ++nb_simple;
	    }
	}
      ++layer;
    }
  while ( nb_simple != 0 );

  DigitalSet & S = shape.pointSet();

  // Display by using two different list to manage OpenGL transparency.

  viewer << SetMode3D( shape_set.styleName(), "Paving" );
  viewer << CustomColors3D(QColor(25,25,255, 255), QColor(25,25,255, 255));
  viewer << S ; 

  viewer << SetMode3D( shape_set.styleName(), "PavingTransp" );
  viewer << CustomColors3D(QColor(250, 0,0, 25), QColor(250, 0,0, 5));
  viewer << shape_set;

  viewer<< DGtalQGLViewer::updateDisplay;
   
  
  trace.endBlock();
  return application.exec();

}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////


