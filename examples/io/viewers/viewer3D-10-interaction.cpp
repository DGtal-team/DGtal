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
 * @file viewer3D-10-interaction.cpp
 * @ingroup examples/3dViewer
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/10/12
 *
 * Simple example of class Viewer3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/helpers/StdDefs.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

typedef Viewer3D<Space,KSpace> MyViewer;
typedef MyViewer::SelectCallbackFct SelectCallbackFct;
typedef KSpace::SCell SCell;

struct BigData
{
  KSpace K;
  std::map< int32_t, SCell > cells;
};

int reaction1( void* viewer, int32_t name, void* data )
{
  BigData* bg = (BigData*) data;
  stringstream ssMessage;
  ssMessage << "Reaction1 with name " << name << " cell " << bg->K.sKCoords( bg->cells[ name ] )  ;
  ((MyViewer *) viewer)->displayMessage(QString(ssMessage.str().c_str()), 100000);
  trace.info() <<  ssMessage.str() << std::endl;
  return 0;
}
int reaction23( void* viewer, int32_t name, void* data )
{
  BigData* bg = (BigData*) data;
  stringstream ssMessage;
  ssMessage <<  "Reaction23 with name " << name << " cell " << bg->K.sKCoords( bg->cells[ name ] );
  ((MyViewer *) viewer)->displayMessage(QString(ssMessage.str().c_str()), 100000);
  trace.info() << ssMessage.str() << std::endl;
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  QApplication application(argc,argv);
  BigData data;
  Point p1( 0, 0, 0 );
  Point p2( 5, 5 ,5 );
  Point p3( 2, 3, 4 );
  KSpace & K = data.K;
  K.init( p1, p2, true );

  MyViewer viewer( K );
  viewer.show();
  viewer.displayMessage(QString("You can use [shift + click right] on surfels to interact ..."), 100000);
  SCell surfel1 = K.sCell( Point( 1, 1, 2 ), KSpace::POS );
  SCell surfel2 = K.sCell( Point( 3, 3, 4 ), KSpace::NEG );
  SCell surfel3 = K.sCell( Point( 5, 6, 5 ), KSpace::POS );
  data.cells[ 10001 ] = surfel1;
  data.cells[ 10002 ] = surfel2;
  data.cells[ 10003 ] = surfel3;
  viewer << SetMode3D( surfel1.className(), "Basic" );
  viewer << SetName3D( 10001 ) << CustomColors3D( Color::Red, Color::Red ) << surfel1;
  viewer << SetName3D( 10002 ) << CustomColors3D( Color::Green, Color::Green ) << surfel2;
  viewer << SetName3D( 10003 ) << CustomColors3D( Color::Blue, Color::Blue ) << surfel3;
  viewer << SetSelectCallback3D( reaction1,  &data, 10001, 10001 );
  viewer << SetSelectCallback3D( reaction23, &data, 10002, 10003 );
  viewer<< MyViewer::updateDisplay;
  return application.exec();
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
