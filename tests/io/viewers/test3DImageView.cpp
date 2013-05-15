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
 * @file test3DImageView.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Functions for testing class Viewer3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <QtGui/qapplication.h>
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/math/BasicMathFunctions.h"
#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;



///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Viewer3D.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testViewer3D()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;  
  trace.beginBlock ( "Testing block ..." );
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::ImageContainerBySTLVector< DGtal::Z2i::Domain, unsigned char>  imageNG;

 QApplication application(argc,argv);
 Viewer3D viewer;
 viewer.setWindowTitle("simpleViewer");
 viewer.show();
 
 
 Point p1( 0, 0, 0 );
 Point p2( 125, 188, 0 );
 Point p3( 30, 30, 30 );
 
 std::string filename =  testPath + "samples/church-small.pgm";
 imageNG image = DGtal::PNMReader<imageNG>::importPGM(filename); 
 
 viewer << image;
 viewer << DGtal::UpdateImagePosition(0, Display3D::xDirection,  0, 0, 0 );
 viewer << SetMode3D( image.domain().className(), "BoundingBox" );
 viewer << image.domain();
 viewer << DGtal::Update2DDomainPosition(0, Display3D::xDirection, 0, 0, 0);
 for(unsigned int i= 0; i< 10; i++){
   if(i%4==0){
     viewer << SetMode3D( image.className(), "" );
   }else if(i%4==1){
     viewer << SetMode3D( image.className(), "BoundingBox" );
   }else if(i%4==2){
     viewer << SetMode3D( image.className(), "Grid" );
   }else if(i%4==3){
     viewer << SetMode3D( image.className(), "InterGrid" );
   }
   viewer << image; 
   viewer << DGtal::UpdateImageData<imageNG>(i+1, image,  i*50, i*50, i*50);
 }

 viewer << p1 << p2 << p3;
 viewer << Display3D::updateDisplay;


 bool res = application.exec();
 trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
 trace.endBlock();
 return res ? 0 : 1;


}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

