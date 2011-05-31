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
 * @file testDisplayKSCell.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/05/30
 *
 * Functions for testing class DisplayKSCell.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;



///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <QtGui/qapplication.h>
#include "DGtal/io-viewers/3dViewers/DGtalQGLViewer.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shapes.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing Display of display of KS space Cell.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDGtalQGLViewer()
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

 QApplication application(argc,argv);
 DGtalQGLViewer viewer;
 viewer.show();

 KSpace K;
 Point plow(0,0,0);  
 Point pup(3,3,2);
 Domain domain( plow, pup );
 K.init( plow, pup, true );
  
  
 //viewer << SetMode3D( domain.styleName(), "Paving" );
 // viewer << domain; 


 // Drawing cell of dimension 3
 Cell voxelA = K.uCell(Point(1,1,1));
 SCell voxelB = K.sCell(Point(1,1,3));
 viewer << voxelB<< voxelA;//
 
 // drawing cells of dimension 2
 SCell surfelA = K.sCell( Point( 2, 1, 3 ) ); 
 SCell surfelB = K.sCell( Point( 1, 0, 1 ), false ); 
 Cell surfelC = K.uCell( Point( 1, 2, 1 ) ); 
 SCell surfelD = K.sCell( Point( 1, 1, 0 ) );
 Cell surfelE = K.uCell( Point( 1, 1, 2 ) ); 
 viewer << surfelA << surfelB << surfelC << surfelD << surfelE;
 
 Cell linelA = K.uCell(Point(2,1 ,2));
 SCell linelB = K.sCell(Point(2,2 ,1));
 SCell linelC = K.sCell(Point(1,2 ,2), false);
 viewer << linelA << linelB << linelC;
 
 viewer <<  DGtalQGLViewer::updateDisplay;
 application.exec();

 
 trace.endBlock();
 return true;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
