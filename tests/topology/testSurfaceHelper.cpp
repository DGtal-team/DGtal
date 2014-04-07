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
 * @file testSurfaceHelper.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2014/04/07
 *
 * Functions for testing class SurfaceHelper.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/io/boards/Board2D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SurfaceHelper.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */


bool testComputeInterior()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef  KhalimskySpaceND<2, int>::Cell Cell;
  typedef  KhalimskySpaceND<2, int>::SCell SCell;

  trace.beginBlock ( "Testing computation of interior from a freeman chain ..." );
  std::string freemanChainFilename = testPath + "samples/contourS.fc";
  fstream fst;
  fst.open (freemanChainFilename.c_str(), ios::in);
  FreemanChain<Z2i::Space::Integer> fc(fst);
  fst.close();
  std::set<SCell> boundaryCell;
  DGtal::KhalimskySpaceND< 2, int > K; 
  int minx, miny, maxx, maxy; 
  fc.computeBoundingBox(minx, miny, maxx, maxy); 
  K.init(Z2i::Point(minx-5,miny-5), Z2i::Point(maxx+5, maxy+5), false);
  FreemanChain<Z2i::Space::Integer>::getContourSCell(K, fc, boundaryCell ); 
  std::set<Cell> interiorCell;
  Surfaces<DGtal::KhalimskySpaceND< 2, int > >::uComputeInterior(K, boundaryCell, interiorCell, false);
  trace.info() << "Interior size: " << interiorCell.size() << " (awaited:  3014)" <<  std::endl;
  
  // Displaying interiorCell
  Board2D aBoard;  

  for( std::set<Cell>::const_iterator it=interiorCell.begin();  it!= interiorCell.end(); it++){
    aBoard << K.uCoords (*it); 
  }

  aBoard<< CustomStyle( (*(boundaryCell.begin())).className(), 
                        new CustomColors(  Color::Red,  Color::None ) );          
  for( std::set<SCell>::const_iterator it= boundaryCell.begin();  it!= boundaryCell.end(); it++){
    aBoard << *it;
  }
  aBoard<< CustomStyle( fc.className(), 
                        new CustomColors(  Color::Red,  Color::None ) );        
  aBoard << SetMode( fc.className(), "InterGrid" );
  aBoard << fc;    
  
  aBoard.saveEPS("testSurfaceHelperComputeInterior.eps");
  nbok += interiorCell.size()== 3014 ? 1 : 0; 
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
  trace.beginBlock ( "Testing class SurfaceHelper" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testComputeInterior(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
