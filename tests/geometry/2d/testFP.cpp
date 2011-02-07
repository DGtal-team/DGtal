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
 * @file testFP.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/26
 *
 * Functions for testing class FP.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/FP.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FP.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testFP()
{

	typedef int Coordinate;
	typedef HyperRectDomain<SpaceND<2,Coordinate> > Domain;
	typedef PointVector<2,Coordinate> Point;
  typedef FreemanChain<Coordinate> Contour; 
	typedef FP<Contour::ConstIterator,Coordinate,4> FP;

/*
  // A Freeman chain code is a string composed by the coordinates of the first pixel, 
	//and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
//  ss << "1 11 0300303303033030303000010101011010110100000303303033030303000010101101010110100000333" << endl;
//  ss << "36 15 1121212121212212121212212122122222322323233323333333323333323303330330030300000100010010010001000101010101111" << endl;
//  ss << "165 78 112332323323332323332323223232322322323232300030003000300300300303003030330330330330300303003003030030300300303003030333333333332323222222322232323222223222222222" << endl;
  // Construct the Freeman chain
  Contour theContour( ss );

*/
  std::string filename = testPath + "samples/france.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  Contour theContour(fst);



  trace.beginBlock ( "FP of a 4-connected digital curve..." );

	FP theFP( theContour.begin(),theContour.end() );
  //trace.info() << theFP << std::endl;

	// Draw the FP
  Point p1( 0, 0 );
  Point p2( 48, 12 );
  Domain domain( p1, p2 );
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  //aBoard << SetMode( domain.styleName(), "Grid" ) << domain;
	aBoard << SetMode( "PointVector", "Grid" ) << theContour;
  aBoard << theFP;
  aBoard.saveSVG("FP4.svg");

  trace.endBlock();
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FP" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testFP(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
