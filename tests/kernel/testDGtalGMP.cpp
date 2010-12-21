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
 * @file testDGtalGMP.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/12/13
 *
 * Functions for testing class DGtalGMP.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/GreedyDecomposition.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include <gmpxx.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DGtalGMP.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDGtalGMP()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "GMP linking test..." );
 
  mpz_class a, b, c;
  
  a = 1234;
  b = "-5678";
  c = a+b;
  trace.info() << "sum is " << c << "\n";
  trace.info() << "absolute value is " << abs(c) << "\n";

  nbok += (abs(c)==4444) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}


/**
 * Example of a test. To be completed.
 *
 */
bool testGMPSpace()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "GMP Space test..." );
   
  typedef SpaceND<2, mpz_class> Space2Type;
  typedef Space2Type::Point Point;
  typedef Space2Type::Point::Coordinate Coordinate;
  typedef HyperRectDomain<Space2Type> Domain;

  typedef ArithmeticalDSS<StandardBase<Coordinate> > DSS4;  
  typedef FreemanChain<Coordinate> ContourType; 
  typedef GreedyDecomposition< ContourType, DSS4 > Decomposition;
  
  // Construct the Freeman chain
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 11121212121212212121212212122122222322323233323333333323333323303330330030300000100010010010001000101010101111" << endl;
  ContourType theContour( ss );
  //Segmentation
  Decomposition theDecomposition( theContour );
  Point p1( 0, 0 );
  Point p2( 31, 31 );
  Domain domain( p1, p2 );
  DGtalBoard aBoard;
  aBoard << SetMode( domain.styleName(), "Grid" )
	 << domain
	 << theContour;

  aBoard.saveSVG("testgmpcontour.svg");
  

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
  trace.beginBlock ( "Testing class DGtalGMP" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDGtalGMP() && testGMPSpace(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
