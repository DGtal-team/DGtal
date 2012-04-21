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
 * @file testConvexIntegerPolygon.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/19
 *
 * Functions for testing class ConvexIntegerPolygon.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/ConvexIntegerPolygon.h"
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConvexIntegerPolygon.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename Space>
bool testConvexIntegerPolygon()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ConvexIntegerPolygon area and centroid" );
  typedef typename Space::Point Point;
  typedef typename Space::Integer Integer;
  typedef ConvexIntegerPolygon<Space> CIP;
  typedef typename CIP::Point3I Point3I;
  typedef typename CIP::Domain Domain;
  CIP cip;
  cip.push_back( Point( 0, 0 ) );
  cip.push_back( Point( 5, 0 ) );
  cip.push_back( Point( 0, 3 ) );
  Integer area2 = cip.twiceArea();
  trace.info() << "- 2*area   = " << area2 << std::endl;
  ++nb, nbok += ( area2 == 15 ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "2*area == 15" << std::endl;
  Point3I c = cip.centroid( area2 );
  trace.info() << "- centroid = " << c << std::endl;
  ++nb, nbok += ( c == Point3I( 75, 45, 45 ) ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "centroid == [75,45,45]" << std::endl;
  Domain d = cip.boundingBoxDomain();
  trace.info() << "- domain = " << d << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Output ConvexIntegerPolygon in <cip.eps>" );
  Board2D board;
  board << SetMode( d.className(), "Grid" ) << d;
  board << SetMode( cip.className(), "Transparent" ) << cip;
  board.saveEPS( "cip.eps" );
  board.saveSVG( "cip.svg" );
  trace.endBlock();

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int, char** )
{
  trace.beginBlock ( "Testing class ConvexIntegerPolygon" );

  typedef SpaceND<2, DGtal::int64_t> Z2;
  typedef SpaceND<2, DGtal::BigInteger> Z2I;
  bool res = testConvexIntegerPolygon<Z2>()
    && testConvexIntegerPolygon<Z2I>();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
