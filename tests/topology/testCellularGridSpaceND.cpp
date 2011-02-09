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
 * @file testCellularGridSpaceND.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/02/08
 *
 * Functions for testing class CellularGridSpaceND.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/KhalimskySpaceND.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellularGridSpaceND.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename KSpace>
bool testCellularGridSpaceND()
{
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::SCell SCell;
  typedef typename KSpace::Point Point;

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );

  KSpace K;
  Point low = { -4, -4, 7 };
  Point high = { 50, 40, 30 };
  bool space_ok = K.init( low, high, false );
  nbok += space_ok ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "K.init( low, high )" << std::endl;
  trace.info() << "K.dim()=" << K.dimension() << endl;
  trace.info() << "K.size( 0 )=" << K.size( 0 ) << endl;
  trace.info() << "K.size( 1 )=" << K.size( 1 ) << endl;
  Point kp = { 1, 1, 1 }; // pixel
  Cell c1 = K.uCell( kp );
  Cell clow = K.uCell( low, kp );
  Cell chigh = K.uCell( high, kp );
  trace.info() << c1 << clow << chigh << endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CellularGridSpaceND" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  typedef KhalimskySpaceND<2> K2;
  typedef KhalimskySpaceND<3> K3;
  bool res = testCellularGridSpaceND<K2>()
   && testCellularGridSpaceND<K3>();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
