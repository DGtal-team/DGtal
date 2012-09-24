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
 * @file testCOBANaivePlane.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class COBANaivePlane.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/arithmetic/COBANaivePlane.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class COBANaivePlane.
///////////////////////////////////////////////////////////////////////////////

/**
 * Example of a test. To be completed.
 *
 */
bool testCOBANaivePlane()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  using namespace Z3i;
  typedef BigInteger Integer;
  trace.beginBlock ( "Testing block: COBANaivePlane instantiation." );
  COBANaivePlane<Z3, BigInteger> plane;
  Point pt0( 0, 0, 0 );
  plane.init( 2, 100, pt0, 3, 2 );
  trace.info() << "(" << nbok << "/" << nb << ") Plane=" << plane
               << std::endl;
  Point pt1( Point( 8, 1, 3 ) );
  bool pt1_inside = plane.extend( pt1 );
  ++nb, nbok += pt1_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt1 
               << " Plane=" << plane << std::endl;
  Point pt2( Point( 2, 7, 1 ) );
  bool pt2_inside = plane.extend( pt2 );
  ++nb, nbok += pt2_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt2 
               << " Plane=" << plane << std::endl;

  Point pt3( Point( 0, 5, 17 ) );
  bool pt3_inside = plane.extend( pt3 );
  ++nb, nbok += pt3_inside == false ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt3
               << " Plane=" << plane << std::endl;

  Point pt4( Point( -10, -10, 10 ) );
  bool pt4_inside = plane.extend( pt4 );
  ++nb, nbok += pt4_inside == false ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt4
               << " Plane=" << plane << std::endl;

  Point pt5 = pt0 + pt1 + pt2 + Point( 0, 0, 2 );
  bool pt5_inside = plane.extend( pt5 );
  ++nb, nbok += pt5_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt5
               << " Plane=" << plane << std::endl;

  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int /*argc*/, char** /*argv*/ )
{
  trace.beginBlock ( "Testing class COBANaivePlane" );
  bool res = testCOBANaivePlane(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
