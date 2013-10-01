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
 * @file testCOBAStandardPlaneComputer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class COBAStandardPlaneComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/geometry/surfaces/CAdditivePrimitiveComputer.h"
#include "DGtal/geometry/surfaces/COBAGenericStandardPlaneComputer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class COBAGenericStandardPlaneComputer.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int /*argc*/, char** /*argv*/ )
{
  using namespace Z3i;

  typedef COBAGenericStandardPlaneComputer<Space, int64_t> PlaneComputer;
  
  bool ok;
  PlaneComputer plane;
  plane.init( 100, 1, 1 );
  ok = plane.extend( Point(0,0,0) );
  trace.info() << "Point(0,0,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(1,0,0) );
  trace.info() << "Point(1,0,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(0,1,0) );
  trace.info() << "Point(0,1,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(1,1,0) );
  trace.info() << "Point(1,1,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(2,0,0) );
  trace.info() << "Point(2,0,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(0,2,0) );
  trace.info() << "Point(0,2,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(0,2,0) );
  trace.info() << "Point(0,2,0) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  ok = plane.extend( Point(1,1,1) );
  trace.info() << "Point(1,1,1) is " << ( ok ? "ok" : "ko" ) << std::endl;
  trace.info() << plane << std::endl;
  return 0;
}
