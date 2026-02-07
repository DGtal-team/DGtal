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
 * @file testParameters.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/05/14
 *
 * Functions for testing class CubicalComplex.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/Parameters.h"

#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Parameters
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "Parameters decimal conversion tests", "[parameters]" )
{
  GIVEN( "A Parameters object" ) {
    Parameters params;
    WHEN( "initialized with strings" ) {
      params( "foo", "bar" )( "Laurel", "Hardy" );
      THEN( "it does store strings" ) {
        REQUIRE( params[ "foo"    ].as<string>() == "bar" );
        REQUIRE( params[ "Laurel" ].as<string>() == "Hardy" );
      }
    }
    WHEN( "initialized with integers" ) {
      params( "prime", 7 )( "negative-int", -2 );
      THEN( "it does store ints" ) {
        REQUIRE( params[ "prime"        ].as<int>() == 7 );
        REQUIRE( params[ "negative-int" ].as<int>() == -2 );
      }
    }
    WHEN( "initialized with doubles" ) {
      params( "pi", 3.14159 )( "planck", 6.62607004e-34 )( "g", 9.80665 );
      THEN( "it does store ints" ) {
        REQUIRE( params[ "pi"     ].as<double>() == Approx( 3.14159 ) );
        REQUIRE( params[ "planck" ].as<double>() == Approx( 6.62607004e-34 ) );
        REQUIRE( params[ "g"      ].as<double>() == Approx( 9.80665 ) );
      }
    }
  }
}
