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
 * @file testCatch
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2015/08/06
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testCatch' <p>
 * Aim: simple test of \ref Catch unit test framework
 */

#include "DGtalCatch.h"

TEST_CASE( "Point Vector Unit tests" )
{
  
  int a = 5;
  int b = 3+2;
  int c = a+1;
  
  SECTION("Comparisons")
    {
      REQUIRE( (a == b) );
      a=6;
      REQUIRE( (a == c) );
    }
  
  SECTION("No side-effects on global variables in section scopes")
    {
      REQUIRE( (a == 5) );
    }

}


/** @ingroup Tests **/
