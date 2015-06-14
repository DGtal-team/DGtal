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
 * @file testPointHashFunctions.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2015/06/14
 *
 * Functions for testing class PointHashFunctions.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"

#define CATCH_CONFIG_MAIN 
#include "catch.hpp"

#include "DGtal/kernel/PointVector.h"
#incldue "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/PointHashFunctions.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;



TEST_CASE("Hash functions on DGtal::Point")
{
  Z3i::Point p(0,0,0);
  Z3i::Point q(10,0,0);
  Z3i::Point p_copy(0,0,0);

  boost::hash<Z3i::Point> myhash;

  #ifdef WITH_C11
  std::hash<Z3i::Point> myhashcpp11;
  #endif
  
  SECTION("Identity test")
    {
      REQUIRE( myhash(p) == myhash(p_copy) );
      CAPTURE( myhash(p) );
      
#ifdef WITH_C11
      REQUIRE( myhashcpp11(p) == myhashcpp11(p_copy) );
#endif
      
    }

  
  SECTION("Difference test")
    {
      REQUIRE( myhash(p) != myhash(q) );
      
#ifdef WITH_C11
      REQUIRE( myhashcpp11(p) != myhashcpp11(q) );
#endif
      
  
    }
  
}
