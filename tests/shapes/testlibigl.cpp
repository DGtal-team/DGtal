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
 * @file testlibigl.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2023/06/14
 *
 * Functions for testing class PolygonalCalculus.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"

//Removing Warnings from libIGL for gcc and clang
#pragma GCC system_header  // For GCC
#pragma clang system_header  // For Clang

#include <igl/readOBJ.h>

#pragma GCC diagnostic pop  // For GCC
#pragma clang diagnostic pop  // For Clang


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing libIGL io.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing LibIGL" )
{
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  
  SECTION("Simple test with OBJ IO")
    {
      igl::readOBJ(testPath + "samples/testObj.obj", V, F);
      REQUIRE( V.rows() == 10);
      REQUIRE( F.rows() == 6);
    }
};

/** @ingroup Tests **/
