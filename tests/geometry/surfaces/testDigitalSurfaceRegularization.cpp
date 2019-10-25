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
 * @file
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2019/10/25
 *
 * Functions for testing class DigitalSurfaceRegularization.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/surfaces/DigitalSurfaceRegularization.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalSurfaceRegularization.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing DigitalSurfaceRegularization" )
{
  typedef LightImplicitDigitalSurface<> DigitalSurface
  
  SECTION("Basic Construction")
  {
    DigitalSurfaceRegularization<DigitalSurface> regul;
    CAPTURE( regul );
    REQUIRE( regul.isValid() );
  }
  
  SECTION("Testing another feature of DigitalSurfaceRegularization")
  {
  }
  
}

/** @ingroup Tests **/
