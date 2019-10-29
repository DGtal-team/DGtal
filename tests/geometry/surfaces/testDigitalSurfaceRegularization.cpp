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
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/geometry/surfaces/DigitalSurfaceRegularization.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalSurfaceRegularization.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing DigitalSurfaceRegularization" )
{
  typedef Shortcuts<Z3i::KSpace> SH3;
  typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
  auto params = SH3::defaultParameters()
              | SHG3::defaultParameters();
  
  params( "polynomial", "goursat" )( "gridstep", 0.5 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto surfels         = SH3::getSurfelRange( surface, params );
  
  SECTION("Basic Construction using Trivial Normals")
  {
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
    regul.init();
    regul.attachTrivialNormalVectors();
    regul.enableVerbose();
    double energy = regul.computeGradient();
    CAPTURE( energy );
    REQUIRE( energy == Approx(6239.7));
    CAPTURE( regul );
    regul.regularize();
    REQUIRE( regul.isValid() );

    auto regularizedPosition = regul.getRegularizedPositions();
    auto original = regul.getOriginalPositions();
    auto normals = regul.getNormalVectors();
    auto cellIndex = regul.getCellIndex();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return original[ cellIndex[c]];},
                       normals, SH3::Colors(), "originalSurf.obj");
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                       normals, SH3::Colors(), "regularizedSurf.obj");
  }
  
  SECTION("Testing another feature of DigitalSurfaceRegularization")
  {
  }
  
}

/** @ingroup Tests **/
