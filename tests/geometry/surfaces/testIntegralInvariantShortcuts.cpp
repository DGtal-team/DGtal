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
 * @date 2020/03/14
 *
 * Functions for testing class IntegralInvariantShortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>

#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantShortcuts.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing IntegralInvariant Shortcuts API" )
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", "goursat" )( "gridstep", 1. );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto K               = SH3::getKSpace( params );
  auto embedder        = SH3::getCellEmbedder( K );
  auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
  auto surfels         = SH3::getSurfelRange( surface, params );

  trace.info() << "Nb surfels= " << surfels.size() << std::endl;

  //Computing some differential quantities
  params("r-radius", 3.0);

  //We compute the curvature tensor, the mean and the Gaussian curvature
  auto Tcurv     = SHG3::getIIPrincipalCurvaturesAndDirections(binary_image, surfels, params);
  auto Kcurv     = SHG3::getIIGaussianCurvatures( binary_image, surfels, params);

  std::vector<double> k1,k2,G;
  for(auto &result: Tcurv)
  {
    k1.push_back( std::get<0>(result) );
    k2.push_back( std::get<1>(result) );
    G.push_back( ( std::get<0>(result) * std::get<1>(result)) );
  }

  SECTION("Testing that mean/Gaussian/tensor curvature shortucut values match")
  {
    for(std::size_t i = 0; i < G.size(); ++i)
     REQUIRE( Kcurv[i] == Approx( G[i] ) );
  }
}

/** @ingroup Tests **/
