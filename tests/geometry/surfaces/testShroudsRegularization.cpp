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
 * @file testShroudsRegularization.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/06/11
 *
 * Functions for testing class ShroudsRegularization
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
#include "DGtal/geometry/surfaces/ShroudsRegularization.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ShroudsRegularization
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing ShroudsRegularization" )
{
  //! [ShroudsRegInit]
  typedef Shortcuts<Z3i::KSpace>         SH3;
  typedef SH3::ExplicitSurfaceContainer  Container;
  typedef ShroudsRegularization< Container >::Regularization RegType;

  auto params = SH3::defaultParameters();
  params( "polynomial", "goursat" )( "gridstep", 1)("verbose", 0);
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
  //! [ShroudsRegInit]

  //! [ShroudsRegUsage]
  auto idxsurface     = SH3::makeIdxDigitalSurface( surface, params );
  ShroudsRegularization< Container > shrouds_reg( idxsurface );
  auto originalPos    = shrouds_reg.positions();
  //! [ShroudsRegUsage]
  {
    auto polySurf = SH3::makeDualPolygonalSurface( idxsurface );
    SH3::saveOBJ( polySurf, "goursat-shrouds-init.obj" );
  }

  //! [ShroudsRegK2]
  double          loo = 0.0;
  double           l2 = 0.0;
  double energyInitK2 = shrouds_reg.energy    ( RegType::SQUARED_CURVATURE );
  std::tie( loo, l2 ) = shrouds_reg.regularize( RegType::SQUARED_CURVATURE,
						0.5, 0.0001, 100 );
  double energyRegK2  = shrouds_reg.energy    ( RegType::SQUARED_CURVATURE );
  //! [ShroudsRegK2]

  REQUIRE( loo < 0.1 );
  REQUIRE( l2 <= loo );
  REQUIRE( energyRegK2 < energyInitK2 );

  {
    //! [ShroudsRegSaveObj]
    auto regularizedPos = shrouds_reg.positions();
    auto polySurf       = SH3::makeDualPolygonalSurface( idxsurface );
    auto polySurfPos    = polySurf->positions();
    for ( size_t i = 0; i < regularizedPos.size(); i++ )
      polySurfPos[ i ] = regularizedPos[ i ];
    SH3::saveOBJ( polySurf, "goursat-shrouds-reg-k2.obj" );
    //! [ShroudsRegSaveObj]
  }

  //! [ShroudsRegArea]
  shrouds_reg.init();
  double energyInitArea= shrouds_reg.energy    ( RegType::AREA );
  std::tie( loo, l2 )  = shrouds_reg.regularize( RegType::AREA,
						 0.5, 0.0001, 100 );
  double energyRegArea = shrouds_reg.energy    ( RegType::AREA );
  //! [ShroudsRegArea]

  REQUIRE( energyRegArea < energyInitArea );

  //! [ShroudsRegSnake]
  shrouds_reg.init();
  double energyInitSnk= shrouds_reg.energy    ( RegType::SNAKE );
  std::tie( loo, l2 ) = shrouds_reg.regularize( RegType::SNAKE,
						0.5, 0.0001, 100 );
  double energyRegSnk = shrouds_reg.energy    ( RegType::SNAKE );
  //! [ShroudsRegSnake]

  REQUIRE( energyRegSnk < energyInitSnk );
}
