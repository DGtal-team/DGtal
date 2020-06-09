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
#include "DGtal/geometry/surfaces/ShroudsRegularization.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalSurfaceRegularization.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing ShroudsRegularization" )
{
  //! [ShroudsRegInit]
  typedef Shortcuts<Z3i::KSpace>         SH3;
  typedef SH3::ExplicitSurfaceContainer  Container;
  auto params = SH3::defaultParameters();
  
  params( "polynomial", "goursat" )( "gridstep", 0.3)("verbose", 0);
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  //! [ShroudsRegInit]
  
  //! [ShroudsRegUsage]
  auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
  auto idxsurface      = SH3::makeIdxDigitalSurface( surface, params );
  auto surfels         = SH3::getIdxSurfelRange( idxsurface, params );
  ShroudsRegularization< Container > shrouds_reg( idxsurface );
  auto originalPos     = shrouds_reg.positions();
  //! [ShroudsRegUsage]

  double loo      = 0.0;
  double  l2      = 0.0;
  typedef ShroudsRegularization< Container >::Regularization RegType;
  std::tie( loo, l2 ) = shrouds_reg.regularize( RegType::SQUARED_CURVATURE,
						0.5, 0.0001, 1000 );
  // const int maxNb = 100;
  // int     nb      = 0;
  // double   r      = 0.5;
  // do {
  //   std::tie( loo, l2 ) = shrouds_reg.oneStepSquaredCurvatureMinimization( r );
  //   trace.info() << "[Iteration " << nb << "] dx <= " << loo << " l2=" << l2 << endl;
  //   r  *= 0.9;
  //   nb += 1;
  // } while ( loo > 0.0001 && nb < maxNb );

  //! [ShroudsRegOutput]
  auto regularizedPos = shrouds_reg.positions();
  //! [ShroudsRegOutput]

  //! [ShroudsRegSaveObj]
  auto polySurf = SH3::makeDualPolygonalSurface( idxsurface );
  auto polySurfPos = polySurf->positions();
  for ( auto i = 0; i < regularizedPos.size(); i++ )
    polySurfPos[ i ] = regularizedPos[ i ];
  SH3::saveOBJ( polySurf, "goursat-shrouds-reg-k2.obj" );
  //! [ShroudsRegSaveObj]
  REQUIRE( loo < 0.1 );
}
