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
  //! [DigitalRegInit]
  typedef Shortcuts<Z3i::KSpace> SH3;
  typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
  auto params = SH3::defaultParameters()
  | SHG3::defaultParameters();

  params( "polynomial", "goursat" )( "gridstep", 1.0)("verbose", 0);
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  //! [DigitalRegInit]

  SECTION("Basic Construction using Trivial Normals and regular/clamped advection")
  {
    //! [DigitalRegUsage]
    auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
    regul.init();
    regul.attachConvolvedTrivialNormalVectors(params);
    //! [DigitalRegUsage]
    double energy = regul.computeGradient();
    CAPTURE( regul );
    REQUIRE( energy == Approx(1684.340));
    //! [DigitalRegCompute]
    auto finalenergy = regul.regularize();
    //! [DigitalRegCompute]
    REQUIRE( finalenergy == Approx( 4.7763 ) );
    REQUIRE( regul.isValid() );

    //! [DigitalRegOutput]
    auto regularizedPosition = regul.getRegularizedPositions();
    auto original  = regul.getOriginalPositions();
    //! [DigitalRegOutput]
    auto normals   = regul.getNormalVectors();

    auto cellIndex = regul.getCellIndex();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return original[ cellIndex[c]];},
                 normals, SH3::Colors(), "originalSurf.obj");
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                 normals, SH3::Colors(), "regularizedSurf.obj");

    //Testing reset() at few points
    regul.reset();
    regularizedPosition = regul.getRegularizedPositions();
    REQUIRE( original[0] == regularizedPosition[0] );
    REQUIRE( original[123] == regularizedPosition[123] );

    //Testing Clamped version
    auto finalenergyClamped = regul.regularize(200,1.0,0.001, DigitalSurfaceRegularization<SH3::DigitalSurface>::clampedAdvection);
    regularizedPosition = regul.getRegularizedPositions();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                 normals, SH3::Colors(), "regularizedSurfClamped.obj");
    REQUIRE( finalenergyClamped == Approx(12.1914) );
    REQUIRE( finalenergy < finalenergyClamped );

    //Testing accessor
    auto aPointelIndex = cellIndex.begin();
    REQUIRE( regularizedPosition[ aPointelIndex->second ] == regul.getRegularizedPosition( aPointelIndex->first) );
  }

  SECTION("Basic Construction with II Normal Vectors")
  {
    auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    //! [DigitalRegII]
    auto ii_normals = SHG3::getIINormalVectors(digitized_shape, surfels, params);
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
    regul.init();
    auto surfelIndex = regul.getSurfelIndex();
    regul.attachNormalVectors([&](SH3::SCell &c){ return ii_normals[ surfelIndex[c] ];} );
    //! [DigitalRegII]

    double energy    = regul.computeGradient();
    CAPTURE( regul );
    REQUIRE( energy == Approx(1588.649));
    regul.regularize();
    REQUIRE( regul.isValid() );

    auto regularizedPosition = regul.getRegularizedPositions();
    auto normals = regul.getNormalVectors();
    auto cellIndex   = regul.getCellIndex();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                 normals, SH3::Colors(), "regularizedSurf-II.obj");
  }

  SECTION("Warm restart")
  {
    auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
    regul.init();
    regul.attachConvolvedTrivialNormalVectors(params);
    CAPTURE( regul );
    auto energy       = regul.regularize(10,1.0,0.1);
    auto secondenergy = regul.regularize(10,1.0,0.1);
    auto thirdenergy  = regul.regularize(10,1.0,0.1);
    CAPTURE(energy);
    CAPTURE(secondenergy);
    CAPTURE(thirdenergy);

    REQUIRE( energy > secondenergy );
    REQUIRE( secondenergy > thirdenergy );
  }

  SECTION("Local weights")
  {
    auto surface         = SH3::makeDigitalSurface( digitized_shape, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
    regul.init();
    regul.attachConvolvedTrivialNormalVectors(params);
    CAPTURE( regul );
    auto energy       = regul.regularize(10,1.0,0.1);

    auto original = regul.getOriginalPositions();
    std::vector<double> alphas(original.size(),0.001);
    std::vector<double> betas(original.size(),1.0);
    std::vector<double> gammas(original.size(), 0.05);

    //Init again with variable (but constant) weights
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul2(surface);
    regul2.init(alphas,betas,gammas);
    regul2.attachConvolvedTrivialNormalVectors(params);
    auto energybis  = regul2.regularize(10,1.0,0.1);
    REQUIRE( energy == energybis );

    energybis = regul2.regularize();
    auto regularizedPosition = regul.getRegularizedPositions();
    auto normals   = regul.getNormalVectors();
    auto cellIndex = regul.getCellIndex();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                 normals, SH3::Colors(), "regularizedSurf-local.obj");

    //Same with higher data attachment for x < 0.0
    //! [DigitalRegLocal]
    DigitalSurfaceRegularization<SH3::DigitalSurface> regul3(surface);
    for(size_t i = 0 ; i < original.size(); ++i)
      if (original[i][0]<0.0)
      {
        alphas[i] = 4.0;
        betas[i]  = 0.0000001;
        gammas[i] = 0.0;
      }
    regul3.init(alphas,betas,gammas);
    regul3.attachConvolvedTrivialNormalVectors(params);
    energybis = regul3.regularize();
    //! [DigitalRegLocal]

    regularizedPosition = regul3.getRegularizedPositions();
    SH3::saveOBJ(surface, [&] (const SH3::Cell &c){ return regularizedPosition[ cellIndex[c]];},
                 normals, SH3::Colors(), "regularizedSurf-localsplit.obj");
  }
}

/** @ingroup Tests **/
