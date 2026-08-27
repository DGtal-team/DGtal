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
 * @file geometry/meshes/visibility-lattice-normals.cpp
 * @ingroup Examples
 * @author Romain Negro (\c romain.negro@univ-smb.fr )
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2026/08/27
 *
 * An example file named visibility-lattice-normals.
 *
 * This file is part of the DGtal library.
 */

/**
   Computes visibility-based normals with VisibilityLatticeEstimator
   from pointels extracted on a digitized implicit 3D shape.

\verbatim
./examples/geometry/meshes/visibility-lattice-normals [gridstep]
\endverbatim

It outputs `goursat-visibility-lattice-normals.obj`.

@see \ref moduleVisibilityLattice
@example geometry/meshes/visibility-lattice-normals.cpp
 */

#include <algorithm>
#include <cstdlib>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/geometry/surfaces/estimation/VisibilityLatticeEstimator.h"

using namespace DGtal;

int main( int argc, char* argv[] )
{
  typedef Shortcuts< Z3i::KSpace >         SH3;
  typedef ShortcutsGeometry< Z3i::KSpace > SHG3;
  typedef VisibilityLatticeEstimator< SH3::Space > VLE;

  const double h = argc > 1 ? atof( argv[ 1 ] ) : 0.25;

  auto params = SH3::defaultParameters() | SHG3::defaultParameters();
  params( "polynomial", "goursat" )( "gridstep", h )
        ( "radius", -1 )( "nStar", 1 )
        ( "sigma", 1.0 )( "neighborhood-radius", 4.0 )
        ( "visibility-center", "centroid" )
        ( "visibility-kernel", "gaussian" );

  auto implicit_shape  = SH3::makeImplicitShape3D( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto bimage          = SH3::makeBinaryImage( digitized_shape, params );
  auto K               = SH3::getKSpace( params );
  auto surface         = SH3::makeDigitalSurface( bimage, K, params );

  SH3::Cell2Index c2i;
  auto pointel_cells = SH3::getPointelRange( c2i, surface );
  std::vector< SH3::Point > pointels( pointel_cells.size() );
  std::transform( pointel_cells.cbegin(), pointel_cells.cend(), pointels.begin(),
                  [&] ( const SH3::Cell& c ) { return K.uKCoords( c ) / 2; } );

  const int verbose  = params[ "verbose" ].as<int>();
  const int nStar    = params[ "nStar"   ].as<int>();
  const int radius   = VLE::resolveVisibilityRadius( params[ "radius" ].as<int>(),
                                                     params[ "gridstep" ].as<double>() );
  const auto center  = VLE::visibilityCenterFromString(
    params[ "visibility-center" ].as<std::string>() );
  const auto kernel  = VLE::visibilityKernelFromString(
    params[ "visibility-kernel" ].as<std::string>() );
  const double sigma = params[ "sigma" ].as<double>();
  const double neigh = params[ "neighborhood-radius" ].as<double>();
  const bool autoLimiter = params[ "auto-limiter" ].as<int>() != 0;
  const double autoLimiterRatio = params[ "auto-limiter-ratio" ].as<double>();

  const auto visibility = VLE::getVisibility( pointels, nStar, radius, verbose );
  const auto normals    = VLE::getNormalVisibility( pointels, visibility,
                                                    sigma, neigh,
                                                    autoLimiter, autoLimiterRatio,
                                                    center, kernel, verbose );

  auto embedder = SH3::getCellEmbedder( K );
  SH3::RealPoints positions( pointel_cells.size() );
  std::transform( pointel_cells.cbegin(), pointel_cells.cend(), positions.begin(),
                  [&] ( const SH3::Cell& c ) { return h * embedder( c ); } );

  const bool ok = SH3::saveVectorFieldOBJ( positions, normals, 0.05, SH3::Colors(),
                                            "goursat-visibility-lattice-normals.obj",
                                            SH3::Color::Black, SH3::Color::Blue );

  trace.info() << "pointels=" << pointels.size()
               << " normals=" << normals.size()
               << " visibility-empty=" << visibility.empty() << std::endl;

  return ( ok && pointels.size() == normals.size() ) ? 0 : 1;
}
