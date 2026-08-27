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
 * @file geometry/tools/exampleVisibilityNormals.cpp
 * @ingroup Examples
 * @author Romain Negro (\c romain.negro@univ-smb.fr )
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2026/07/29
 *
 * An example file named exampleVisibilityNormals.
 *
 * This file is part of the DGtal library.
 */

/**
   Computes visibility-based normals on pointels extracted from a digitized implicit shape.

\verbatim
./examples/geometry/tools/exampleVisibilityNormals [gridstep]
\endverbatim

@example geometry/tools/exampleVisibilityNormals.cpp
 */

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"

using namespace DGtal;

int main( int argc, char* argv[] )
{
  typedef Shortcuts< Z3i::KSpace >         SH3;
  typedef ShortcutsGeometry< Z3i::KSpace > SHG3;

  const double h = argc > 1 ? atof( argv[ 1 ] ) : 0.25;

  auto params = SH3::defaultParameters() | SHG3::defaultParameters();
  params( "polynomial", "goursat" )( "gridstep", h )
        ( "radius", -1 )( "nStar", 1 )
        ( "sigma", 1.0 )( "neighborhood-radius", 4.0 );

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

  auto visibility = SHG3::getVisibility( pointels, params );
  auto normals    = SHG3::getNormalVisibility( pointels, visibility, params );

  auto embedder = SH3::getCellEmbedder( K );
  SH3::RealPoints positions( pointel_cells.size() );
  std::transform( pointel_cells.cbegin(), pointel_cells.cend(), positions.begin(),
                  [&] ( const SH3::Cell& c ) { return h * embedder( c ); } );

  const bool ok = SH3::saveVectorFieldOBJ( positions, normals, 0.05, SH3::Colors(),
                                            "goursat-visibility-normals.obj",
                                            SH3::Color::Black, SH3::Color::Blue );

  trace.info() << "pointels=" << pointels.size()
               << " normals=" << normals.size()
               << " visibility-empty=" << visibility.empty() << std::endl;

  return ( ok && pointels.size() == normals.size() ) ? 0 : 1;
}
