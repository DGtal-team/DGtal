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
 * @file surface-AT-normals
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/06/09
 *
 * An example file named surface-AT-normals.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/geometry/surfaces/SurfaceATSolver.h"
#include "DGtal/dec/DiscreteExteriorCalculusFactory.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  typedef Z3i::KSpace                      KSpace;
  typedef Shortcuts< KSpace >              SH3;
  typedef ShortcutsGeometry< KSpace >      SHG3;
  typedef SH3::Surfel                      Surfel;
  typedef SHG3::RealVector                 RealVector;

  const double alpha_at  = 0.1;
  const double lambda_at = 0.1;
  const double e1        = 2.0;
  const double e2        = 0.25;
  const double er        = 2.0;
  
  auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
  params( "colormap", "Tics" );
  trace.beginBlock ( "Load vol file -> build digital surface -> estimate II normals." );
  auto bimage    = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
  auto K         = SH3::getKSpace( bimage, params );
  auto surface   = SH3::makeDigitalSurface( bimage, K, params );
  auto surfels   = SH3::getSurfelRange( surface, params );
  auto ii_normals= SHG3::getIINormalVectors( bimage, surfels, params );
  trace.endBlock();

  trace.beginBlock ( "Creating AT solver for digital surface" );
  typedef DiscreteExteriorCalculusFactory<EigenLinearAlgebraBackend> CalculusFactory;
  const auto calculus = CalculusFactory::createFromNSCells<2>( surfels.begin(), surfels.end() );
  SurfaceATSolver< KSpace > at_solver(calculus, 2);
  std::map< Surfel, RealVector > input_data;
  for ( size_t i = 0; i < surfels.size(); i++ )
    input_data[ surfels[ i ] ] = ii_normals[ i ];
  auto nb = at_solver.initVectorInput( input_data, false );
  if ( nb != surfels.size() )
    trace.warning() << "Not all the surfels have an input data." << endl;
  at_solver.setUp( alpha_at, lambda_at );
  at_solver.solveGammaConvergence( e1, e2, er );
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
