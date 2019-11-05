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
 * @file exampleSurfaceATNormals.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/06/09
 *
 * An example file named exampleSurfaceATNormals.
 *
 * This file is part of the DGtal library.
 */

/////////////////////////////////////////////////////////////////////
//! [AT-surface-includes]
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/dec/ATSolver2D.h"
#include "DGtal/dec/DiscreteExteriorCalculusFactory.h"
//! [AT-surface-includes]
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
  typedef SH3::Cell                        Cell;
  typedef SHG3::RealVector                 RealVector;

  const double alpha_at  = 0.1;
  const double lambda_at = 0.01;
  const double e1        = 2.0;
  const double e2        = 0.25;
  const double er        = 2.0;

  const string   volfile = argc > 1 ? argv[ 1 ] : examplesPath + "samples/Al.100.vol";
  const double threshold = argc > 2 ? atof( argv[ 2 ] ) : 0.5;
  trace.beginBlock ( "Load vol file -> build digital surface -> estimate II normals." );
  //! [AT-surface-init]
  auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
  auto bimage    = SH3::makeBinaryImage( volfile, params );
  auto K         = SH3::getKSpace( bimage, params );
  auto surface   = SH3::makeDigitalSurface( bimage, K, params );
  auto surfels   = SH3::getSurfelRange( surface, params );
  auto linels    = SH3::getCellRange( surface, 1 );
  auto ii_normals= SHG3::getIINormalVectors( bimage, surfels, params );
  auto uembedder = SH3::getCellEmbedder( K );
  auto embedder  = SH3::getSCellEmbedder( K );
  //! [AT-surface-init]
  trace.endBlock();

  trace.beginBlock ( "Creating AT solver for digital surface" );
  //! [AT-surface-calculus]
  typedef DiscreteExteriorCalculusFactory<EigenLinearAlgebraBackend> CalculusFactory;
  const auto calculus = CalculusFactory::createFromNSCells<2>( surfels.begin(), surfels.end() );
  //! [AT-surface-calculus]
  //! [AT-surface-solve]
  ATSolver2D< KSpace > at_solver(calculus, 1);
  at_solver.initInputVectorFieldU2( ii_normals, surfels.cbegin(), surfels.cend() );
  at_solver.setUp( alpha_at, lambda_at );
  at_solver.solveGammaConvergence( e1, e2, er );
  //! [AT-surface-solve]
  trace.endBlock();

  trace.beginBlock ( "Save AT normals as OBJ file" );
  //! [AT-surface-getU2]
  auto at_normals = ii_normals;
  at_solver.getOutputVectorFieldU2( at_normals, surfels.cbegin(), surfels.cend() );
  //! [AT-surface-getU2]
  SH3::RealPoints positions( surfels.size() );
  std::transform( surfels.cbegin(), surfels.cend(), positions.begin(),
		  [&] (const SH3::SCell& c) { return embedder( c ); } );
  SH3::Colors colors( surfels.size() );
  for ( size_t i = 0; i < surfels.size(); i++ )
    colors[ i ] = SH3::Color( (unsigned char) 255.0*fabs( at_normals[ i ][ 0 ] ),
			      (unsigned char) 255.0*fabs( at_normals[ i ][ 1 ] ),
			      (unsigned char) 255.0*fabs( at_normals[ i ][ 2 ] ) );
  std::transform( surfels.cbegin(), surfels.cend(), positions.begin(),
		  [&] (const SH3::SCell& c) { return embedder( c ); } );

  bool ok1  = SH3::saveOBJ( surface, at_normals, SH3::Colors(),
			    "output-surface.obj" );
  bool ok2 = SH3::saveOBJ( surface, at_normals, colors,
			   "output-surface-at-normals.obj" );
  bool ok3 = SH3::saveVectorFieldOBJ( positions, at_normals, 0.05, SH3::Colors(),
				      "output-vf-at-normals.obj",
				      SH3::Color( 0, 0, 0 ), SH3::Color::Red );
  //! [AT-surface-getV0]
  SH3::Scalars features( linels.size() );
  at_solver.getOutputScalarFieldV0( features, linels.cbegin(), linels.cend(),
				    at_solver.Maximum );
  //! [AT-surface-getV0]
  SH3::RealPoints  f0;
  SH3::RealVectors f1;
  for ( size_t i = 0; i < linels.size(); i++ )
    {
      if ( features[ i ] < threshold )
	{
	  const Cell  linel = linels[ i ];
	  const Dimension d = * K.uDirs( linel );
	  const Cell     p0 = K.uIncident( linel, d, false );
	  const Cell     p1 = K.uIncident( linel, d, true  );
	  f0.push_back( uembedder( p0 ) );
	  f1.push_back( uembedder( p1 ) - uembedder( p0 ) );
	}
    }
  bool ok4 = SH3::saveVectorFieldOBJ( f0, f1, 0.1, SH3::Colors(),
				      "output-features.obj",
				      SH3::Color( 0, 0, 0 ), SH3::Color::Red );

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
