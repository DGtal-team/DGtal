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
 * @ingroup Examples
 * @author David Coeurjolly (david.coeurjolly@cnrs.fr)
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 *
 * @date 2026/06/08
 *
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"

// Helpers
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"


// Visualization
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/colormaps/GradientColorMap.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace> SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
///////////////////////////////////////////////////////////////////////////////

int main()
{

  //! [Parallel-instantiation]
  auto params = SH3::defaultParameters() | SHG3::defaultParameters();
  params( "polynomial", "goursat" )( "gridstep", 0.125 );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K         = SH3::getKSpace( params );
  auto surface   = SH3::makeDigitalSurface( digitized_shape, K, params );
  auto surfels   = SH3::getSurfelRange( surface, params );
  //! [Parallel-instantiation]

  //! [Parallel-run]
  trace.info()<< "Input vol domain: "<< digitized_shape->getDomain() << std::endl;
  //Sequential
  trace.beginBlock("Single thread");
  auto curv      = SHG3::getIIMeanCurvatures( digitized_shape, surfels, params );
  trace.endBlock();

  //Parallel
  trace.beginBlock("4 threads");
  auto curv_par4  = SHG3::getIIMeanCurvatures( digitized_shape, surfels, params( "ii-thread-number", 4 ) );
  trace.endBlock();

  //Parallel
  trace.beginBlock("8 threads");
  auto curv_par8  = SHG3::getIIMeanCurvatures( digitized_shape, surfels, params( "ii-thread-number", 8 ) );
  trace.endBlock();

  //Parallel
  trace.beginBlock("16 threads");
  auto curv_par16  = SHG3::getIIMeanCurvatures( digitized_shape, surfels, params( "ii-thread-number", 16 ) );
  trace.endBlock();
  //! [Parallel-run]

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
