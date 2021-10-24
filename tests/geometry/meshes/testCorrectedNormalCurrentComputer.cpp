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
 * @file testCellGeometry.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2019/01/04
 *
 * Functions for testing class CellGeometry.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellGeometry.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "CorrectedNormalCurrentComputer sphere tests", "[cnc][sphere]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, 20, 20,
                               SMH::NormalsType::VERTEX_NORMALS );
  CNCComputer cnc_computer ( sphere, false );
  CNCComputer cncu_computer( sphere, true );
  GIVEN( "A discretized sphere of radius 1" ) {
    auto mu0   = cnc_computer .computeMu0();
    auto mu0_u = cncu_computer.computeMu0();
    THEN( "Its total mu0 measure is close to 4*pi" ) {
      double total_area   = mu0.measure();
      double total_area_u = mu0_u.measure();
      Approx sphere_area = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == sphere_area );
      REQUIRE( total_area_u == sphere_area );
    }
  }
}

SCENARIO( "CorrectedNormalCurrentComputer convergence tests", "[cnc][convegrence]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu0 measure tends toward the sphere area" ) {
      std::vector< double > errors;
      for ( unsigned int n = 10; n < 100; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          CNCComputer cnc_computer ( sphere, false );
          auto mu0   = cnc_computer .computeMu0();
          errors.push_back( mu0.measure() );
        }
      double sphere_area = 4.0 * M_PI;
      for ( auto & v : errors ) v = fabs( v - sphere_area ) / sphere_area;
      for ( auto i = 0; i < errors.size()-1; i++ )
        REQUIRE( errors[ i+1 ] < errors[ i ] );
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
