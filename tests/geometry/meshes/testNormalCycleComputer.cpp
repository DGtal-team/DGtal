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
 * @file testNormalCycleComputer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/24
 *
 * Functions for testing class NormalCycleComputer.
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
#include "DGtal/geometry/meshes/NormalCycleComputer.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class NormalCycleComputer.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "NormalCycleComputer sphere tests", "[nc][sphere]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef NormalCycleComputer< RealPoint, RealVector > NCComputer;

  SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, 10, 10,
                               SMH::NormalsType::FACE_NORMALS );
  NCComputer nc_computer ( sphere );
  GIVEN( "A discretized sphere of radius 1 with 10x10 quadrangles and triangles" ) {
    THEN( "Its total mu0 measure is close to 4*pi (area)" ) {
      auto mu0   = nc_computer .computeMu0();
      double total_area   = mu0.measure();
      Approx sphere_area  = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == sphere_area );
    }
    THEN( "Its total mu1 measure is close to 8*pi (twice mean curvature)" ) {
      auto mu1   = nc_computer .computeMu1();
      double total_mu1    = mu1.measure();
      Approx twice_mean_c = Approx( 8.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   == twice_mean_c );
    }
    THEN( "Its total mu2 measure is close to 4*pi (Gaussian curvature)" ) {
      auto mu2   = nc_computer .computeMu2();
      double total_mu2    = mu2.measure();
      Approx gaussian_c   = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu2   == gaussian_c );
    }
  }
}

SCENARIO( "NormalCycleComputer Schwarz lantern tests", "[nc][lantern]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef NormalCycleComputer< RealPoint, RealVector > NCComputer;

  SM lantern = SMH::makeLantern( 1.0, 1.0, RealPoint { 0.0, 0.0, 0.0 }, 30, 12,
                                 SMH::NormalsType::VERTEX_NORMALS );
  NCComputer nc_computer ( lantern );
  GIVEN( "A discretized lantern of radius 1 with 30x12x2 triangles" ) {
    THEN( "Its total mu0 measure is close to 2*pi (area)" ) {
      auto mu0   = nc_computer .computeMu0();
      double total_area   = mu0.measure();
      Approx lantern_area  = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   != lantern_area );
    }
    THEN( "Its total mu1 measure is not close to 2*pi (twice mean curvature)" ) {
      auto mu1   = nc_computer .computeMu1();
      double total_mu1    = mu1.measure();
      Approx twice_mean_c = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   != twice_mean_c );
    }
    THEN( "Its total mu2 measure not close to 0 (Gaussian curvature)" ) {
      auto mu2   = nc_computer .computeMu2();
      double total_mu2    = mu2.measure();
      Approx gaussian_c   = Approx( 0.0 ).epsilon(0.05);
      REQUIRE( total_mu2   != gaussian_c );
    }
  }
}


SCENARIO( "NormalCycleComputer convergence tests", "[nc][convergence]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef NormalCycleComputer< RealPoint, RealVector > NCComputer;

  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu0 measure tends toward the sphere area" ) {
      std::vector< double > errors_mu0;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          NCComputer nc_computer ( sphere );
          auto mu0   = nc_computer .computeMu0();
          errors_mu0.push_back( mu0.measure() );
        }
      double sphere_area     = 4.0 * M_PI;
      for ( auto & v : errors_mu0 ) v = fabs( v - sphere_area ) / sphere_area;
      for ( auto i = 0; i < errors_mu0.size()-1; i++ ) {
        REQUIRE( errors_mu0[ i+1 ] < errors_mu0[ i ] );
      }
    }
  }
  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu1 measure tends toward twice the sphere area" ) {
      std::vector< double > errors_mu1;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          NCComputer nc_computer ( sphere );
          auto mu1   = nc_computer .computeMu1();
          errors_mu1.push_back( mu1.measure() );
        }
      double sphere_twice_mc = 8.0 * M_PI;
      for ( auto & v : errors_mu1 ) v = fabs( v - sphere_twice_mc ) / sphere_twice_mc;
      for ( auto i = 0; i < errors_mu1.size()-1; i++ ) {
        REQUIRE( errors_mu1[ i+1 ] < errors_mu1[ i ] );
      }
    }
  }
  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu2 measure is the sphere area" ) {
      std::vector< double > errors_mu2;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          NCComputer nc_computer ( sphere  );
          auto mu2   = nc_computer .computeMu2();
          errors_mu2.push_back( mu2.measure() );
        }
      double sphere_gauss_c  = 4.0 * M_PI;
      for ( auto & v : errors_mu2 ) v = fabs( v - sphere_gauss_c ) / sphere_gauss_c;
      for ( auto i = 0; i < errors_mu2.size(); i++ ) {
        REQUIRE( errors_mu2[ i ] == Approx( 0.0 ).margin( 1e-8 ) );
      }
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
