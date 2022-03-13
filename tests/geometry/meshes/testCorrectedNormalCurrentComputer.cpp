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
 * @file testCorrectedNormalCurrentComputer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/24
 *
 * Functions for testing class CorrectedNormalCurrentComputer.
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
// Functions for testing class CorrectedNormalCurrentComputer.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "CorrectedNormalCurrentComputer interpolated curvature measures on sphere tests", "[icnc][sphere]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, 20, 20,
                               SMH::NormalsType::VERTEX_NORMALS );
  CNCComputer cnc_computer ( sphere, false );
  CNCComputer cncu_computer( sphere, true );
  GIVEN( "A discretized sphere of radius 1 with 20x20x2 triangles" ) {
    THEN( "Its total mu0 measure is close to 4*pi (area)" ) {
      auto mu0   = cnc_computer .computeMu0();
      auto mu0_u = cncu_computer.computeMu0();
      double total_area   = mu0.measure();
      double total_area_u = mu0_u.measure();
      Approx sphere_area  = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == sphere_area );
      REQUIRE( total_area_u == sphere_area );
    }
    THEN( "Its total mu1 measure is close to 8*pi (twice mean curvature)" ) {
      auto mu1   = cnc_computer .computeMu1();
      auto mu1_u = cncu_computer.computeMu1();
      double total_mu1    = mu1.measure();
      double total_mu1_u  = mu1_u.measure();
      Approx twice_mean_c = Approx( 8.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   == twice_mean_c );
      REQUIRE( total_mu1_u == twice_mean_c );
    }
    THEN( "Its total mu2 measure is close to 4*pi (Gaussian curvature)" ) {
      auto mu2   = cnc_computer .computeMu2();
      auto mu2_u = cncu_computer.computeMu2();
      double total_mu2    = mu2.measure();
      double total_mu2_u  = mu2_u.measure();
      Approx gaussian_c   = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu2   == gaussian_c );
      Approx exact_gaussian_c = Approx( 4.0 * M_PI ).epsilon(0.000005);
      REQUIRE( total_mu2_u == exact_gaussian_c );
    }
  }
}

SCENARIO( "CorrectedNormalCurrentComputer face-constant curvature measures on sphere tests", "[ccnc][sphere]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, 20, 20,
                               SMH::NormalsType::FACE_NORMALS );
  CNCComputer cnc_computer ( sphere, false );
  CNCComputer cncu_computer( sphere, true );
  GIVEN( "A discretized sphere of radius 1 with 20x20x2 triangles" ) {
    THEN( "Its total mu0 measure is close to 4*pi (area)" ) {
      auto mu0   = cnc_computer .computeMu0();
      auto mu0_u = cncu_computer.computeMu0();
      double total_area   = mu0.measure();
      double total_area_u = mu0_u.measure();
      Approx sphere_area  = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == sphere_area );
      REQUIRE( total_area_u == sphere_area );
    }
    THEN( "Its total mu1 measure is close to 8*pi (twice mean curvature)" ) {
      auto mu1   = cnc_computer .computeMu1();
      auto mu1_u = cncu_computer.computeMu1();
      double total_mu1    = mu1.measure();
      double total_mu1_u  = mu1_u.measure();
      Approx twice_mean_c = Approx( 8.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   == twice_mean_c );
      REQUIRE( total_mu1_u == twice_mean_c );
    }
    THEN( "Its total mu2 measure is close to 4*pi (Gaussian curvature)" ) {
      auto mu2   = cnc_computer .computeMu2();
      auto mu2_u = cncu_computer.computeMu2();
      double total_mu2    = mu2.measure();
      double total_mu2_u  = mu2_u.measure();
      Approx gaussian_c   = Approx( 4.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu2   == gaussian_c );
      Approx exact_gaussian_c = Approx( 4.0 * M_PI ).epsilon(0.000005);
      REQUIRE( total_mu2_u == exact_gaussian_c );
    }
  }
}

SCENARIO( "CorrectedNormalCurrentComputer interpolated curvature measures on Schwarz lantern tests", "[icnc][lantern]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  SM lantern = SMH::makeLantern( 1.0, 1.0, RealPoint { 0.0, 0.0, 0.0 }, 30, 12,
                                 SMH::NormalsType::VERTEX_NORMALS );
  CNCComputer cnc_computer ( lantern, false );
  CNCComputer cncu_computer( lantern, true );
  GIVEN( "A discretized lantern of radius 1 with 30x12x2 triangles" ) {
    THEN( "Its total mu0 measure is close to 2*pi (area)" ) {
      auto mu0   = cnc_computer .computeMu0();
      auto mu0_u = cncu_computer.computeMu0();
      double total_area   = mu0.measure();
      double total_area_u = mu0_u.measure();
      Approx lantern_area  = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == lantern_area );
      REQUIRE( total_area_u == lantern_area );
    }
    THEN( "Its total mu1 measure is close to 2*pi (twice mean curvature)" ) {
      auto mu1   = cnc_computer .computeMu1();
      auto mu1_u = cncu_computer.computeMu1();
      double total_mu1    = mu1.measure();
      double total_mu1_u  = mu1_u.measure();
      Approx twice_mean_c = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   == twice_mean_c );
      REQUIRE( total_mu1_u == twice_mean_c );
    }
    THEN( "Its total mu2 measure is close to 0 (Gaussian curvature)" ) {
      auto mu2   = cnc_computer .computeMu2();
      auto mu2_u = cncu_computer.computeMu2();
      double total_mu2    = mu2.measure();
      double total_mu2_u  = mu2_u.measure();
      Approx exact_gaussian_c = Approx( 0.0 ).epsilon(0.000005);
      REQUIRE( total_mu2   == exact_gaussian_c );
      REQUIRE( total_mu2_u == exact_gaussian_c );
    }
  }
}

SCENARIO( "CorrectedNormalCurrentComputer face-constant curvature measures on Schwarz lantern tests", "[ccnc][lantern]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  SM lantern = SMH::makeLantern( 1.0, 1.0, RealPoint { 0.0, 0.0, 0.0 }, 30, 12,
                                 SMH::NormalsType::FACE_NORMALS );
  CNCComputer cnc_computer ( lantern, false );
  CNCComputer cncu_computer( lantern, true );
  GIVEN( "A discretized lantern of radius 1 with 30x12x2 triangles" ) {
    THEN( "Its total mu0 measure is close to 2*pi (area)" ) {
      auto mu0   = cnc_computer .computeMu0();
      auto mu0_u = cncu_computer.computeMu0();
      double total_area   = mu0.measure();
      double total_area_u = mu0_u.measure();
      Approx lantern_area  = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_area   == lantern_area );
      REQUIRE( total_area_u == lantern_area );
    }
    THEN( "Its total mu1 measure is close to 2*pi (twice mean curvature)" ) {
      auto mu1   = cnc_computer .computeMu1();
      auto mu1_u = cncu_computer.computeMu1();
      double total_mu1    = mu1.measure();
      double total_mu1_u  = mu1_u.measure();
      Approx twice_mean_c = Approx( 2.0 * M_PI ).epsilon(0.05);
      REQUIRE( total_mu1   == twice_mean_c );
      REQUIRE( total_mu1_u == twice_mean_c );
    }
    THEN( "Its total mu2 measure is close to 0 (Gaussian curvature)" ) {
      auto mu2   = cnc_computer .computeMu2();
      auto mu2_u = cncu_computer.computeMu2();
      double total_mu2    = mu2.measure();
      double total_mu2_u  = mu2_u.measure();
      Approx exact_gaussian_c = Approx( 0.0 ).epsilon(0.000005);
      REQUIRE( total_mu2   == exact_gaussian_c );
      REQUIRE( total_mu2_u == exact_gaussian_c );
    }
  }
}


SCENARIO( "CorrectedNormalCurrentComputer ICNC convergence tests", "[icnc][convergence]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu0 measure tends toward the sphere area" ) {
      std::vector< double > errors_mu0;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          CNCComputer cnc_computer ( sphere, false );
          auto mu0   = cnc_computer .computeMu0();
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
          CNCComputer cnc_computer ( sphere, false );
          auto mu1   = cnc_computer .computeMu1();
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
    THEN( "The total mu2 measure tends toward the sphere area" ) {
      std::vector< double > errors_mu2;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::VERTEX_NORMALS );
          CNCComputer cnc_computer ( sphere, false );
          auto mu2   = cnc_computer .computeMu2();
          errors_mu2.push_back( mu2.measure() );
        }
      double sphere_gauss_c  = 4.0 * M_PI;
      for ( auto & v : errors_mu2 ) v = fabs( v - sphere_gauss_c ) / sphere_gauss_c;
      for ( auto i = 0; i < errors_mu2.size()-1; i++ ) {
        REQUIRE( errors_mu2[ i+1 ] < errors_mu2[ i ] );
      }
    }
  }
}

SCENARIO( "CorrectedNormalCurrentComputer CCNC convergence tests", "[ccnc][convergence]" )
{
  using namespace Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >       SM;
  typedef SurfaceMeshHelper< RealPoint, RealVector > SMH;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNCComputer;

  GIVEN( "A sphere of radius 1 discretized finer and finer" ) {
    THEN( "The total mu0 measure tends toward the sphere area" ) {
      std::vector< double > errors_mu0;
      for ( unsigned int n = 10; n < 50; n += 10 )
        {
          SM sphere = SMH::makeSphere( 1.0, RealPoint { 0.0, 0.0, 0.0 }, n, n,
                                       SMH::NormalsType::FACE_NORMALS );
          CNCComputer cnc_computer ( sphere, false );
          auto mu0   = cnc_computer .computeMu0();
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
                                       SMH::NormalsType::FACE_NORMALS );
          CNCComputer cnc_computer ( sphere, false );
          auto mu1   = cnc_computer .computeMu1();
          errors_mu1.push_back( mu1.measure() );
        }
      double sphere_twice_mc = 8.0 * M_PI;
      for ( auto & v : errors_mu1 ) v = fabs( v - sphere_twice_mc ) / sphere_twice_mc;
      for ( auto i = 0; i < errors_mu1.size()-1; i++ ) {
        REQUIRE( errors_mu1[ i+1 ] < errors_mu1[ i ] );
      }
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
