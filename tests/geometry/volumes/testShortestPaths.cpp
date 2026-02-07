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
 * @file testShortestPaths.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/05/09
 *
 * Functions for testing shortest paths.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/geometry/volumes/TangencyComputer.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class TangencyComputer::ShortestPaths
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "TangencyComputer::ShortestPaths 3D tests", "[shortest_paths][3d][tangency]" )
{
  typedef Z3i::Space          Space;
  typedef Z3i::KSpace         KSpace;
  typedef Shortcuts< KSpace > SH3;
  typedef Space::Point        Point;
  typedef std::size_t         Index;

  SECTION( "Computing shortest paths on a 3D unit sphere digitized at gridstep 0.25" )
    {
      // Make digital sphere
      const double h = 0.25;
      auto   params  = SH3::defaultParameters();
      params( "polynomial", "sphere1" )( "gridstep",  h );
      params( "minAABB", -2)( "maxAABB", 2)( "offset", 1.0 )( "closed", 1 );
      auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
      auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
      auto K            = SH3::getKSpace( params );
      auto binary_image = SH3::makeBinaryImage(digitized_shape,
                                               SH3::Domain(K.lowerBound(),K.upperBound()),
                                               params );
      auto surface = SH3::makeDigitalSurface( binary_image, K, params );
      std::vector< Point >    lattice_points;
      auto pointels = SH3::getPointelRange( surface );
      for ( auto p : pointels ) lattice_points.push_back( K.uCoords( p ) );
      REQUIRE( pointels.size() == 296 );
      // Find lowest and uppest point.
      const Index nb = lattice_points.size();
      Index   lowest = 0;
      Index   uppest = 0;
      for ( Index i = 1; i < nb; i++ )
        {
          if ( lattice_points[ i ] < lattice_points[ lowest ] ) lowest = i;
          if ( lattice_points[ uppest ] < lattice_points[ i ] ) uppest = i;
        }
      // Compute shortest paths
      typedef TangencyComputer< KSpace >::Index _Index;
      TangencyComputer< KSpace > TC( K );
      TC.init( lattice_points.cbegin(), lattice_points.cend() );
      auto SP = TC.makeShortestPaths( sqrt(3.0) );
      SP.init( lowest ); //< set source
      double last_distance = 0.0;
      _Index  last = 0;
      double prev_distance = 0.0;
      std::set< _Index > V;
      unsigned int nb_multiple_pops = 0;
      unsigned int nb_decreasing_distance = 0;
      while ( ! SP.finished() )
        {
          last = std::get<0>( SP.current() );
          last_distance = std::get<2>( SP.current() );
          if ( V.count( last ) ) nb_multiple_pops += 1;
          V.insert( last );
          SP.expand();
          if ( last_distance < prev_distance ) nb_decreasing_distance += 1;
          prev_distance = last_distance;
        }
      // THEN( "No point is popped several times" )
      REQUIRE( nb_multiple_pops == 0 );
      // AND_THEN( "The sequence of popped points has non decreasing distances" )
      REQUIRE( nb_decreasing_distance == 0 );
      // AND_THEN( "The furthest point is also the antipodal point" )
      REQUIRE( last == uppest );
      // AND_THEN( "The furthest point is at distance close but lower than pi" )
      REQUIRE( last_distance*h >= 2.8 );
      REQUIRE( last_distance*h <= 3.14159265358979323844 );
      // Compute approximate shortest paths
      SP = TC.makeShortestPaths( 0 );
      SP.init( uppest ); //< set source
      double last_distance_opt = 0.0;
      while ( ! SP.finished() )
        {
          last = std::get<0>( SP.current() );
          last_distance_opt = std::get<2>( SP.current() );
          SP.expand();
        }
      // THEN( "The furthest point is also the antipodal point" )
      REQUIRE( last == lowest );
      // AND_THEN( "The furthest point is at distance close but lower than pi" )
      REQUIRE( last_distance_opt*h >= 2.8 );
      REQUIRE( last_distance_opt*h <= 3.14159265358979323844 );
      // AND_THEN( "This distance is greater or equal to the exacts shortest path" )
      REQUIRE( last_distance_opt*h >= last_distance*h );
    }
}
