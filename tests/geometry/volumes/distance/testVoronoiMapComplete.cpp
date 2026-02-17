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
 * @author Robin Lamy
 *
 * @date 2021/07/20
 *
 * Functions for testing class VoronoiMapComplete.
 *
 * This file is part of the DGtal library.
 */
///////////////////////////////////////////////////////////////////////////////
#include <limits>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtalCatch.h"

#include "DGtal/geometry/volumes/distance/VoronoiMapComplete.h"
#include "DGtal/geometry/volumes/distance/VoronoiMap.h"
///////////////////////////////////////////////////////////////////////////////
using namespace DGtal;

typedef Z2i::DigitalSet DigitalSet;
typedef Z2i::Point Point;
typedef Z2i::Domain Domain;

typedef VoronoiMapComplete<Z2i::Space, DigitalSet, Z2i::L2Metric> CompleteVMap;
typedef VoronoiMap<Z2i::Space, DigitalSet, Z2i::L2Metric> VMap;
typedef ImageContainerBySTLVector<HyperRectDomain<Z2i::Space>,
                                  std::unordered_set<Z2i::Point>>
TImageContainer;

TImageContainer * brut_force_voronoi_map_complete( DigitalSet & set )
{
  TImageContainer * voronoi_map = new TImageContainer( set.domain() );
  std::vector<Point> Sites;
  // Getting all voronoi sites in one vector
  for ( Point point : set.domain() )
  {
    if ( !set( point ) )
    {
      Sites.push_back( point );
    }
  }
  std::cout << std::endl;

  for ( Point point : set.domain() )
  {

    std::unordered_set<Point> voronoi_points;
    double voronoi_distance =
    std::numeric_limits<int>::max(); // maximum int value
    for ( Point site : Sites )
    {
      if ( ( point - site ).norm() < voronoi_distance )
      {
        voronoi_points.clear();
        voronoi_points.insert( site );
        voronoi_distance = ( point - site ).norm();
      }
      else if ( ( point - site ).norm() == voronoi_distance )
      {
        voronoi_points.insert( site );
      }
    }
    voronoi_map->setValue( point, voronoi_points );
  }
  return voronoi_map;
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VoronoiMapComplete
///////////////////////////////////////////////////////////////////////////////
TEST_CASE( "Testing VoronoiMapComplete 2D" )
{
  /**
   * Random set generation
   */
  Point lowerBound( 0, 0 ), upperBound( 31, 31 );
  Domain domain( lowerBound, upperBound );

  DigitalSet set( domain );
  int point_setup_index = 0;
  int x;
  int y;

  while ( point_setup_index < 400 )
  {
    x = rand() % 32;
    y = rand() % 32;
    set.insert( Point( x, y ) );
    point_setup_index++;
  }

  /**
   * Voronoi Map Complete from class generation
   */
  CompleteVMap vmap( set.domain(), set, Z2i::l2Metric );

  /**
   * Voronoi Map Complete from brut force generation
   */
  TImageContainer * brut_force_vmap = brut_force_voronoi_map_complete( set );

  SECTION(
  "Testing that VoronoiMapComplete class gives the right voronoi sites sets" )
  {
    for ( Point point : set )
    {
      std::unordered_set<Point> brut_force_set =
      brut_force_vmap->operator()( point );
      CompleteVMap::Value class_set = vmap( point );

      // Checking that all voronoi sites from the brut force set are in the
      // algorithm set
      for ( Point voronoi_point : brut_force_set )
      {
        REQUIRE( std::find( class_set.begin(), class_set.end(),
                            voronoi_point ) != class_set.end() );
      }

      // Checking that all voronoi sites from the algorithm set are in the brut
      // force set
      for ( Point voronoi_point : class_set )
      {
        REQUIRE( std::find( brut_force_set.begin(), brut_force_set.end(),
                            voronoi_point ) != brut_force_set.end() );
      }
    }
  }

  SECTION(
  "Testing Complete Voronoi Map from Discrete Bisector Function paper" )
  {
    Point _lowerBound( 0, 0 ), _upperBound( 6, 7 );
    Domain _domain( _lowerBound, _upperBound );
    DigitalSet _set( _domain );
    for ( Point point : _set.domain() )
    {
      if ( point != Point( 1, 0 ) && point != Point( 5, 0 ) &&
           point != Point( 2, 2 ) && point != Point( 4, 4 ) &&
           point != Point( 0, 6 ) && point != Point( 6, 6 ) &&
           point != Point( 3, 7 ) )
        _set.insert( point );
    }

    TImageContainer * brutForceVoronoiMap =
    brut_force_voronoi_map_complete( _set );
    CompleteVMap _vmap( _set.domain(), _set, Z2i::l2Metric );

    for ( Point point : _set )
    {
      std::unordered_set<Point> brut_force_set =
      brutForceVoronoiMap->operator()( point );
      CompleteVMap::Value class_set = _vmap( point );

      // Checking that all voronoi sites from the brut force set are in the
      // algorithm set
      for ( Point voronoi_point : brut_force_set )
        REQUIRE( std::find( class_set.begin(), class_set.end(),
                            voronoi_point ) != class_set.end() );

      // Checking that all voronoi sites from the algorithm set are in the brut
      // force set
      for ( Point voronoi_point : class_set )
        REQUIRE( std::find( brut_force_set.begin(), brut_force_set.end(),
                            voronoi_point ) != brut_force_set.end() );
    }
  }
}

TEST_CASE( "No sites" )
{
  /**
   * Random set generation
   */
  Point lowerBound( 0, 0 ), upperBound( 255, 255 );
  Domain domain( lowerBound, upperBound );
  DigitalSet set( domain );

  trace.beginBlock( "Complete Map (no site)" );
  CompleteVMap vmap( set.domain(), set, Z2i::l2Metric );
  trace.endBlock();
  trace.beginBlock( "Partial Map (no site)" );
  VMap partialmap( set.domain(), set, Z2i::l2Metric );
  trace.endBlock();
}

TEST_CASE( "Testing Timings" )
{
  /**
   * Random set generation
   */
  Point lowerBound( 0, 0 ), upperBound( 255, 255 );
  Domain domain( lowerBound, upperBound );

  DigitalSet set( domain );
  int point_setup_index = 0;
  int x;
  int y;

  while ( point_setup_index < 5000 )
  {
    x = rand() % 256;
    y = rand() % 256;
    set.insert( Point( x, y ) );
    point_setup_index++;
  }
  std::cout << std::endl;
  trace.beginBlock( "Complete Map" );
  CompleteVMap vmap( set.domain(), set, Z2i::l2Metric );
  trace.endBlock();

  size_t maxsize = 0;
  for ( auto & v : vmap.constRange() )
    maxsize = std::max( maxsize, v.size() );
  trace.info() << "Max number of co-cyclic points = " << maxsize << std::endl;

  trace.beginBlock( "Partial Map" );
  VMap partialmap( set.domain(), set, Z2i::l2Metric );
  trace.endBlock();
}
