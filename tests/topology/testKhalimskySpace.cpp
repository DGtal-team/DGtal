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
 * @file testCellularGridSpaceND.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-smb.fr )
 * LAboratory of MAthematics - LAMA (CNRS, UMR 5127), University of Savoie Mont Blanc, France
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 * LAboratory of MAthematics - LAMA (CNRS, UMR 5127), University of Savoie Mont Blanc, France
 *
 * @date 2016/03/23
 *
 * Functions for testing classes KhalimskySpaceND and KhalimskyPreSpaceND.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstddef>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/KhalimskyPreSpaceND.h"

#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/CPreCellularGridSpaceND.h"

#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"

#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
/** Tests uNext and sNext
 *
 * @tparam KSpace the Khalimsky space type (auto-deduced).
 * @param K    the Khalimsky space.
 * @param low  the first point to scan.
 * @param high the last point to scan.
 */
template < typename KSpace >
void testScan( KSpace const & K,
               typename KSpace::Point const & low,
               typename KSpace::Point const & high
             )
{
  INFO( "Testing uNext & sNext with low = " << low << " & high = " << high );

  using Point = typename KSpace::Point;
  using Space = typename KSpace::Space;
  using PK    = typename KSpace::PreCellularGridSpace;

  const HyperRectDomain< Space > domain( low, high );

  // Looping through cell topology
  for ( unsigned int t = 0; t < 1u << KSpace::dimension; ++t )
    {
      // Constructing the Khalimsky coordinates
      Point refPoint;
      for ( DGtal::Dimension i = 0; i < KSpace::dimension; ++i )
        refPoint[ i ] = t & (1u << i) ? 1 : 0;

      INFO( "Current topology is " << refPoint );

      // Initializing unsigned cells
      const auto refUCell = PK::uCell( refPoint );
      auto currUCell = K.uCell( low, refUCell );
      const auto lowUCell  = currUCell;
      const auto highUCell = K.uCell( high, refUCell );
      REQUIRE( K.uTopology( currUCell ) == PK::uTopology( refUCell ) );

      // Initializing signed cells
      const auto refSCell = PK::sCell( refPoint, PK::POS );
      auto currSCell = K.sCell( low, refSCell );
      const auto lowSCell  = currSCell;
      const auto highSCell = K.sCell( high, refSCell );
      REQUIRE( K.sTopology( currSCell ) == PK::sTopology( refSCell ) );

      // Spanning the domain
      bool uCheck = true;
      bool sCheck = true;
      for ( Point const & pt : domain )
        {
          REQUIRE( uCheck == true );
          REQUIRE( sCheck == true );

          REQUIRE( currUCell == K.uCell( pt, refUCell ) );
          REQUIRE( currSCell == K.sCell( pt, refSCell ) );

          uCheck = K.uNext( currUCell, lowUCell, highUCell );
          sCheck = K.sNext( currSCell, lowSCell, highSCell );
        }

      // Checking scan end conditions.
      REQUIRE( uCheck == false );
      REQUIRE( sCheck == false );
    }
}

///////////////////////////////////////////////////////////////////////////////
/** Comparing unsigned cells list that are inside a Khalimsky space.
 *
 * @tparam KSpace the Khalimsky space type.
 * @tparam Cells the second unsigned cell list.
 * @param K the Khalimsky space.
 * @param u an unsigned cell list from \a K.
 * @param v an unsigned cell list from an another Khalimsky space.
 */
template < typename KSpace, typename Cells >
void cmpUCellsIfInside( KSpace const & K,
                        typename KSpace::Cells const & u,
                        Cells const & v
                      )
{
  REQUIRE( u.size() <= v.size() );

  std::size_t cnt = 0;

  // Scanning the lists
  for( auto const & cell : v )
    {
      if ( ! K.uIsInside( cell ) )
        continue;

      REQUIRE( std::find( u.cbegin(), u.cend(), K.uCell( cell ) ) != u.cend() );

      ++cnt;
    }

  // Checking counter
  REQUIRE( u.size() == cnt );
}

///////////////////////////////////////////////////////////////////////////////
/** Comparing signed cells list that are inside a Khalimsky space.
 *
 * @tparam KSpace the Khalimsky space type.
 * @tparam Cells  the second signed cell list.
 * @param K the Khalimsky space.
 * @param u an signed cell list from \a K.
 * @param v an signed cell list from an another Khalimsky space.
 */
template < typename KSpace, typename Cells  >
void cmpSCellsIfInside( KSpace const & K,
                        typename KSpace::SCells const & u,
                        Cells const & v
                      )
{
  REQUIRE( u.size() <= v.size() );

  std::size_t cnt = 0;

  // Scanning the lists
  for( auto const & cell : v )
    {
      if ( ! K.sIsInside( cell ) )
        continue;

      REQUIRE( std::find( u.cbegin(), u.cend(), K.sCell( cell ) ) != u.cend() );

      ++cnt;
    }

  // Checking counter
  REQUIRE( u.size() == cnt );
}

///////////////////////////////////////////////////////////////////////////////
/** Tests (proper) neighborhoods
 *
 * @tparam KSpace the Khalimsky space type (auto-deduced).
 * @param K the Khalimsky space.
 * @param aPoint the point around which to check the neighborhoods.
 */
template < typename KSpace >
void testNeighborhood( KSpace const & K,
                       typename KSpace::Point const & aPoint
                     )
{
  INFO( "Testing (proper) neighborhood around point " << aPoint );

  using Point = typename KSpace::Point;
  using PK    = typename KSpace::PreCellularGridSpace;

  // Looping through cell topology
  for ( unsigned int t = 0; t < 1u << KSpace::dimension; ++t )
    {
      // Constructing the Khalimsky coordinates
      Point refPoint;
      for ( DGtal::Dimension i = 0; i < KSpace::dimension; ++i )
        refPoint[ i ] = t & (1u << i) ? 1 : 0;

      INFO( "Current topology is " << refPoint );

      // Constructing the unsigned cell
      const auto refUCell  = PK::uCell( refPoint );

      if ( ! K.uIsInside( PK::uCell( aPoint, refUCell ) ) )
        continue; // Do not test if current point is outside space.

      const auto currUCell = K.uCell( aPoint, refUCell );
      REQUIRE( K.uTopology( currUCell ) == PK::uTopology( refUCell ) );

      // Constructing the signed cell
      const auto refSCell  = PK::sCell( refPoint, PK::NEG );
      const auto currSCell = K.sCell( aPoint, refSCell );
      REQUIRE( K.sTopology( currSCell ) == PK::sTopology( refSCell ) );

      // Testing neighbordhoods
        {
          const auto currUCells = K.uNeighborhood( currUCell );
          const auto refUCells  = PK::uNeighborhood( currUCell );
          const auto currSCells = K.sNeighborhood( currSCell );
          const auto refSCells  = PK::sNeighborhood( currSCell );

          REQUIRE( currUCells.size() == currSCells.size() );
          REQUIRE( refUCells.size() == refSCells.size() );
          REQUIRE( refUCells.size() == 2*K.dimension + 1 );
          cmpUCellsIfInside( K, currUCells, refUCells );
          cmpSCellsIfInside( K, currSCells, refSCells );
        }

      // Testing proper neighbordhoods
        {
          const auto currUCells = K.uProperNeighborhood( currUCell );
          const auto refUCells  = PK::uProperNeighborhood( currUCell );
          const auto currSCells = K.sProperNeighborhood( currSCell );
          const auto refSCells  = PK::sProperNeighborhood( currSCell );

          REQUIRE( currUCells.size() == currSCells.size() );
          REQUIRE( refUCells.size() == refSCells.size() );
          REQUIRE( refUCells.size() == 2*K.dimension );
          cmpUCellsIfInside( K, currUCells, refUCells );
          cmpSCellsIfInside( K, currSCells, refSCells );
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
/** Test faces and co-faces.
 *
 * @tparam KSpace the Khalimsky space type (auto-deduced).
 * @param K the Khalimsky space.
 * @param aPoint the point around which to check the neighborhoods.
 */
template < typename KSpace >
void testFaces( KSpace const & K,
                       typename KSpace::Point const & aPoint
                     )
{
  INFO( "Testing faces and cofaces around point " << aPoint );

  using Point = typename KSpace::Point;
  using PK    = typename KSpace::PreCellularGridSpace;

  // Looping through cell topology
  for ( unsigned int t = 0; t < 1u << KSpace::dimension; ++t )
    {
      // Constructing the Khalimsky coordinates
      Point refPoint;
      for ( DGtal::Dimension i = 0; i < KSpace::dimension; ++i )
        refPoint[ i ] = t & (1u << i) ? 1 : 0;

      INFO( "Current topology is " << refPoint );

      // Constructing the unsigned cell
      const auto refUCell  = PK::uCell( refPoint );

      if ( ! K.uIsInside( PK::uCell( aPoint, refUCell ) ) )
        continue; // Do not test if current point is outside space.

      const auto currUCell = K.uCell( aPoint, refUCell );
      REQUIRE( K.uTopology( currUCell ) == PK::uTopology( refUCell ) );

      // Constructing the signed cell
      const auto refSCell  = PK::sCell( refPoint, PK::NEG );
      const auto currSCell = K.sCell( aPoint, refSCell );
      REQUIRE( K.sTopology( currSCell ) == PK::sTopology( refSCell ) );

      // Testing faces
        {
          const auto currUCells = K.uFaces( currUCell );
          const auto refUCells  = PK::uFaces( currUCell );

          REQUIRE( refUCells.size() == floor( std::pow( 3, K.uDim( currUCell ) ) - 1 ) );
          cmpUCellsIfInside( K, currUCells, refUCells );
        }

      // Testing proper neighbordhoods
        {
          const auto currUCells = K.uCoFaces( currUCell );
          const auto refUCells  = PK::uCoFaces( currUCell );

          cmpUCellsIfInside( K, currUCells, refUCells );
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
/** Testing block incidence
 *
 * @tparam KSpace the Khalimsky space type.
 * @param  K      the Khalimsky space.
 * @param  aPoint a point where to test the incidences.
 */
template < typename KSpace >
void testIncidence( KSpace const & K,
                    typename KSpace::Point const & aPoint
                  )
{
  INFO( "Testing block Incidence in KSpace..." );
  
  using SCell = typename KSpace::SCell;
  using DirIterator = typename KSpace::DirIterator;

  SCell sspel = K.sSpel( aPoint, K.POS );

  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
        if ( *q1 != *q2 )
          {
            SCell s0 = K.sIncident( sspel, *q1, true );
            SCell s1 = K.sIncident( sspel, *q2, true );
            SCell l10 = K.sIncident( s0, *q2, true );
            SCell l01 = K.sIncident( s1, *q1, true );
            INFO( "D+_" << *q2 << "(D+_" << *q1 << "(V))=" << l10
              << " D+_" << *q1 << "(D+_" << *q2 << "(V))=" << l01
            );
            REQUIRE( l10 == K.sOpp( l01 ) );
          }
      }
}

///////////////////////////////////////////////////////////////////////////////
/** Testing direct incidence
 *
 * @tparam KSpace the Khalimsky space type.
 * @param  K      the Khalimsky space.
 * @param  aPoint a point where to test the incidences.
 */
template < typename KSpace >
void testDirectIncidence( KSpace const & K,
                          typename KSpace::Point const & aPoint
                        )
{
  INFO( "Testing direct Incidence in KSpace..." );

  using SCell = typename KSpace::SCell;
  using DirIterator = typename KSpace::DirIterator;

  SCell sspel = K.sSpel( aPoint, K.POS );
  
  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
        if ( *q1 != *q2 )
          {
            SCell s0 = K.sDirectIncident( sspel, *q1 );
            SCell l10 = K.sDirectIncident( s0, *q2 );
            SCell s1 = K.sDirectIncident( sspel, *q2 );
            SCell l01 = K.sDirectIncident( s1, *q1 );
            INFO( "Dd_" << *q2 << "(Dd_" << *q1 << "(V))=" << l10
              << " Dd_" << *q1 << "(Dd_" << *q2 << "(V))=" << l01
            );

            REQUIRE( l10 != l01 );
            REQUIRE( K.sSign( s0 ) == K.POS );
            REQUIRE( K.sSign( s1 ) == K.POS );
            REQUIRE( K.sSign( l10 ) == K.POS );
            REQUIRE( K.sSign( l01 ) == K.POS );
            REQUIRE( s0 == K.sIncident( sspel, *q1, K.sDirect( sspel, *q1 ) ) );
            REQUIRE( s1 == K.sIncident( sspel, *q2, K.sDirect( sspel, *q2 ) ) );
            REQUIRE( l10 == K.sIncident( s0, *q2, K.sDirect( s0, *q2 ) ) );
            REQUIRE( l01 == K.sIncident( s1, *q1, K.sDirect( s1, *q1 ) ) );
          }
      }
}

///////////////////////////////////////////////////////////////////////////////
/** Testing SurfelAdjacency
 *
 * @tparam KSpace the Khalimsky space type.
 * @param  K      the Khalimsky space.
 * @param  aPoint a point where to test the incidences.
 */
template <typename KSpace>
void testSurfelAdjacency( KSpace const & K,
                          typename KSpace::Point const & aPoint
                        )
{
  using SCell = typename KSpace::SCell;
  using Point = typename KSpace::Point;
  using Integer = typename KSpace::Integer;
  
  INFO( "Testing surfel adjacency ..." );
  SurfelAdjacency<KSpace::dimension> SAdj( true );
  
  INFO( "Testing surfel directness ..." );
  const SCell sspel = K.sSpel( Point::diagonal(1), K.POS );

  for ( Dimension k = 0; k < K.dimension; ++k )
    {
      SCell surfel = K.sIncident( sspel, k, true );
      SCell innerspel = K.sDirectIncident( surfel, K.sOrthDir( surfel ) );
      INFO( "spel=" << sspel << " surfel=" << surfel << " innerspel=" << innerspel );
      REQUIRE( sspel == innerspel );
      
      surfel = K.sIncident( sspel, k, false );
      innerspel = K.sDirectIncident( surfel, K.sOrthDir( surfel ) );
      INFO( "spel=" << sspel << " surfel=" << surfel << " innerspel=" << innerspel );
      REQUIRE( sspel == innerspel );
    }

  INFO( "Testing surfel neighborhood ..." );
  SurfelNeighborhood<KSpace> SN;
  SCell surfel = K.sIncident( sspel, 0, false );
  SN.init( &K, &SAdj, surfel );

  INFO( "Testing surface tracking ..." );
  using Space = SpaceND< KSpace::dimension, Integer >;
  using Domain = HyperRectDomain<Space>;
  using DigitalSet = typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type;

  const Domain domain( aPoint - Point::diagonal(3), aPoint + Point::diagonal(3) );
  DigitalSet shape_set( domain );
  const Point pcenter = Point::diagonal(0) + Point::base(0);
  Shapes<Domain>::addNorm1Ball( shape_set, pcenter, 1 );

  std::set<SCell> bdry;
  Surfaces<KSpace>::trackBoundary( bdry, K, SAdj, shape_set, surfel );
  REQUIRE( bdry.size() == ( 2*K.dimension*(2*K.dimension-1) ) );
  
  std::set<SCell> bdry_direct;
  Surfaces<KSpace>::trackClosedBoundary( bdry_direct, K, SAdj, shape_set, surfel );
  REQUIRE( bdry_direct.size() == ( 2*K.dimension*(2*K.dimension-1) ) );

  if ( K.dimension == 2 )
    {
      INFO( "Testing Board2D" );
      Board2D board;
      board.setUnit( LibBoard::Board::UCentimeter );
      board << SetMode( domain.className(), "Paving" ) << domain;
      for ( typename std::set<SCell>::const_iterator it = bdry_direct.begin(),
        it_end = bdry_direct.end(); it != it_end; ++it )
        board << *it;
      board.saveEPS( "cells-2.eps" );
      board.saveSVG( "cells-2.svg" );
    }
}

///////////////////////////////////////////////////////////////////////////////
/** Testing Cell drawing on Board.
 * @tparam KSpace the Khalimsky space type.
 * @param  K      the Khalimsky space.
 * @param  aPoint a point where to test the incidences.
 */



///////////////////////////////////////////////////////////////////////////////
// Test cases

TEST_CASE( "Checking concepts" )
{
  BOOST_CONCEPT_ASSERT(( concepts::CPreCellularGridSpaceND< KhalimskyPreSpaceND<2> > ));
  BOOST_CONCEPT_ASSERT(( concepts::CPreCellularGridSpaceND< KhalimskyPreSpaceND<3> > ));
  BOOST_CONCEPT_ASSERT(( concepts::CPreCellularGridSpaceND< KhalimskyPreSpaceND<4> > ));
  BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< KhalimskySpaceND<4> > ));
}

TEST_CASE( "2D Khalimsky pre-space", "[KPreSpace][2D]" )
{
  const KhalimskyPreSpaceND<2> K{};
  INFO( "Khalimsky space is " << K );

  testScan( K, {-1, -2}, {1, 2} );
  testIncidence( K, {0, 0} );
  testDirectIncidence( K, {0, 0} );
  testSurfelAdjacency( K, {0, 0} );
}

TEST_CASE( "3D Khalimsky pre-space", "[KPreSpace][3D]" )
{
  const KhalimskyPreSpaceND<3> K{};
  INFO( "Khalimsky space is " << K );

  testScan( K, {-2, -3, -4}, {2, 3, 4} );
  testIncidence( K, {0, 0, 0} );
  testDirectIncidence( K, {0, 0, 0} );
  testSurfelAdjacency( K, {0, 0, 0} );
}

TEST_CASE( "4D Khalimsky pre-space", "[KPreSpace][4D]" )
{
  const KhalimskyPreSpaceND<3> K{};
  INFO( "Khalimsky space is " << K );

  testScan( K, {-1, -2, -3, -4}, {1, 0, 1, -1} );
  testIncidence( K, {0, 0, 0, 0} );
  testDirectIncidence( K, {0, 0, 0, 0} );
}

TEST_CASE( "2D closed Khalimsky space", "[KSpace][2D][closed]" )
{
  KhalimskySpaceND<2> K;
  const bool spaceOK = K.init( {-2, -3}, {3, 3}, K.CLOSED );
  INFO( "Khalimsky space is " << K );
  REQUIRE( spaceOK == true );

  testScan( K, {-1, -2}, {1, 2} );
  testNeighborhood( K, {0, 0} );
  testNeighborhood( K, {-2, 3} );
  testFaces( K, {0, 0} );
  testFaces( K, {-2, 3} );
  testIncidence( K, {0, 0} );
  testDirectIncidence( K, {0, 0} );
  testSurfelAdjacency( K, {0, 0} );
}

TEST_CASE( "4D closed Khalimsky space", "[KSpace][4D][closed]" )
{
  KhalimskySpaceND<4> K;
  const bool spaceOK = K.init( {-2, -3, -1, 0}, {2, 3, 2, 3}, K.CLOSED );
  INFO( "Khalimsky space is " << K );
  REQUIRE( spaceOK == true );

  testScan( K, {-1, -2, 0, 1}, {1, 2, 1, 2} );
  testNeighborhood( K, {0, 0, 0, 0} );
  testNeighborhood( K, {-2, 3, -1, 3} );
  testFaces( K, {0, 0, 0, 0} );
  testFaces( K, {-2, 3, -1, 3} );
  testIncidence( K, {0, 0, 0, 0} );
  testDirectIncidence( K, {0, 0, 0, 0} );
}

TEST_CASE( "2D open Khalimsky space", "[KSpace][2D][open]" )
{
  KhalimskySpaceND<2> K;
  const bool spaceOK = K.init( {-2, -3}, {2, 3}, K.OPEN );
  INFO( "Khalimsky space is " << K );
  REQUIRE( spaceOK == true );

  testScan( K, {-1, -2}, {1, 2} );
  testNeighborhood( K, {0, 0} );
  testNeighborhood( K, {-2, 3} );
  testFaces( K, {0, 0} );
  testFaces( K, {-2, 3} );
  testIncidence( K, {0, 0} );
  testDirectIncidence( K, {0, 0} );
}

TEST_CASE( "2D periodic Khalimsky space", "[KSpace][2D][periodic]" )
{
  KhalimskySpaceND<2> K;
  const bool spaceOK = K.init( {-2, -3}, {2, 3}, K.PERIODIC );
  INFO( "Khalimsky space is " << K );
  REQUIRE( spaceOK == true );

  testScan( K, {-1, -2}, {1, 2} );
  testNeighborhood( K, {0, 0} );
  testNeighborhood( K, {-2, 3} );
  testFaces( K, {0, 0} );
  testFaces( K, {-2, 3} );
  testIncidence( K, {0, 3} );
  testDirectIncidence( K, {0, 3} );
}

TEST_CASE( "2D mixed Khalimsky space", "[KSpace][2D][closed][periodic]" )
{
  KhalimskySpaceND<2> K;
  const bool spaceOK = K.init( {-2, -3}, {2, 3}, { K.CLOSED, K.PERIODIC } );
  INFO( "Khalimsky space is " << K );
  REQUIRE( spaceOK == true );

  testScan( K, {-1, 2}, {1, 5} );
  testNeighborhood( K, {0, 0} );
  testNeighborhood( K, {-2, 4} );
  testFaces( K, {0, 0} );
  testFaces( K, {-2, 4} );
  testIncidence( K, {0, 3} );
  testDirectIncidence( K, {0, 3} );
}
