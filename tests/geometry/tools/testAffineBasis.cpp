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
 * @file testAffineBasis.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/24
 *
 * Functions for testing class AffineBasis.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

std::random_device rd;
std::mt19937 g(rd());

template < typename Point >
std::vector< Point >
makeRandomVectors( int nb, int amplitude )
{
  std::uniform_int_distribution<int> U(-amplitude, amplitude);
  std::vector< Point > P;
  for ( auto n = 0; n < nb; ++n )
    {
      Point A;
      for ( auto i = 0; i < Point::dimension; i++ )
        A[ i ] = U( g );
      P.push_back( A );
    }
  return P;
}

template < typename Point >
std::vector< Point >
makeRandomLatticePointsFromDirVectors( int nb, const vector< Point>& V )
{
  std::uniform_int_distribution<int> U(-10, 10);
  vector< Point > P;
  int n = V[0].size();
  int m = V.size();
  Point A;
  for ( auto i = 0; i < n; i++ )
    A[ i ] = U( g );
  P.push_back( A );
  for ( auto k = 0; k < nb; k++ )
    {
      Point B = A;
      for ( auto i = 0; i < m; i++ )
        {
          int l = U( g );
          B += l * V[ i ];
        }
      P.push_back( B );
    }
  std::shuffle( P.begin(), P.end(), g );
  return P;
}

template <typename TPoint>
bool sameDirection( const TPoint& a, const TPoint& b )
{
  return ( a[ 0 ] == b[ 0 ] ) ? ( a == b ) : ( a == -b );
}

template <typename TPoint>
std::pair<TPoint, TPoint>
boundingBox( const std::vector< TPoint >& v )
{
  TPoint l, u;
  if ( ! v.empty() )
    {
      l = u = v[ 0 ];
      for ( auto i = 1; i < v.size(); i++ )
        {
          l = l.inf( v[ i ] );
          u = u.sup( v[ i ] );
        }
    }
  return std::make_pair( l, u );
}

template <typename T, typename C>
T
determinant( const PointVector<2, T, C>& u,
     const PointVector<2, T, C>& v )
{
  return u [ 0 ] * v [ 1 ] - u [ 1 ] * v [ 0 ];
}

template <typename T, typename C>
T
determinant( const PointVector<3, T, C>& u,
             const PointVector<3, T, C>& v,
             const PointVector<3, T, C>& w )
{
  typedef PointVector<2,T,C> P2;
  return u[ 0 ] * determinant( P2( v[1], v[2] ), P2( w[1], w[2] ) )
    -    u[ 1 ] * determinant( P2( u[0], u[2] ), P2( w[0], w[2] ) )
    +    u[ 2 ] * determinant( P2( u[0], u[1] ), P2( v[0], v[1] ) );
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Point2i > unit tests", "[affine_basis][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;
  GIVEN( "Given B = (0,0) + { (8,2), (-4,-1),  (-8,-2), (16,4), (200,50) } of affine dimension 1" ) {
    Point o( 0, 0 );
    std::vector<Point> X
      = { Point(8,2), Point(-4,-1), Point(-8,-2), Point(16,4), Point(200,50) };
    Basis B( o, X, Basis::Type::SCALED_REDUCED );
    THEN( "When reduced, it has dimension 1" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 1 );
    }
    THEN( "When reduced, it is the vector (-4,-1) or (4,1)" ) {
      Point b0 = B.basis()[ 0 ];
      CAPTURE( b0 );
      REQUIRE( ((b0 == Point(-4,-1)) || (b0 == Point(4,1))) );
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 3D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Z3 > LLL tests", "[affine_basis][3d][LLL]" )
{
  typedef SpaceND< 3, int>                 Space;
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >          Affine;
  typedef AffineBasis< Point >             Basis;

  WHEN( "Using basis B = (1, 0, -2) (1, 0, -1)" ) {
    std::vector< Point > b = { Point( 0, 0, 0), Point(1, 0, -2), Point(1, 0, -1) };
    const auto [ o, B ] = Affine::affineBasis( b );
    Point e  = functions::computeIndependentVector( B );
    Basis AB( b, Basis::Type::LLL_REDUCED );
    bool parallel = AB.isParallel( e );
    const auto [ d, L, r ] = AB.decomposeVector( e );
    CAPTURE( B ); 
    CAPTURE( AB.basis() ); 
    CAPTURE( d ); 
    CAPTURE( L );
    CAPTURE( r );
    CAPTURE( e );
    REQUIRE( ! parallel );
    REQUIRE( r != Point::zero );
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 4D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Point4i > unit tests", "[affine_basis][4i]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X, Basis::Type::SCALED_REDUCED );
    THEN( "When reduced, it has dimension 1" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 1 );
    }
    THEN( "When reduced, it is the vector V[0] or -V[0]" ) {
      CAPTURE( V );
      Point b0 = B.basis()[ 0 ];
      CAPTURE( b0 );
      REQUIRE( ((b0 == V[0]) || (b0 == -V[0])) );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X, Basis::Type::SCALED_REDUCED );
    THEN( "When reduced, basis has dimension 2" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 2 );
    }
    THEN( "When reduced, basis spans vectors V[i] or -V[i]" ) {
      CAPTURE( V );
      Point b0 = B.basis()[ 0 ];
      Point b1 = B.basis()[ 1 ];
      CAPTURE( b0 );
      CAPTURE( b1 );
      REQUIRE( B.isParallel( V[ 0 ] ) );
      REQUIRE( B.isParallel( V[ 1 ] ) );
    }
    THEN( "every point of X has rational coordinates with no remainder" ) {
      unsigned int nb_ok = 0;
      for ( auto p : X )
        {
          const auto [d, lambda, rem ] = B.decompose( p );
          nb_ok += ( rem == Point::zero ) ? : 1;
          if ( rem != Point::zero )
            std::cout << "p=" << p << " d=" << d
                      << " lambda=" << lambda << " rem=" << rem << "\n";
        }
      REQUIRE( nb_ok == X.size() );
    }
    THEN( "every lattice point can be written as a linear combination" ) {
      auto Y = makeRandomVectors<Point>( 20, 10 );
      unsigned int nb_ok = 0;
      for ( auto y : Y )
        {
          const auto p = y + B.first;
          const auto [d, lambda, rem ] = B.decompose( p );
          auto q = B.recompose( d, lambda, rem );
          nb_ok += ( p == q ) ? : 1;
          if ( p != q )
            std::cout << "p=" << p << " d=" << d
                      << " lambda=" << lambda << " rem=" << rem
                      << " q=" << q << "\n";
        }
      REQUIRE( nb_ok == Y.size() );
    }
  }
}

SCENARIO( "AffineBasis< Point4i > projection tests", "[affine_basis][4i][4d]" )
{
  typedef SpaceND< 4, int64_t >            Space;      
  typedef Space::Point                     Point;
  typedef SpaceND< 2, int64_t >            Space2;      
  typedef Space2::Point                    PPoint;
  typedef AffineBasis< Point >             Basis;
  typedef Space::RealPoint                 RealPoint;
  typedef Space2::RealPoint                PRealPoint;
  typedef AffineBasis< RealPoint >         RealBasis;

  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors, and Y the same set but with real coordinates" ) {
    std::vector< Point > V = { Point{ 3, 4, 0, 2 }, Point{ -2, -1, 5, -7 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X, Basis::Type::SCALED_REDUCED );
    std::vector< RealPoint > Y( X.size() );
    for ( auto i = 0; i < Y.size(); i++ )
      Basis::transform( Y[ i ], X[ i ] );
    RealBasis RB( Y, RealBasis::Type::SCALED_REDUCED );
    std::vector< PPoint >     pX;
    std::vector< PRealPoint > pY;
    auto lcm  = B .projectPoints( pX, X );
    auto rlcm = RB.projectPoints( pY, Y );
    auto rbox = boundingBox( pY );
    double factor = ( rbox.second - rbox.first ).squaredNorm();
    THEN( "When reduced, their affine bases has same dimension 2" ) {
      CAPTURE( B.basis() );
      CAPTURE( RB.basis() );
      REQUIRE( B.dimension() == 2 );
      REQUIRE( RB.dimension() == 2 );
    }
    THEN( "Their projections have the same geometry (i.e. orientations within points)" ) {
      CAPTURE( lcm );
      CAPTURE( rlcm );
      CAPTURE( pX );
      CAPTURE( pY );
      REQUIRE( pX.size() == pY.size() );
      // Computing arbitrary determinants between triplets of points
      const std::size_t nb = 2000;
      std::size_t    nb_ok = 0;
      const double     eps = 1e-12 * factor;
      for ( auto i = 0; i < nb; i++ )
        {
          const std::size_t j = rand() % pX.size();
          const std::size_t k = rand() % pX.size();
          const std::size_t l = rand() % pX.size();
          const auto u    = pX[ k ] - pX[ j ];
          const auto v    = pX[ l ] - pX[ j ];
          const auto ru   = pY[ k ] - pY[ j ];
          const auto rv   = pY[ l ] - pY[ j ];
          const auto det  = u [ 0 ] * v [ 1 ] - u [ 1 ] * v [ 0 ];
          const auto rdet = ru[ 0 ] * rv[ 1 ] - ru[ 1 ] * rv[ 0 ];
          if ( rdet > eps )       nb_ok += ( det >  0 ) ? 1 : 0;
          else if ( rdet < -eps ) nb_ok += ( det <  0 ) ? 1 : 0;
          else                    nb_ok += ( det == 0 ) ? 1 : 0;
          if ( nb_ok != i+1 )
            {
              std::cout << "eps=" << eps << " factor=" << factor << "\n";
              std::cout << "u =" << u  << " v =" << v << " det ="  << det << "\n";
              std::cout << "ru=" << ru << " rv=" << rv << " rdet=" << rdet << "\n";
              break;
            }
        }
      REQUIRE( nb_ok == nb );
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 5D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Point5i > unit tests", "[affine_basis][5i]" )
{
  typedef SpaceND< 5, BigInteger >            Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >          Affine;
  typedef AffineBasis< Point >             Basis;

  unsigned int nb           = 0;
  unsigned int nb_s_parallel_l = 0;
  unsigned int nb_equal_big = 0;
  unsigned int nb_equal_N   = 0;
  unsigned int nb_equal_Nr  = 0;
  for ( auto n = 0; n < 10; n++ )
    {
      std::vector< Point > X = makeRandomVectors<Point>( 5, 100 );
      std::vector< Point > Y;
      for ( auto i = 1; i < X.size(); i++ ) Y.push_back( X[ i ] - X[ 0 ] );
      Basis B ( X[ 0 ], Y, Basis::Type::LLL_REDUCED );
      Basis Br( X, Basis::Type::SCALED_REDUCED );
      if ( functions::computeAffineDimension( X ) != 4 ) continue;
      auto N = functions::computeOrthogonalVectorToBasis( B.basis() );
      auto Nr = functions::computeOrthogonalVectorToBasis( Br.basis() );
      auto N_big  = Affine::template orthogonalVector<BigInteger>( B.basis() );
      auto Nr_big = Affine::template orthogonalVector<BigInteger>( Br.basis() );
      auto N_cast  = N_big;
      auto Nr_cast = Nr_big;
      for ( auto i = 0; i < 5; i++ ) N_cast[ i ]  = N[ i ];
      for ( auto i = 0; i < 5; i++ ) Nr_cast[ i ] = Nr[ i ];
      nb_equal_big += sameDirection( N_big,   Nr_big ) ? 1 : 0;
      nb_equal_N   += sameDirection( N_cast,  N_big  ) ? 1 : 0;
      nb_equal_Nr  += sameDirection( Nr_cast, Nr_big ) ? 1 : 0;
      nb           += 1;
      nb_s_parallel_l  += Br.isParallel( B  ) ? 1 : 0;
      if ( ! Br.isParallel( B ) )
        {
          std::cout << "* Br is not // to B:\n" << Br << "\n";
          for ( auto v : B.basis() )
            std::cout << "  v=" << v << " //=" << ( Br.isParallel( v ) ? "True" : "False" )
                      << "\n";
        }
      // std::cout << B << "\n" << Br << "\n";
      // std::cout << "N    =" << N << " Nr    =" << Nr << "\n";
      // std::cout << "N_big=" << N_big << " Nr_big=" << Nr_big << "\n";
    }
  THEN( "Normals with big integers are the same" ) {  
    REQUIRE( nb_equal_big == nb );
  }
  THEN( "There is less overflow in orthogonal vector computation using non reduced basis" ) {
    REQUIRE( nb_equal_N >= nb_equal_Nr );
  }
  THEN( "The LLL-reduced basis and the scaled-reduced basis are parallel" ) {
    REQUIRE( nb_s_parallel_l == nb );
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 10D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Z10 > LLL tests", "[affine_basis][10d][LLL]" )
{
  typedef SpaceND< 10, int64_t>            Space;
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;


  WHEN( "Using basis is unimodular" ) {
    // when the matrix is unimodular, outputs canonic vectors.
    std::vector< Point > B = {
      Point{   -3,   10,   47,   61,  -53, -126,  713,  601,-1476, 1569},
      Point{    2,   -7,  -33,  -43,   37,   89, -502, -425, 1047,-1103},
      Point{   -3,   11,   53,   69,  -59, -142,  800,  677,-1663, 1764},
      Point{    1,   -9,  -48,  -63,   52,  130, -727, -623, 1543,-1604},
      Point{   -2,    9,   48,   63,  -49, -124,  680,  583,-1409, 1533},
      Point{    5,  -25, -118, -163,  113,  334,-1761,-1595, 4030,-3838},
      Point{   -3,   17,   85,  118,  -84, -245, 1297, 1173,-2974, 2824},
      Point{    5,  -24, -119, -156,  126,  321,-1782,-1534, 3799,-3921},
      Point{    2,  -10,  -44,  -65,   42,  137, -699, -659, 1713,-1489},
      Point{    1,   -5,  -23,  -27,   33,   64, -405, -315,  784, -868}
    };
    Point o = Point::zero;
    Basis S( o, B, Basis::Type::SCALED_REDUCED );
    Basis L( o, B, Basis::Type::LLL_REDUCED );
    THEN( "The LLL-reduced basis is canonic" ) {
      const auto&  V    = L.basis();
      unsigned int nb   = 0;
      unsigned int nbok = 0;
      for ( auto i = 0; i < V.size(); i++ )
      {
        nbok += V[ i ].norm1() == 1 ? : 0;
        nb++;
      }
      CAPTURE( L.basis() );
      REQUIRE( nbok == nb );
    }
    THEN( "The scaled-reduced basis is triangular" ) {
      const auto&  V    = S.basis();
      unsigned int nb   = 0;
      unsigned int nbok = 0;
      for ( auto i = 0; i < V.size(); i++ )
        for ( auto j = 0; j < i; j++ )        
          {
            nbok += ( V[ i ][ j ] == 0 ) ? 1 : 0;
            nb++;
          }
      CAPTURE( S.basis() );
      REQUIRE( nbok == nb );
    }
    THEN( "The LLL-reduced basis and the scaled-reduced basis are parallel" ) {
      // REQUIRE( L.isParallel( S ) );
      REQUIRE( S.isParallel( L ) );
    }
  }
}


SCENARIO( "AffineBasis< Point5i > projection tests", "[affine_basis][5i][5d]" )
{
  typedef SpaceND< 5, int64_t >            Space;      
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;
  typedef Space::RealPoint                 RealPoint;
  typedef AffineBasis< RealPoint >         RealBasis;

  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors, and Y the same set but with real coordinates" ) {
    typedef SpaceND< 2, int64_t >            Space2;      
    typedef Space2::Point                    PPoint;
    typedef Space2::RealPoint                PRealPoint;
    std::vector< Point > V = { Point{ 3, 4, 0, 2, -5 }, Point{ -2, -1, 5, -7, 1 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X, Basis::Type::SCALED_REDUCED );
    std::vector< RealPoint > Y( X.size() );
    for ( auto i = 0; i < Y.size(); i++ )
      Basis::transform( Y[ i ], X[ i ] );
    RealBasis RB( Y, RealBasis::Type::SCALED_REDUCED );
    std::vector< PPoint >     pX;
    std::vector< PRealPoint > pY;
    auto lcm  = B .projectPoints( pX, X );
    auto rlcm = RB.projectPoints( pY, Y );
    auto rbox = boundingBox( pY );
    double factor = ( rbox.second - rbox.first ).squaredNorm();
    THEN( "When reduced, their affine bases has same dimension 2" ) {
      CAPTURE( B.basis() );
      CAPTURE( RB.basis() );
      REQUIRE( B.dimension() == 2 );
      REQUIRE( RB.dimension() == 2 );
    }
    THEN( "Their projections have the same geometry (i.e. orientations within points)" ) {
      CAPTURE( lcm );
      CAPTURE( rlcm );
      CAPTURE( pX );
      CAPTURE( pY );
      REQUIRE( pX.size() == pY.size() );
      // Computing arbitrary determinants between triplets of points
      const std::size_t nb = 200;
      std::size_t    nb_ok = 0;
      const double     eps = 1e-12 * factor;
      for ( auto i = 0; i < nb; i++ )
        {
          const std::size_t j = rand() % pX.size();
          const std::size_t k = rand() % pX.size();
          const std::size_t l = rand() % pX.size();
          const auto u    = pX[ k ] - pX[ j ];
          const auto v    = pX[ l ] - pX[ j ];
          const auto ru   = pY[ k ] - pY[ j ];
          const auto rv   = pY[ l ] - pY[ j ];
          const auto det  = determinant( u, v );//u [ 0 ] * v [ 1 ] - u [ 1 ] * v [ 0 ];
          const auto rdet = determinant(ru,rv );//ru[ 0 ] * rv[ 1 ] - ru[ 1 ] * rv[ 0 ];
          if ( rdet > eps )       nb_ok += ( det >  0 ) ? 1 : 0;
          else if ( rdet < -eps ) nb_ok += ( det <  0 ) ? 1 : 0;
          else                    nb_ok += ( det == 0 ) ? 1 : 0;
          if ( nb_ok != i+1 )
            {
              std::cout << "eps=" << eps << " factor=" << factor << "\n";
              std::cout << "u =" << u  << " v =" << v << " det ="  << det << "\n";
              std::cout << "ru=" << ru << " rv=" << rv << " rdet=" << rdet << "\n";
              break;
            }
        }
      REQUIRE( nb_ok == nb );
    }
  }

  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 3 lattice vectors, and Y the same set but with real coordinates" ) {
    typedef SpaceND< 3, int64_t >            Space3;      
    typedef Space3::Point                    PPoint;
    typedef Space3::RealPoint                PRealPoint;
    std::vector< Point > V = { Point{ 3, 4, 0, 2, 5 },
                               Point{ -2, -1, 5, -7, 1 },
                               Point{ -5, 2, 11, 1, 4 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X, Basis::Type::SCALED_REDUCED );
    std::vector< RealPoint > Y( X.size() );
    for ( auto i = 0; i < Y.size(); i++ )
      Basis::transform( Y[ i ], X[ i ] );
    RealBasis RB( Y, RealBasis::Type::SCALED_REDUCED );
    std::vector< PPoint >     pX;
    std::vector< PRealPoint > pY;
    auto lcm  = B .projectPoints( pX, X );
    auto rlcm = RB.projectPoints( pY, Y );
    auto rbox = boundingBox( pY );
    double factor = ( rbox.second - rbox.first ).squaredNorm();
    THEN( "When reduced, their affine bases has same dimension 2" ) {
      CAPTURE( B.basis() );
      CAPTURE( RB.basis() );
      REQUIRE( B.dimension() == 3 );
      REQUIRE( RB.dimension() == 3 );
    }
    THEN( "Their projections have null remainder" ) {
      CAPTURE( B.basis() );
      unsigned int nb_null_rem = 0;
      for ( auto i = 0; i < X.size(); i++ )
        {
          const auto [ d, L, r ] = B.decomposeVector( X[i] - B.origin() );
          nb_null_rem += ( r == Point::zero ) ? 1 : 0;
          if ( r != Point::zero )
            std::cout << (X[i] - B.origin()) << " => " << d << "/" << L << "/" << r << "\n";
        }
      REQUIRE( nb_null_rem == X.size() );
    }
    THEN( "Their projections have the same geometry (i.e. orientations within points)" ) {
      CAPTURE( B.basis() );
      CAPTURE( RB.basis() );
      CAPTURE( lcm );
      CAPTURE( rlcm );
      CAPTURE( pX );
      CAPTURE( pY );
      REQUIRE( pX.size() == pY.size() );
      // Computing arbitrary determinants between quadruplets of points
      const std::size_t nb = 200;
      std::size_t    nb_ok = 0;
      const double     eps = 1e-12 * factor;
      for ( auto i = 0; i < nb; i++ )
        {
          const std::size_t j = rand() % pX.size();
          const std::size_t k = rand() % pX.size();
          const std::size_t l = rand() % pX.size();
          const std::size_t m = rand() % pX.size();
          const auto u    = pX[ k ] - pX[ j ];
          const auto v    = pX[ l ] - pX[ j ];
          const auto w    = pX[ m ] - pX[ j ];
          const auto ru   = pY[ k ] - pY[ j ];
          const auto rv   = pY[ l ] - pY[ j ];
          const auto rw   = pY[ m ] - pY[ j ];
          const auto det  = determinant( u, v, w );
          const auto rdet = determinant( ru, rv, rw );
          if ( rdet > eps )       nb_ok += ( det >  0 ) ? 1 : 0;
          else if ( rdet < -eps ) nb_ok += ( det <  0 ) ? 1 : 0;
          else                    nb_ok += ( det == 0 ) ? 1 : 0;
          if ( nb_ok != i+1 )
            {
              std::cout << "eps=" << eps << " factor=" << factor << "\n";
              std::cout << "u =" << u << " v =" << v << " w =" << w
                        << " det ="  << det << "\n";
              std::cout << "ru=" << ru << " rv=" << rv << " rw=" << rw
                        << " rdet=" << rdet << "\n";
              break;
            }
        }
      REQUIRE( nb_ok == nb );
    }
  }

}




