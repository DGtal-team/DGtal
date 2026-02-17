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
 * @file testIntegerConverter.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/04/10
 *
 * Functions for testing class IntegerConverter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/IntegerConverter.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegerConverter.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "Integer types sizes", "[integer_conversions]" )
{
  WHEN( "Checking integral types" )  {
    THEN( "Integral types are progressive" ) {
      CAPTURE( sizeof( int ) );
      CAPTURE( sizeof( long ) );
      CAPTURE( sizeof( long long ) );
      REQUIRE( sizeof( int ) == 4 );
      REQUIRE( sizeof( int ) <= sizeof( long ) );
      REQUIRE( sizeof( long ) <= sizeof( long long ) );
      REQUIRE( sizeof( long long ) == 8 );
    }
  }
}
SCENARIO( "IntegerConverter< 1, int32 >", "[integer_conversions]" )
{
  typedef IntegerConverter< 1, DGtal::int32_t > Converter;
  DGtal::int32_t    small_int32   = 0x12345678;
  DGtal::int64_t    small_int64   = 0x12345678L;
  DGtal::BigInteger small_bigint  = 0x12345678;
  WHEN( "Converting small integers" ) {
    DGtal::int32_t a = Converter::cast( small_int32 );
    DGtal::int32_t b = Converter::cast( small_int64 );
    DGtal::int32_t c = Converter::cast( small_bigint );
    THEN( "Their values are all identical" ) {
      REQUIRE( a == small_int32 );
      REQUIRE( a == b );
      REQUIRE( a == c );
    }
  }
  WHEN( "Converting medium integers" ) {
    DGtal::int64_t medium_int64  = 0x123456789ABCDEFL;
    DGtal::int32_t a = Converter::cast( medium_int64 );
    THEN( "The value is lost with a warning" ) {
      REQUIRE( DGtal::int64_t( a ) != medium_int64 );
    }
  }
}

SCENARIO( "IntegerConverter< 1, int64 >", "[integer_conversions]" )
{
  typedef IntegerConverter< 1, DGtal::int64_t > Converter;
  DGtal::int32_t    medium_int32  = DGtal::int32_t( 0x123456789ABCDEFL );
  DGtal::int64_t    medium_int64  = 0x123456789ABCDEFL;
  DGtal::BigInteger medium_bigint = 0x123456789ABCDEFL;
  WHEN( "Converting 64bits integers" ) {
    DGtal::int64_t a = Converter::cast( medium_int32 );
    DGtal::int64_t b = Converter::cast( medium_int64 );
    DGtal::int64_t c = Converter::cast( medium_bigint );
   THEN( "Only bigger integers are identical" ) {
      REQUIRE( a == medium_int32 );
      REQUIRE( a != b );
      REQUIRE( b == medium_int64 );
      REQUIRE( b == c );
   }
    THEN( "It gives the same results with NumberTraits" ) {
      DGtal::int64_t ap = NumberTraits<DGtal::int32_t>::castToInt64_t( medium_int32 );
      DGtal::int64_t bp = NumberTraits<DGtal::int64_t>::castToInt64_t( medium_int64 );
      DGtal::int64_t cp = NumberTraits<DGtal::BigInteger>::castToInt64_t( medium_bigint );
      REQUIRE( a == ap );
      REQUIRE( b == bp );
      REQUIRE( c == cp );
    }
  }
}


SCENARIO( "IntegerConverter< 1, BigInteger >", "[integer_conversions]" )
{
  typedef IntegerConverter< 1, DGtal::BigInteger > Converter;
  DGtal::int32_t    big_int32  = DGtal::int32_t( 0x123456789ABCDEFL );
  DGtal::int64_t    big_int64  = 0x123456789ABCDEFL;
  DGtal::BigInteger big_bigint = 0x123456789ABCDEFL;
  big_int32  *= big_int32;
  big_int64  *= big_int64;
  big_bigint *= big_bigint;
  WHEN( "Converting big integers" ) {
    DGtal::BigInteger a = Converter::cast( big_int32 );
    DGtal::BigInteger b = Converter::cast( big_int64 );
    DGtal::BigInteger c = Converter::cast( big_bigint );
    DGtal::BigInteger b_prime = big_int64;

    THEN( "Only bigger integers are identical" ) {
      REQUIRE( a == big_int32 );
      REQUIRE( a != b );
      REQUIRE( b == b_prime );
      REQUIRE( b != c );
      REQUIRE( c == big_bigint );
    }
  }
}
