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
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2019/02/21
 *
 * This file is part of the DGtal library.
 */

#include <limits>

#include "DGtal/base/BasicTypes.h"
#include "DGtal/kernel/NumberTraits.h"

#include "DGtalCatch.h"

/// Transform a std::integral_constant<bool, value> to the corresponding DGtal tag.
template <typename T>
struct IntegralCstToTag
{
  using type = DGtal::TagUnknown;
};

template <>
struct IntegralCstToTag<std::true_type>
{
  using type = DGtal::TagTrue;
};

template <>
struct IntegralCstToTag<std::false_type>
{
  using type = DGtal::TagFalse;
};

/// Transform a boolean value to the corresponding DGtal tag.
template <bool V>
struct ValueToTag
{
  using type = DGtal::TagUnknown;
};

template <>
struct ValueToTag<true>
{
  using type = DGtal::TagTrue;
};

template <>
struct ValueToTag<false>
{
  using type = DGtal::TagFalse;
};

#define REQUIRE_SAME_TAG(a, b)    REQUIRE( (std::is_same<a, typename IntegralCstToTag<b>::type>::value) )
#define REQUIRE_SAME_VALUE(a, b)  REQUIRE( (std::is_same<a, typename ValueToTag<b>::type>::value) )
#define REQUIRE_SAME_TYPE(a, b)   REQUIRE( (std::is_same<a, b>::value) )

/// Check that we can make a reference to the static attributes ZERO and ONE (ODR-use).
template <typename T>
void checkParamRef(T const&)
{
}

/// Check main traits for a fundamental type
template <typename T>
void checkFundamentalType()
{
  using NT = typename DGtal::NumberTraits<T>;
  using NL = typename std::numeric_limits<T>;

  REQUIRE_SAME_VALUE( typename NT::IsBounded,  NL::is_bounded );
  REQUIRE_SAME_VALUE( typename NT::IsSigned, NL::is_signed );
  REQUIRE_SAME_VALUE( typename NT::IsUnsigned, ! NL::is_signed );
  REQUIRE_SAME_VALUE( typename NT::IsIntegral, NL::is_integer );
  REQUIRE_SAME_VALUE( typename NT::IsSpecialized, true );
  REQUIRE_SAME_TYPE( typename NT::ReturnType, T );
  REQUIRE_SAME_TYPE( typename std::decay<typename NT::ParamType>::type, T );

  REQUIRE( NT::ZERO == T(0) );
  REQUIRE( NT::ONE  == T(1) );

  REQUIRE( NT::zero() == T(0) );
  REQUIRE( NT::one()  == T(1) );

  REQUIRE( NT::min() == NL::min() );
  REQUIRE( NT::max() == NL::max() );
  REQUIRE( NT::digits() == NL::digits );
  REQUIRE( NT::isBounded() == (NL::is_bounded ? DGtal::BOUNDED : DGtal::UNBOUNDED) );
  REQUIRE( NT::isSigned() == (NL::is_signed ? DGtal::SIGNED : DGtal::UNSIGNED) );

  REQUIRE( NT::castToInt64_t(T(3.25)) == 3 );
  REQUIRE( NT::castToDouble(T(3.25)) == (NL::is_integer ? 3. : 3.25) );

  checkParamRef(NT::ZERO);
  checkParamRef(NT::ONE);
}

/// Check traits for a fundamental integer type
template <typename T>
void checkFundamentalIntegerType()
{
  checkFundamentalType<T>();

  using NT = typename DGtal::NumberTraits<T>;

  REQUIRE_SAME_TYPE( typename NT::SignedVersion, typename std::make_signed<T>::type );
  REQUIRE_SAME_TYPE( typename NT::UnsignedVersion, typename std::make_unsigned<T>::type );

  REQUIRE( NT::even(T(42)) == true );
  REQUIRE( NT::even(T(43)) == false );
  REQUIRE( NT::odd(T(42)) == false );
  REQUIRE( NT::odd(T(43)) == true );
}

/// Check traits for a fundamental float type
template <typename T>
void checkFundamentalFloatType()
{
  checkFundamentalType<T>();
}

#define TEST_TYPE_TRAITS( test, a )         TEST_CASE( #a ) { test<a>(); }
#define TEST_FUNDAMENTAL_INTEGER_TYPE( a )  TEST_TYPE_TRAITS( checkFundamentalIntegerType, a )
#define TEST_FUNDAMENTAL_FLOAT_TYPE( a )    TEST_TYPE_TRAITS( checkFundamentalFloatType, a )

TEST_FUNDAMENTAL_INTEGER_TYPE(   signed char )
TEST_FUNDAMENTAL_INTEGER_TYPE( unsigned char )
TEST_FUNDAMENTAL_INTEGER_TYPE(   signed short )
TEST_FUNDAMENTAL_INTEGER_TYPE( unsigned short )
TEST_FUNDAMENTAL_INTEGER_TYPE(   signed int )
TEST_FUNDAMENTAL_INTEGER_TYPE( unsigned int )
TEST_FUNDAMENTAL_INTEGER_TYPE(   signed long int )
TEST_FUNDAMENTAL_INTEGER_TYPE( unsigned long int )
TEST_FUNDAMENTAL_INTEGER_TYPE(   signed long long int )
TEST_FUNDAMENTAL_INTEGER_TYPE( unsigned long long int )

TEST_FUNDAMENTAL_INTEGER_TYPE(  int8_t )
TEST_FUNDAMENTAL_INTEGER_TYPE( uint8_t )
TEST_FUNDAMENTAL_INTEGER_TYPE(  int16_t )
TEST_FUNDAMENTAL_INTEGER_TYPE( uint16_t )
TEST_FUNDAMENTAL_INTEGER_TYPE(  int32_t )
TEST_FUNDAMENTAL_INTEGER_TYPE( uint32_t )
TEST_FUNDAMENTAL_INTEGER_TYPE(  int64_t )
TEST_FUNDAMENTAL_INTEGER_TYPE( uint64_t )

TEST_FUNDAMENTAL_FLOAT_TYPE( float )
TEST_FUNDAMENTAL_FLOAT_TYPE( double )
TEST_FUNDAMENTAL_FLOAT_TYPE( long double )


/// Check traits for a BigInteger
TEST_CASE( "BigInteger" )
{
  using T = DGtal::BigInteger;
  using NT = typename DGtal::NumberTraits<T>;

  REQUIRE_SAME_VALUE( typename NT::IsBounded,     false );
  REQUIRE_SAME_VALUE( typename NT::IsSigned,      true );
  REQUIRE_SAME_VALUE( typename NT::IsUnsigned,    true );
  REQUIRE_SAME_VALUE( typename NT::IsIntegral,    true );
  REQUIRE_SAME_VALUE( typename NT::IsSpecialized, true );

  REQUIRE_SAME_TYPE( typename NT::ReturnType, T );
  REQUIRE_SAME_TYPE( typename std::decay<NT::ParamType>::type, T );

  REQUIRE( NT::zero() == T(0) );
  REQUIRE( NT::one() == T(1) );

  REQUIRE( NT::even(T(42)) == true );
  REQUIRE( NT::even(T(43)) == false );
  REQUIRE( NT::odd(T(42)) == false );
  REQUIRE( NT::odd(T(43)) == true );

  REQUIRE( NT::isBounded() == DGtal::UNBOUNDED );
  REQUIRE( NT::isSigned() == DGtal::SIGNED );

  REQUIRE( NT::castToInt64_t(T(3.25)) == 3 );
  REQUIRE( NT::castToDouble(T(3.25)) == 3. );

  checkParamRef(NT::ZERO);
  checkParamRef(NT::ONE);
}
