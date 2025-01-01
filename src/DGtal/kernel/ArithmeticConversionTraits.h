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

#pragma once

/**
 * @file
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2018/09/19
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithmeticConversionTraits_RECURSES)
#error Recursive header files inclusion detected in ArithmeticConversionTraits.h
#else // defined(ArithmeticConversionTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticConversionTraits_RECURSES

#if !defined ArithmeticConversionTraits_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticConversionTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <type_traits>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithmeticConversionTraits
  /**
   * \brief Aim: Trait class to get result type of arithmetic binary operators
   *    between two given types.
   *
   * @tparam T      First operand type.
   * @tparam U      Second operand type.
   * @tparam Enable Internal type used for SFINAE (do not specify it).
   *
   * If arithmetic operations are valid between @a T and @a U types, it
   * provides a @a type typedef that is defined to the result type of such
   * operations.
   *
   * Otherwise, there is no @a type member.
   *
   * The idea is to be able to use binary operators (specifically the
   * arithmetic operators) with two different operand types and to identify
   * the best result type.
   *
   * For arithmetic types (`int`, `double`, ...), this trait class relies on
   * `std::common_type` standard type trait that use internal conversion rules
   * applied with ternary operator, i.e. the type of
   * `false ? std::declval<T>() : std::declval<U>()`. For arithmetic types, it
   * is equivalent to the type of `T() + U()`.
   * For example, those rules result in the following conversion chain:
   * int8_t -> int16_t -> int32_t -> int64_t -> float -> double.
   *
   * However, this class is not equivalent to `std::common_type` that defines
   * the common type that @a U and @a T can be implicitly converted to.
   * For example, a @ref PointVector added with an integer will result in a
   * @ref PointVector but there is no implicit conversion from an integer to
   * a @ref PointVector (and if there was such conversion, it may not lead to
   * the same result).
   *
   * This class is meant to be specialized when needed, provided that
   * (similarly to the specialization conditions of `std::common_type`):
   * - @a T or @a U are user-defined types,
   * - @a T and @a U used for the specialization aren't references, neither const.
   *
   * @see https://en.cppreference.com/w/cpp/types/common_type
   * @see https://en.cppreference.com/w/cpp/language/operator_arithmetic#Conversions
   */
  template <typename T, typename U, typename Enable = void>
  struct ArithmeticConversionTraits
  {
  };

  /** @brief Specialization in order to remove const specifiers and references from given types
   *
   * @see ArithmeticConversionTraits
   */
  template <typename T, typename U>
  struct ArithmeticConversionTraits< T, U,
      typename std::enable_if<
           ! std::is_same< T, typename std::remove_cv< typename std::remove_reference<T>::type >::type >::value
        || ! std::is_same< U, typename std::remove_cv< typename std::remove_reference<U>::type >::type >::value >::type >
    : ArithmeticConversionTraits<
        typename std::remove_cv< typename std::remove_reference<T>::type >::type,
        typename std::remove_cv< typename std::remove_reference<U>::type >::type >
  {
  };


  /** @brief Specialization for (fundamental) arithmetic types.
   *
   * Resulting type is deduced from usual arithmetic conversion using
   * std::common_type.
   *
   * @see ArithmeticConversionTraits
   */
  template <typename T, typename U>
  struct ArithmeticConversionTraits< T, U,
      typename std::enable_if<    std::is_arithmetic<T>::value
                               && std::is_arithmetic<U>::value >::type >
  {
    using type = typename std::common_type<T, U>::type; //! Arithmetic operation result type.
  };

  /** @brief Result type of arithmetic binary operators between two given types.
   *
   * @tparam T      First operand type.
   * @tparam U      Second operand type.
   *
   * @see ArithmeticConversionTraits
   */
  template <typename T, typename U>
  using ArithmeticConversionType = typename ArithmeticConversionTraits<T, U>::type;

  /** @brief Helper to determine if an arithmetic operation between two given
   *      types has a valid result type (ie is valid).
   *
   *  Without valid specialization, inherits from `std::false_type`.
   *
   *  @see ArithmeticConversionTraits
   */
  template <typename T, typename U, typename Enable = void>
  struct IsArithmeticConversionValid
    : std::false_type
  {
  };

  /** @brief Specialization when arithmetic operation between the two given
   *      type is valid.
   *
   *  Inherits from `std::true_type`.
   *
   *  @see IsArithmeticConversionValid
   *  @see ArithmeticConversionTraits
   */
  template <typename T, typename U>
  struct IsArithmeticConversionValid<T, U,
      typename std::conditional<false, ArithmeticConversionType<T, U>, void>::type >
    : std::true_type
  {
  };


  /** @brief Call constructor for the result type of an arithmetic operation.
   *
   * @tparam LHS    First operand type.
   * @tparam RHS    Second operand type.
   * @tparam Args   Types of the parameters forwarded to the constructor.
   *
   * @param lhs     First operand (only used for auto-deducing its type).
   * @param rhs     Second operand (only used for auto-deducing its type).
   * @param args    Parameters forwarded to the constructor.
   */
  template <
    typename LHS,
    typename RHS,
    typename... Args >
  inline
  ArithmeticConversionType<LHS, RHS>
  constructFromArithmeticConversion( LHS const& lhs, RHS const& rhs, Args &&... args )
    {
      boost::ignore_unused_variable_warning(lhs);
      boost::ignore_unused_variable_warning(rhs);

      return ArithmeticConversionType<LHS, RHS>( std::forward<Args>(args)... );
    }

#ifdef DGTAL_WITH_GMP
  /** @brief Specialization when first operand is a @ref BigInteger.
   *
   * @warning result type if set to BigInteger instead of the possible
   *    more complex __gmp_expr.
   *
   * @see ArithmeticConversionTraits
   */
  template <typename T, typename GMP1, typename GMP2>
  struct ArithmeticConversionTraits<T, __gmp_expr<GMP1, GMP2>,
      typename std::enable_if< std::is_integral<T>::value >::type >
  {
    using type = BigInteger;
  };

  /** @brief Specialization when second operand is a @ref BigInteger.
   *
   * @warning result type if set to BigInteger instead of the possible
   *    more complex __gmp_expr.
   *
   * @see ArithmeticConversionTraits
   */
  template <typename GMP1, typename GMP2, typename U>
  struct ArithmeticConversionTraits<__gmp_expr<GMP1, GMP2>, U,
      typename std::enable_if< std::is_integral<U>::value >::type >
  {
    using type = BigInteger;
  };

  /** @brief Specialization when both operands are @ref BigInteger.
   *
   * @warning result type if set to BigInteger instead of the possible
   *    more complex __gmp_expr.
   *
   * @see ArithmeticConversionTraits
   */
  template <typename GMPL1, typename GMPL2, typename GMPR1, typename GMPR2>
  struct ArithmeticConversionTraits<__gmp_expr<GMPL1, GMPL2>, __gmp_expr<GMPR1, GMPR2>>
  {
    using type = BigInteger;
  };
#endif

} // namespace DGtal

#endif // !defined ArithmeticConversionTraits_h

#undef ArithmeticConversionTraits_RECURSES
#endif // else defined(ArithmeticConversionTraits_RECURSES)
