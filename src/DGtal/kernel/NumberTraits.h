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
 * @file NumberTraits.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/03
 *
 * Header file for module NumberTraits.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NumberTraits_RECURSES)
#error Recursive header files inclusion detected in NumberTraits.h
#else // defined(NumberTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NumberTraits_RECURSES

#if !defined NumberTraits_h
/** Prevents repeated inclusion of headers. */
#define NumberTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <type_traits>
#include <limits>

#include <boost/call_traits.hpp>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /// Bounding type of a number
  enum BoundEnum  {BOUNDED = 0, UNBOUNDED = 1, BOUND_UNKNOWN = 2};

  /// Sign type of a number
  enum SignEnum   {SIGNED = 0, UNSIGNED = 1, SIGN_UNKNOWN = 2};


/////////////////////////////////////////////////////////////////////////////
// template class NumberTraits and NumberTraitsImpl

  /**
   * Description of template class 'NumberTraitsImpl' <p>
   * \brief Aim: The traits class for all models of Cinteger (implementation)
   *
   * Since CInteger describes the concept Integer, this class is used
   * by models of CIinteger to specialize some definitions related to
   * Integer. For instance it defines whether a given Integer is
   * signed or not and what is signed/unsigned version.
   *
   * This is the fallback declaration for unknown types
   * (IsSpecialized points to TagFalse).
   *
   * @tparam T      Number type.
   * @tparam Enable Internal template parameter used for SFINAE.
   *
   * @see NumberTraits
   */
  template <typename T, typename Enable = void>
  struct NumberTraitsImpl
  {
    // ----------------------- Associated types ------------------------------
    typedef TagUnknown IsBounded;   ///< Is the number bounded.
    typedef TagUnknown IsUnsigned;  ///< Is the number unsigned.
    typedef TagUnknown IsSigned;    ///< Is the number signed.
    typedef TagUnknown IsIntegral;  ///< Is the number of integral type.
    typedef TagFalse IsSpecialized; ///< Is that a number type with specific traits.
    typedef T SignedVersion;        ///< Alias to the signed version of the number type.
    typedef T UnsignedVersion;      ///< Alias to the unsigned version of the number type.
    typedef T ReturnType;           ///< Alias to the type that should be used as return type.

    /** @brief Defines a type that represents the "best" way to pass
     *  a parameter of type T to a function.
     */
    typedef typename boost::call_traits<T>::param_type ParamType;


    /// Constant Zero.
    static const T ZERO = T(0);

    /// Constant One.
    static const T ONE  = T(1);

    /// Return the zero of this integer.
    static ReturnType zero();

    /// Return the one of this integer.
    static ReturnType one();

    /** @brief
     * Return the minimum possible value for this type of integer or
     * ONE if not bounded or unknown.
     */
    static ReturnType min();

    /** @brief
     * Return the maximum possible value for this type of integer or
     * ZERO if not bounded or unknown.
     */
    static ReturnType max();

    /** @brief
     * Return the number of significant binary digits for this integer type,
     * or 0 if unbounded or unknown.
     */
    static unsigned int digits();

    /** @brief Return the bounding type of the number.
     *
     * @return BOUNDED, UNBOUNDED, or BOUND_UNKNOWN.
     */
    static BoundEnum isBounded();

    /** @brief Return the sign type of the number.
     *
     * @return SIGNED, UNSIGNED or SIGN_UNKNOWN.
     */
    static SignEnum isSigned();

    /** @brief
     * Cast method to DGtal::int64_t (for I/O or board export uses
     * only).
     */
    static DGtal::int64_t castToInt64_t(const T & aT)
    {
      return static_cast<DGtal::int64_t>(aT);
    }

    /** @brief
     * Cast method to DGtal::uint64_t (for I/O or board export uses
     * only).
     */
    static inline constexpr
    DGtal::uint64_t castToUInt64_t(const T & aT) noexcept
    {
      return static_cast<DGtal::uint64_t>(aT);
    }

    /** @brief
     * Cast method to double (for I/O or board export uses
     * only).
     */
    static double castToDouble(const T & aT)
    {
      return static_cast<double>(aT);
    }
    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is even.
     */
    static bool even( ParamType aT )
    {
      return ( aT & ONE ) == ZERO;
    }

    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is odd.
     */
    static bool odd( ParamType aT )
    {
      return ( aT & ONE ) != ZERO;
    }

  }; // end of class NumberTraitsImpl

  // Definition of the static attributes in order to allow ODR-usage.
  template <typename T, typename Enable> const T NumberTraitsImpl<T, Enable>::ZERO;
  template <typename T, typename Enable> const T NumberTraitsImpl<T, Enable>::ONE;

  // Some custom structs to factorize the code.
  namespace details
  {

    /// Convert a boolean to the corresponding DGtal tag (TagTrue or TagFalse).
    template <bool Value>
    struct BoolToTag
    {
      using type = DGtal::TagTrue;
    };

    template <>
    struct BoolToTag<false>
    {
      using type = DGtal::TagFalse;
    };

    /// NumberTraits common part for fundamental integer and floating-point types.
    template <typename T>
    struct NumberTraitsImplFundamental
    {
    private:
      using NL = std::numeric_limits<T>; ///< Type alias to std::numeric_limits

    public:
      // ----------------------- Associated types ------------------------------
      using IsBounded     = typename BoolToTag<NL::is_bounded>::type; ///< Is the number bounded.
      using IsUnsigned    = typename BoolToTag<!NL::is_signed>::type; ///< Is the number unsigned.
      using IsSigned      = typename BoolToTag<NL::is_signed>::type;  ///< Is the number signed.
      using IsIntegral    = typename BoolToTag<NL::is_integer>::type; ///< Is the number of integral type.
      using IsSpecialized = TagTrue;  ///< Is that a number type with specific traits.

      using ReturnType  = T;  ///< Alias to the type that should be used as return type.

      /** @brief Defines a type that represents the "best" way to pass
       *  a parameter of type T to a function.
       */
      using ParamType   = typename boost::call_traits<T>::param_type;

      /// Constant Zero.
      static constexpr T ZERO = T(0);

      /// Constant One.
      static constexpr T ONE  = T(1);

      /// Return the zero of this integer.
      static inline constexpr
      ReturnType zero() noexcept
      {
        return ZERO;
      }

      /// Return the one of this integer.
      static inline constexpr
      ReturnType one() noexcept
      {
        return ONE;
      }

      /// Return the minimum possible value for this type of number.
      static inline constexpr
      ReturnType min() noexcept
      {
        return NL::min();
      }

      /// Return the maximum possible value for this type of number.
      static inline constexpr
      ReturnType max() noexcept
      {
        return NL::max();
      }

      /// Return the number of significant binary digits for this type of number.
      static inline constexpr
      unsigned int digits() noexcept
      {
        return static_cast<unsigned int>(NL::digits);
      }

      /** @brief Return the bounding type of the number.
       *
       * @return BOUNDED, UNBOUNDED, or BOUND_UNKNOWN.
       */
      static inline constexpr
      BoundEnum isBounded() noexcept
      {
        return NL::is_bounded ? BOUNDED : UNBOUNDED;
      }

      /** @brief Return the sign type of the number.
       *
       * @return SIGNED, UNSIGNED or SIGN_UNKNOWN.
       */
      static inline constexpr
      SignEnum isSigned() noexcept
      {
        return NL::is_signed ? SIGNED : UNSIGNED;
      }

      /** @brief
       * Cast method to DGtal::int64_t (for I/O or board export uses
       * only).
       */
      static inline constexpr
      DGtal::int64_t castToInt64_t(const T & aT) noexcept
      {
        return static_cast<DGtal::int64_t>(aT);
      }

      /** @brief
       * Cast method to DGtal::uint64_t (for I/O or board export uses
       * only).
       */
      static inline constexpr
      DGtal::uint64_t castToUInt64_t(const T & aT) noexcept
      {
        return static_cast<DGtal::uint64_t>(aT);
      }

      /** @brief
       * Cast method to double (for I/O or board export uses
       * only).
       */
      static inline constexpr
      double castToDouble(const T & aT) noexcept
      {
        return static_cast<double>(aT);
      }

      /** @brief Check the parity of a number.
       *
       * @param aT any number.
       * @return 'true' iff the number is even.
       */
      static inline constexpr
      bool even( ParamType aT ) noexcept
      {
        return ( aT & ONE ) == ZERO;
      }

      /** @brief Check the parity of a number.
       *
       * @param aT any number.
       * @return 'true' iff the number is odd.
       */
      static inline constexpr
      bool odd( ParamType aT ) noexcept
      {
        return ( aT & ONE ) != ZERO;
      }

    };

    // Definition of the static attributes in order to allow ODR-usage.
    template <typename T> constexpr T NumberTraitsImplFundamental<T>::ZERO;
    template <typename T> constexpr T NumberTraitsImplFundamental<T>::ONE;

  } // namespace details

  /// Specialization of NumberTraitsImpl for fundamental integer types.
  template <typename T>
  struct NumberTraitsImpl<T, typename std::enable_if<std::is_integral<T>::value>::type>
    : details::NumberTraitsImplFundamental<T>
  {
  private:
    using NTIF = typename details::NumberTraitsImplFundamental<T>;  ///< Internal type alias to avoid repetitions.

  public:
    using SignedVersion   = typename std::make_signed<T>::type;   ///< Alias to the signed version of the number type.
    using UnsignedVersion = typename std::make_unsigned<T>::type; ///< Alias to the unsigned version of the number type.

    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is even.
     */
    static inline constexpr
    bool even( typename NTIF::ParamType aT ) noexcept
    {
      return ( aT & NTIF::ONE ) == NTIF::ZERO;
    }

    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is odd.
     */
    static inline constexpr
    bool odd( typename NTIF::ParamType aT ) noexcept
    {
      return ( aT & NTIF::ONE ) != NTIF::ZERO;
    }

  }; // end of class NumberTraitsImpl

  /// Specialization of NumberTraitsImpl for fundamental floating-point types.
  template <typename T>
  struct NumberTraitsImpl<T, typename std::enable_if<std::is_floating_point<T>::value>::type>
    : details::NumberTraitsImplFundamental<T>
  {
    using SignedVersion   = T; ///< Alias to the signed version of a floating-point type (aka itself).
    using UnsignedVersion = T; ///< Alias to the unsigned version of a floating-point type (aka itself).
  }; // end of class NumberTraitsImpl

  /** @brief Specialization of NumberTraitsImpl for DGtal::BigInteger
   *
   * Note that DGtal::BigInteger represents
   * signed and unsigned arbitrary-size integers. Therefore both
   * IsUnsigned and IsSigned are TagTrue.
   */
  template <typename Enable>
  struct NumberTraitsImpl<DGtal::BigInteger, Enable>
  {
    typedef TagTrue IsIntegral;     ///< A BigInteger is of integral type.
    typedef TagFalse IsBounded;     ///< A BigInteger is not bounded.
    typedef TagTrue IsUnsigned;     ///< A BigInteger can be signed and unsigned.
    typedef TagTrue IsSigned;       ///< a BigInteger can be signed and unsigned.
    typedef TagTrue IsSpecialized;  ///< Is that a number type with specific traits.

    typedef DGtal::BigInteger SignedVersion;    ///< Alias to the signed version of a BigInteger (aka a BigInteger).
    typedef DGtal::BigInteger UnsignedVersion;  ///< Alias to the unsigned version of a BigInteger (aka a BigInteger).
    typedef DGtal::BigInteger ReturnType;       ///< Alias to the type that should be used as return type.

    /** @brief Defines a type that represents the "best" way to pass
     *  a parameter of type T to a function.
     */
    typedef typename boost::call_traits<BigInteger>::param_type ParamType;

    /// Constant Zero.
    static const DGtal::BigInteger ZERO ;

    /// Constant One.
    static const DGtal::BigInteger ONE ;

    /// Return the zero of this integer.
    static inline
    ReturnType zero() noexcept
    {
      return ZERO;
    }

    /// Return the one of this integer.
    static inline
    ReturnType one() noexcept
    {
      return ONE;
    }

    /// Return the minimum possible value (trigger an error since BitInteger is unbounded).
    static inline
    ReturnType min() noexcept
    {
      FATAL_ERROR_MSG(false, "UnBounded integer type does not support min() function");
      return zero();
    }

    /// Return the maximum possible value (trigger an error since BitInteger is unbounded).
    static inline
    ReturnType max() noexcept
    {
      FATAL_ERROR_MSG(false, "UnBounded integer type does not support max() function");
      return zero();
    }

    /// Return the number of significant binary digits (trigger an error since BitInteger is unbounded).
    static inline
    unsigned int digits() noexcept
    {
      FATAL_ERROR_MSG(false, "UnBounded integer type does not support digits() function");
      return 0;
    }

    /** @brief Return the bounding type of the number.
     *
     * @return BOUNDED, UNBOUNDED, or BOUND_UNKNOWN.
     */
    static inline
    BoundEnum isBounded() noexcept
    {
      return UNBOUNDED;
    }

    /** @brief Return the sign type of the number.
     *
     * @return SIGNED, UNSIGNED or SIGN_UNKNOWN.
     */
    static inline
    SignEnum isSigned() noexcept
    {
      return SIGNED;
    }

    /** @brief
     * Cast method to DGtal::int64_t (for I/O or board export uses
     * only).
     */
    static inline
    DGtal::int64_t castToInt64_t(const DGtal::BigInteger & aT) noexcept
    {
      return static_cast<DGtal::int64_t>(aT);
    }

    /** @brief
     * Cast method to DGtal::uint64_t (for I/O or board export uses
     * only).
     */
    static inline
    DGtal::uint64_t castToUInt64_t(const DGtal::BigInteger & aT) noexcept
    {
      return static_cast<DGtal::uint64_t>(aT);
    }


    /** @brief
     * Cast method to double (for I/O or board export uses
     * only).
     */
    static inline
    double castToDouble(const DGtal::BigInteger & aT) noexcept
    {
      return static_cast<double>(aT);
    }

    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is even.
     */
    static inline
    bool even( ParamType aT ) noexcept
    {
      return (boost::multiprecision::integer_modulus(aT, 2) == 0);
    }

    /** @brief Check the parity of a number.
     *
     * @param aT any number.
     * @return 'true' iff the number is odd.
     */
    static inline
    bool odd( ParamType aT ) noexcept
    {
      return (boost::multiprecision::integer_modulus(aT, 2) == 1);
    }
  }; // end of class NumberTraits<DGtal::BigInteger>.

  // Definition of the static attributes in order to allow ODR-usage.
  template <typename Enable> const DGtal::BigInteger NumberTraitsImpl<DGtal::BigInteger, Enable>::ZERO = 0;
  template <typename Enable> const DGtal::BigInteger NumberTraitsImpl<DGtal::BigInteger, Enable>::ONE  = 1;

  /**
   * Description of template class 'NumberTraits' <p>
   * \brief Aim: The traits class for all models of Cinteger.
   *
   * Since CInteger describes the concept Integer, this class is used
   * by models of CIinteger to specialize some definitions related to
   * Integer. For instance it defines whether a given Integer is
   * signed or not and what is signed/unsigned version.
   *
   * It inherits from NumberTraitsImpl by passing it the given number type
   * without any CV qualifier.
   *
   * @see NumberTraitsImpl
   */
  template <typename T>
  struct NumberTraits
    : NumberTraitsImpl<typename std::decay<T>::type>
  {
  };

  class Warning_promote_trait_not_specialized_for_this_case { };

  template<class A, class B>
  struct promote_trait
  {
    typedef Warning_promote_trait_not_specialized_for_this_case promote_t;
  };

  template<>
  struct promote_trait<int32_t, int64_t>
  {
    typedef int64_t promote_t;
  };

} // namespace DGtal

#endif // !defined NumberTraits_h

#undef NumberTraits_RECURSES
#endif // else defined(NumberTraits_RECURSES)
