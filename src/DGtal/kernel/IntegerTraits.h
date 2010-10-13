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
//LICENSE-END
#pragma once

/**
 * @file IntegerTraits.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/03
 *
 * Header file for module IntegerTraits.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegerTraits_RECURSES)
#error Recursive header files inclusion detected in IntegerTraits.h
#else // defined(IntegerTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegerTraits_RECURSES

#if !defined IntegerTraits_h
/** Prevents repeated inclusion of headers. */
#define IntegerTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <boost/integer_traits.hpp>
#include <boost/call_traits.hpp>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  enum BoundEnum {BOUNDED = 0, UNBOUNDED = 1, BOUND_UNKNOWN = 2};
  enum SignEnum {SIGNED = 0, UNSIGNED = 1, SIGN_UNKNOWN = 2};
  

  /////////////////////////////////////////////////////////////////////////////
  // template class IntegerTraits
  /**
   * Description of template class 'IntegerTraits' <p>
   * \brief Aim: The traits class for all models of Cinteger.
   *
   * Since CInteger describes the concept Integer, this class is used
   * by models of CIinteger to specialize some definitions related to
   * Integer. For instance it defines whether a given Integer is
   * signed or not and what is signed/unsigned version.
   */
  template <typename T>
  struct IntegerTraits
  {
    // ----------------------- Associated types ------------------------------
    typedef TagUnknown IsBounded;
    typedef TagUnknown IsUnsigned;
    typedef TagFalse IsSpecialized;
    typedef T SignedVersion;
    typedef T UnsignedVersion;
    typedef T ReturnType;

    //Defines a type that represents the "best" way to pass
    // a parameter of type T to a function.
    typedef typename boost::call_traits<T>::param_type ParamType;


    /**
     * Constant Zero.
     */
    static const T ZERO = T(0);

    /**
     * Constant One.
     */
    static const T ONE = T(1);

    /**
     * @return the zero of this integer.
     */
    static ReturnType zero();

    /**
     * @return the one of this integer.
     */
    static ReturnType one();

    /**
     * @return the minimum possible value for this type of integer or
     * ONE if not bounded or unknown.
     */
    static ReturnType min();

    /**
     * @return the maximum possible value for this type of integer or
     * ZERO if not bounded or unknown.
     */
    static ReturnType max();

    /**
     * @return the number of significant digits for this integer type,
     * or 0 if unbounded or unknown.
     */
    static unsigned int digits();

    /**
     * @return BOUNDED, UNBOUNDED, or BOUND_UNKNOWN.
     */
    static BoundEnum isBounded();
    /**
     * @return SIGNED, UNSIGNED or SIGN_UNKNOWN.
     */
    static SignEnum isUnsigned();

  }; // end of class IntegerTraits



#if ( defined(WIN32))
  /// On VS2008, unsigned int and uint??_t are different
  /**
   * Specialization for <int>.
   */
  template <>
  struct IntegerTraits<int>
  {
    typedef TagTrue IsBounded;
    typedef TagFalse IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int SignedVersion;
    typedef unsigned int UnsignedVersion;
    typedef int ReturnType;
    typedef boost::call_traits<int>::param_type ParamType;
    static const int ZERO = 0;
    static const int ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<int>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<int>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<int>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<int>.

  /**
   * Specialization for <int>.
   */
  template <>
  struct IntegerTraits<unsigned int>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int SignedVersion;
    typedef unsigned int UnsignedVersion;
    typedef int ReturnType;
    typedef boost::call_traits<unsigned int>::param_type ParamType;
    static const unsigned int ZERO = 0;
    static const unsigned int ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<unsigned int>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<unsigned int>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<unsigned int>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<unsigned int>.
#endif //WIN32

   /**
   * Specialization for <usigned char>.
   */
  template <>
  struct IntegerTraits<unsigned char>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef char SignedVersion;
    typedef unsigned char UnsignedVersion;
    typedef unsigned char ReturnType;
    typedef boost::call_traits<unsigned char>::param_type ParamType;
    static const unsigned char ZERO = 0;
    static const unsigned char ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<unsigned char>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<unsigned char>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<unsigned char>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<unsigned char>.



   /**
   * Specialization for <usigned char>.
   */
  template <>
  struct IntegerTraits<std::size_t>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef long int SignedVersion;
    typedef std::size_t UnsignedVersion;
    typedef std::size_t ReturnType;
    typedef boost::call_traits<std::size_t>::param_type ParamType;
    static const std::size_t ZERO = 0;
    static const std::size_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<std::size_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<std::size_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<std::size_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<std::size_t>.

  /**
   * Specialization for <uint16_t>.
   */
  template <>
  struct IntegerTraits<uint16_t>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int16_t SignedVersion;
    typedef uint16_t UnsignedVersion;
    typedef uint16_t ReturnType;
    typedef boost::call_traits<uint16_t>::param_type ParamType;
    static const uint16_t ZERO = 0;
    static const uint16_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<uint16_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<uint16_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<uint16_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<uint16_t>.

  /**
   * Specialization for <int16_t>.
   */
  template <>
  struct IntegerTraits<int16_t>
  {
    typedef TagTrue IsBounded;
    typedef TagFalse IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int16_t SignedVersion;
    typedef uint16_t UnsignedVersion;
    typedef int16_t ReturnType;
    typedef boost::call_traits<int16_t>::param_type ParamType;
    static const int16_t ZERO = 0;
    static const int16_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<int16_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<int16_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<int16_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return SIGNED; }
  }; // end of class IntegerTraits<int16_t>.

  /**
   * Specialization for <uint32_t>.
   */
  template <>
  struct IntegerTraits<uint32_t>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int32_t SignedVersion;
    typedef uint32_t UnsignedVersion;
    typedef uint32_t ReturnType;
    typedef boost::call_traits<uint32_t>::param_type ParamType;
    static const uint32_t ZERO = 0;
    static const uint32_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<uint32_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<uint32_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<uint32_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<uint32_t>.

  /**
   * Specialization for <int32_t>.
   */
  template <>
  struct IntegerTraits<int32_t>
  {
    typedef TagTrue IsBounded;
    typedef TagFalse IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int32_t SignedVersion;
    typedef uint32_t UnsignedVersion;
    typedef int32_t ReturnType;
    typedef boost::call_traits<int32_t>::param_type ParamType;
    static const int32_t ZERO = 0;
    static const int32_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<int32_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<int32_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<int32_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return SIGNED; }
  }; // end of class IntegerTraits<int32_t>.

  /**
   * Specialization for <uint64_t>.
   */
  template <>
  struct IntegerTraits<uint64_t>
  {
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int64_t SignedVersion;
    typedef uint64_t UnsignedVersion;
    typedef uint64_t ReturnType;
    typedef boost::call_traits<uint64_t>::param_type ParamType;
    static const uint64_t ZERO = 0;
    static const uint64_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<uint64_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<uint64_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<uint64_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return UNSIGNED; }
  }; // end of class IntegerTraits<uint64_t>.

  /**
   * Specialization for <int64_t>.
   */
  template <>
  struct IntegerTraits<int64_t>
  {
    typedef TagTrue IsBounded;
    typedef TagFalse IsUnsigned;
    typedef TagTrue IsSpecialized;
    typedef int64_t SignedVersion;
    typedef uint64_t UnsignedVersion;
    typedef int64_t ReturnType;
    typedef boost::call_traits<int64_t>::param_type ParamType;
    static const int64_t ZERO = 0;
    static const int64_t ONE = 1;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static ReturnType min()
    { return boost::integer_traits<int64_t>::const_min; }
    static ReturnType max()
    { return boost::integer_traits<int64_t>::const_max; }
    static unsigned int digits()
    { return boost::integer_traits<int64_t>::digits; }
    static BoundEnum isBounded()
    { return BOUNDED; }
    static SignEnum isUnsigned()
    { return SIGNED; }
  }; // end of class IntegerTraits<int64_t>.


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/IntegerTraits.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegerTraits_h

#undef IntegerTraits_RECURSES
#endif // else defined(IntegerTraits_RECURSES)
