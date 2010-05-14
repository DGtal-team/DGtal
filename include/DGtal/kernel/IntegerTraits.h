#pragma once

/** 
 * @file IntegerTraits.h.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/13
 * 
 * Header file for module IntegerTraits
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegerTraits_RECURSES)
#error Recursive header files inclusion detected in IntegerTraits.h.h
#else // defined(IntegerTraits.h_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegerTraits_RECURSES

#if !defined IntegerTraits_h
/** Prevents repeated inclusion of headers. */
#define IntegerTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include <boost/cstdint.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  ///Copy of Boost integer types
  typedef boost::uint16_t uint16_t;
  typedef boost::uint32_t uint32_t;
  typedef boost::uint64_t uint64_t;
  
  typedef boost::int16_t int16_t;
  typedef boost::int32_t int32_t;
  typedef boost::int64_t int64_t;

  template <class T> 
  struct IntegerTraits : TypeTraits<T> {
    typedef Tag_false IsUnbounded;
    typedef Tag_false IsSigned;
    typedef typename TypeTraits<T>::ReturnType ReturnType;
    
    static ReturnType zero() { return 0; }
    static ReturnType one() { return 1; }
    static bool hasUnlimitedDigits() { return false; }
  };
  
  template <>
  struct IntegerTraits<int32_t> : TypeTraits<int32_t> {
    typedef Tag_false Is_unbounded;
    typedef Tag_true Is_signed;

    static ReturnType zero() { return 0; }
    static ReturnType one() { return 1; }
    static bool hasUnlimitedDigits() { return false; }
  };
  
  template <>
  struct IntegerTraits<uint32_t> : TypeTraits<uint32_t> {
    typedef Tag_false Is_unbounded;
    typedef Tag_false Is_signed;
    
    static ReturnType zero() { return 0; }
    static ReturnType one() { return 1; }
    static bool hasUnlimitedDigits() { return false; }
  };
  
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegerTraits.h_h

#undef IntegerTraits_RECURSES
#endif // else defined(IntegerTraits.h_RECURSES)
