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
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

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
    typedef T SignedVersion;
    typedef T UnsignedVersion;
    typedef T ReturnType;

    /**
     * @return the zero of this integer.
     */
    static ReturnType zero();

    /**
     * @return the one of this integer.
     */
    static ReturnType one();

    /**
     * @return the number of significant digits for this integer type,
     * or 0 if unbounded or unknown.
     */
    static unsigned int digits();

    /**
     * @return 1 if bounded, 0, if not bounded, 2 if unknown.
     */
    static unsigned int isBounded();
    /**
     * @return 1 if unsigned, 0, if signed, 2 if unknown.
     */
    static unsigned int isUnsigned();

  }; // end of class IntegerTraits


  /**
   * Specialization for <int>.
   */
  template <>
  struct IntegerTraits<int>
  {
    // ----------------------- Associated types ------------------------------
    typedef TagTrue IsBounded;
    typedef TagFalse IsUnsigned;
    typedef int SignedVersion;
    typedef unsigned int UnsignedVersion;
    typedef int ReturnType;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static unsigned int digits()
    { return std::numeric_limits<int>::digits; }
    static unsigned int isBounded()
    { return 1; }
    static unsigned int isUnsigned()
    { return 0; }

  }; // end of class IntegerTraits

  /**
   * Specialization for <unsigned int>.
   */
  template <>
  struct IntegerTraits<unsigned int>
  {
    // ----------------------- Associated types ------------------------------
    typedef TagTrue IsBounded;
    typedef TagTrue IsUnsigned;
    typedef int SignedVersion;
    typedef unsigned int UnsignedVersion;
    typedef unsigned int ReturnType;
    static ReturnType zero()
    { return 0; }
    static ReturnType one()
    { return 1; }
    static unsigned int digits()
    { return std::numeric_limits<unsigned int>::digits; }
    static unsigned int isBounded()
    { return 1; }
    static unsigned int isUnsigned()
    { return 1; }

  }; // end of class IntegerTraits

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/IntegerTraits.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegerTraits_h

#undef IntegerTraits_RECURSES
#endif // else defined(IntegerTraits_RECURSES)
