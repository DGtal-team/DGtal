#pragma once

/**
 * @file CInteger.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CInteger.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CInteger_RECURSES)
#error Recursive header files inclusion detected in CInteger.h
#else // defined(CInteger_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CInteger_RECURSES

#if !defined CInteger_h
/** Prevents repeated inclusion of headers. */
#define CInteger_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/IntegerTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CInteger
  /**
   * Description of \b concept '\b CInteger' <p>     
   * @ingroup Concepts
   *
   * \brief Aim: The concept CInteger specifies what are the usual
   * integer numbers, more precisely the ones that are representable
   * on a computer.
   *
   * Generally, all the basic computer integer types are models of
   * this concept. More elaborate integer types with variable sizes,
   * for instance the big integers of GMP, are also models of this
   * concept.
   * 
   * <p> Refinement of boost::Assignable<T>, boost::EqualityComparable<T>
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CInteger
   * - \t x, \t y	: Object of type X
   * - \t i, \t j	: basic integer type.
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table>
   * <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> Construction from basic integer type</td>
   * <td> X( i ) </td> <td> </td> <td> </td>
   * <td> </td> <td> \c X represents the integer \c i</td> <td> </td> <td> </td>
   * </tr>
   * <tr> <td> Addition </td> <td> x+y </td>
   * <td>  </td> <td> X </td>
   * <td>  </td> <td> addition of two integers </td> 
   * <td>  </td> <td>  </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br> 
   * 
   * short, int, unsigned int, long long, unsigned long long,
   * uint16_t, uint32_t, uint64_t, int16_t, int32_t, int64_t.
   *
   * <p> Notes <br>
   *
   * @todo Complete integer checking.
   */
  template <typename T>
  struct CInteger : boost::Assignable<T>, boost::EqualityComparable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE(CInteger)
    {
      T x( 0 ); // require constructor from built-in integer.
      T y( 1 ); // require constructor from built-in integer.
      x = x + y;
      x = x - y;
      ConceptUtils::sameType( myX, ++x );
      ConceptUtils::sameType( myX, --x );
      // @todo x+y with short is promoted to int. We should use some
      // verification with possible promoting.
      // ConceptUtils::sameType( myX, x + y );
      ConceptUtils::sameType( myX, IntegerTraits<T>::ZERO );
      ConceptUtils::sameType( myX, IntegerTraits<T>::ONE );
    }
      
    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    T myY;
    typename IntegerTraits<T>::IsUnsigned myIsUnsigned;
    typename IntegerTraits<T>::IsBounded myIsBounded;
    typename IntegerTraits<T>::SignedVersion mySignedVersion;
    typename IntegerTraits<T>::UnsignedVersion myUnsignedVersion;
    typename IntegerTraits<T>::ReturnType myReturnType;

    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CInteger
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/CInteger.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CInteger_h

#undef CInteger_RECURSES
#endif // else defined(CInteger_RECURSES)
