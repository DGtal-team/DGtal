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
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CInteger
  /**
Description of \b concept '\b CInteger' <p>
     @ingroup Concepts

     \brief Aim: The concept CInteger specifies what are the usual
     integer numbers, more precisely the ones that are representable
     on a computer.

     Generally, all the basic computer integer types are models of
     this concept. More elaborate integer types with variable sizes,
     for instance the big integers of GMP, are also models of this
     concept.

 ### Refinement of boost::Assignable<T>,
     boost::EqualityComparable<T>, boost::LessThanComparable<T>

 ### Associated types :

 ### Notation
     - \t X : A type that is a model of CInteger
     - \t x, \t y  : object of type X
     - \t i, \t j  : basic integer type.

 ### Definitions

 ### Valid expressions and semantics

     <table>
     <tr>
     <td class=CName> \b Name </td>
     <td class=CExpression> \b Expression </td>
     <td class=CRequirements> \b Type requirements </td>
     <td class=CReturnType> \b Return type </td>
     <td class=CPrecondition> \b Precondition </td>
     <td class=CSemantics> \b Semantics </td>
     <td class=CPostCondition> \b Postcondition </td>
     <td class=CComplexity> \b Complexity </td>
     </tr>
     <tr>
     <td class=CName>            Construction from basic integer type </td>
     <td class=CExpression>      X( i ) </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       \t X represents the integer \t i</td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Addition </td>
     <td class=CExpression>      \t x + \t y </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       addition of two integers </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Subtraction </td>
     <td class=CExpression>      \t x - \t y </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       subtraction of two integers </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Multiplication </td>
     <td class=CExpression>      \t x * \t y </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       multiplication of two integers </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Division </td>
     <td class=CExpression>      \t x / \t y </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       euclidean division of two integers </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Modulo </td>
     <td class=CExpression>      \t x % \t y </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       modulo of two integers </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Pre-increment </td>
     <td class=CExpression>      ++ \t x </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       \t x is incremented then its value is returned  </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            Pre-decrement </td>
     <td class=CExpression>      -- \t x </td>
     <td class=CRequirements>    </td>
     <td class=CReturnType>      \t X</td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       \t x is decremented then its value is returned  </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should have constant \t ZERO  in \t NumberTraits. </td>
     <td class=CExpression>      NumberTraits<X>::ZERO </td>
     <td class=CRequirements>    constant should be defined </td>
     <td class=CReturnType>      const \t X </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       the value 0</td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should have constant \t ONE in \t NumberTraits. </td>
     <td class=CExpression>      NumberTraits<X>::ONE </td>
     <td class=CRequirements>    constant should be defined </td>
     <td class=CReturnType>      const \t X </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       the value 1</td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>            \t X should be tagged in \t NumberTraits for \t IsUnsigned. </td>
     <td class=CExpression>      typename NumberTraits<X>::IsUnsigned </td>
     <td class=CRequirements>    TagTrue or TagFalse </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should be tagged in \t NumberTraits for \t IsSigned. </td>
     <td class=CExpression>      typename NumberTraits<X>::IsSigned </td>
     <td class=CRequirements>    TagTrue or TagFalse </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should be tagged in \t NumberTraits for \t IsBounded. </td>
     <td class=CExpression>      typename NumberTraits<X>::IsBounded </td>
     <td class=CRequirements>    TagTrue or TagFalse </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should have type \t SignedVersion  in \t NumberTraits. </td>
     <td class=CExpression>      typename NumberTraits<X>::SignedVersion </td>
     <td class=CRequirements>    type must be defined </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should have type \t UnsignedVersion  in \t NumberTraits. </td>
     <td class=CExpression>      typename NumberTraits<X>::UnsignedVersion </td>
     <td class=CRequirements>    type must be defined </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>
     <tr>
     <td class=CName>             \t X should have type \t ReturnType  in \t NumberTraits. </td>
     <td class=CExpression>      typename NumberTraits<X>::ReturnType </td>
     <td class=CRequirements>    type must be defined </td>
     <td class=CReturnType>      </td>
     <td class=CPrecondition>    </td>
     <td class=CSemantics>       </td>
     <td class=CPostCondition>   </td>
     <td class=CComplexity>      </td>
     </tr>

     </table>

 ### Invariants###

 ### Models###

     short, int, unsigned int, long long, unsigned long long,
     uint16_t, uint32_t, uint64_t, int16_t, int32_t, int64_t,
     DGtal::BigInteger

 ### Notes###

@tparam T the type that should be a model of integer.
   */
  template <typename T>
  struct CInteger : boost::Assignable<T>, boost::EqualityComparable<T>, boost::LessThanComparable<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE(CInteger)
    {
      T x( 0 ); // require constructor from built-in integer.
      T y( 1 ); // require constructor from built-in integer.
      ConceptUtils::sameType( myX, ++x );
      ConceptUtils::sameType( myX, --x );
      ConceptUtils::sameType( myX, NumberTraits<T>::ZERO );
      ConceptUtils::sameType( myX, NumberTraits<T>::ONE );

      // @note x-y with short is promoted to int. We should use some
      // verification with possible promoting.
      //
      // @note T(x-y) is required (instead of 'x-y') because of
      // gmpxx. Indeed, x+y returns an expression template in GMP and
      // thus cannot be of type T.
      ConceptUtils::sameType( myX, T(x-y) );
      // @note x+y with short is promoted to int. We should use some
      // verification with possible promoting.
      //
      // @note T(x+y) is required (instead of 'x+y') because of
      // gmpxx. Indeed, x+y returns an expression template in GMP and
      // thus cannot be of type T.
      ConceptUtils::sameType( myX, T(x+y) );
      ConceptUtils::sameType( myX, T(x*y) );
      ConceptUtils::sameType( myX, T(x/y) );
      ConceptUtils::sameType( myX, T(x%y) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    typename NumberTraits<T>::IsUnsigned myIsUnsigned;
    typename NumberTraits<T>::IsSigned myIsSigned;
    typename NumberTraits<T>::IsBounded myIsBounded;
    typename NumberTraits<T>::SignedVersion mySignedVersion;
    typename NumberTraits<T>::UnsignedVersion myUnsignedVersion;
    typename NumberTraits<T>::ReturnType myReturnType;

    BOOST_STATIC_ASSERT(( ConceptUtils::CheckTrueOrFalse<typename NumberTraits<T>::IsUnsigned>::value ));
    BOOST_STATIC_ASSERT(( ConceptUtils::CheckTrueOrFalse<typename NumberTraits<T>::IsSigned>::value ));
    BOOST_STATIC_ASSERT(( ConceptUtils::CheckTrueOrFalse<typename NumberTraits<T>::IsBounded>::value ));

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CInteger

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CInteger_h

#undef CInteger_RECURSES
#endif // else defined(CInteger_RECURSES)
