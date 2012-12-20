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
 * @file BasicBoolFunctions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Defines several boolean function object wrapper types.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicBoolFunctions_RECURSES)
#error Recursive header files inclusion detected in BasicBoolFunctions.h
#else // defined(BasicBoolFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicBoolFunctions_RECURSES

#if !defined BasicBoolFunctions_h
/** Prevents repeated inclusion of headers. */
#define BasicBoolFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/function.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /**
   * May hold bool functions taking no arguments.
   */
  typedef boost::function0< bool > BoolFunction0;

  /**
   * May hold bool functions taking one bool argument.
   */
  typedef boost::function1< bool, bool > BoolFunction1;

  /**
   * May hold bool functions taking two bool arguments.
   */
  typedef boost::function2< bool, bool, bool > BoolFunction2;

  /**
   * May hold bool functions taking three bool arguments (Useful ?).
   */
  typedef boost::function3< bool, bool, bool, bool > BoolFunction3;

  /**
   * Functor with no argument returning true.
   */
  struct TrueBoolFct0 {
    bool operator()() const;
  };

  /**
   * Functor with no argument returning false.
   */
  struct FalseBoolFct0 {
    bool operator()() const;
  };

  /**
   * The object function (0 bool args) returning true.
   */
  static const BoolFunction0 trueBF0 = TrueBoolFct0();

  /**
   * The object function (0 bool args) returning false.
   */
  static const BoolFunction0 falseBF0 = FalseBoolFct0();

  /**
   * Functor boolean identity.
   */
  struct IdentityBoolFct1 {
    bool operator()( bool b ) const;
  };

  /**
   * Functor boolean NOT.
   */
  struct NotBoolFct1 {
    bool operator()( bool b ) const;
  };

  /**
   * The object function identity (1 bool args).
   */
  static const BoolFunction1 identityBF1 = IdentityBoolFct1();

  /**
   * The object function NOT (1 bool args).
   */
  static const BoolFunction1 notBF1 = NotBoolFct1();

  /**
   * Functor boolean and.
   */
  struct AndBoolFct2 {
    bool operator()( bool b1, bool b2 ) const;
  };

  /**
   * Functor boolean or.
   */
  struct OrBoolFct2 {
    bool operator()( bool b1, bool b2 ) const;
  };

  /**
   * Functor boolean xor.
   */
  struct XorBoolFct2 {
    bool operator()( bool b1, bool b2 ) const;
  };

  /**
   * Functor boolean implies.
   */
  struct ImpliesBoolFct2 {
    bool operator()( bool b1, bool b2 ) const;
  };

  /**
   * The object function and (2 bool args).
   */
  static const BoolFunction2 andBF2 = AndBoolFct2();

  /**
   * The object function or (2 bool args).
   */
  static const BoolFunction2 orBF2 = OrBoolFct2();

  /**
   * The object function xor (2 bool args).
   */
  static const BoolFunction2 xorBF2 = XorBoolFct2();

  /**
   * The object function implies (2 bool args).
   */
  static const BoolFunction2 impliesBF2 = ImpliesBoolFct2();

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/BasicBoolFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicBoolFunctions_h

#undef BasicBoolFunctions_RECURSES
#endif // else defined(BasicBoolFunctions_RECURSES)
