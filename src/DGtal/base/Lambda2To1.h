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
 * @file Lambda2To1.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/02
 *
 * Header file for module Lambda2To1.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Lambda2To1_RECURSES)
#error Recursive header files inclusion detected in Lambda2To1.h
#else // defined(Lambda2To1_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Lambda2To1_RECURSES

#if !defined Lambda2To1_h
/** Prevents repeated inclusion of headers. */
#define Lambda2To1_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Lambda2To1
  /**
   * Description of template class 'Lambda2To1' <p> \brief Aim: A functor
   * object that that realizes ( V x W -> S ) x V -> ( W -> S ). It
   * fixes the first parameter of a two-variate mapping so as to build
   * a one-variate mapping.
   *
   * @tparam TFunctor2 any mapping V x W -> S.
   * @tparam TArgument1 the type V.
   * @tparam TArgument2 the type W.
   * @tparam TReturnType the type S.
   */
  template <typename TFunctor2, 
            typename TArgument1, 
            typename TArgument2, 
            typename TReturnType>
  class Lambda2To1
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef Lambda2To1< TFunctor2, TArgument1, TArgument2, TReturnType > Self;
    typedef TFunctor2 Functor2;
    typedef TArgument1 FormerArgument1;
    typedef TArgument2 FormerArgument2;
    typedef TReturnType ReturnType;
    typedef FormerArgument2 Argument1;
    typedef Argument1 Argument;
    typedef ReturnType Value;

    /**
     * Destructor.
     */
    ~Lambda2To1();

    /**
     * Constructor from functor and fixed first argument.
     */
    Lambda2To1( const Functor2 & f2, const FormerArgument1 & arg1 );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Lambda2To1 ( const Self & other );

    /**
       @param any object of type W
       @return the value myF2( myArg1, arg )
    */
    Value operator()( const Argument & arg ) const;
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    const Functor2 & myF2; //< The functor V x W -> S
    const FormerArgument1 & myArg1; //< The value V

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Lambda2To1();

  private:
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Lambda2To1 & operator= ( const Lambda2To1 & other );

  }; // end of class Lambda2To1

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/Lambda2To1.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Lambda2To1_h

#undef Lambda2To1_RECURSES
#endif // else defined(Lambda2To1_RECURSES)
