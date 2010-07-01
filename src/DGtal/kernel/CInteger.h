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
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CInteger
  /**
   * Description of \b concept '\b CInteger' <p>
   * Aim:
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CInteger
   * - \t x, \t y	: Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
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
    
    // ------------------------- Private Datas --------------------------------
  private:
    
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
