#pragma once

/**
 * @file CUnsignedInteger.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for concept CUnsignedInteger.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CUnsignedInteger_RECURSES)
#error Recursive header files inclusion detected in CUnsignedInteger.h
#else // defined(CUnsignedInteger_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CUnsignedInteger_RECURSES

#if !defined CUnsignedInteger_h
/** Prevents repeated inclusion of headers. */
#define CUnsignedInteger_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CUnsignedInteger
  /**
   * Description of \b concept '\b CUnsignedInteger' <p>
   * @ingroup Concepts
   * Aim: Concept checking for Unsigned Integer.
   * 
   * <p> Refinement of CInteger
   *
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CUnsignedInteger
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
   */
  template <typename T>
  struct CUnsignedInteger: CInteger<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CUnsignedInteger
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/CUnsignedInteger.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CUnsignedInteger_h

#undef CUnsignedInteger_RECURSES
#endif // else defined(CUnsignedInteger_RECURSES)
