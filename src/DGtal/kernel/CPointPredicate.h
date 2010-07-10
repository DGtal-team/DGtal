#pragma once

/**
 * @file CPointPredicate.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Header file for concept CPointPredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CPointPredicate_RECURSES)
#error Recursive header files inclusion detected in CPointPredicate.h
#else // defined(CPointPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CPointPredicate_RECURSES

#if !defined CPointPredicate_h
/** Prevents repeated inclusion of headers. */
#define CPointPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CPointPredicate
  /**
   * Description of \b concept '\b CPointPredicate' <p>
   * @ingroup Concepts
   * \brief Aim: Defines a predicate on a point.
   * 
   * Associates a boolean to points.
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * - Point
   *
   * <p> Notation
   * - \t X : A type that is a model of CPointPredicate
   * - \t x : Object of type X
   * - \t p : Object of type Point
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
   * <td> Apply predicate </td> <td> x( p )</td> <td> </td> <td> \c bool </td>
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
  struct CPointPredicate
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Point Point;

    BOOST_CONCEPT_USAGE( CPointPredicate )
    {
      // x( p ) returns bool.
      ConceptUtils::sameType( myBool, myPred.operator() ( myPoint ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myPred;
    Point myPoint;
    bool myBool;
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CPointPredicate
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/CPointPredicate.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPointPredicate_h

#undef CPointPredicate_RECURSES
#endif // else defined(CPointPredicate_RECURSES)
