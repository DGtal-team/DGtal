#pragma once

/**
 * @file CAdjacency.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/04
 *
 * Header file for concept CAdjacency.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CAdjacency_RECURSES)
#error Recursive header files inclusion detected in CAdjacency.h
#else // defined(CAdjacency_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CAdjacency_RECURSES

#if !defined CAdjacency_h
/** Prevents repeated inclusion of headers. */
#define CAdjacency_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CAdjacency
  /**
   * Description of \b concept '\b CAdjacency' <p>
   * @ingroup Concepts
   *
   * \brief Aim: The concept CAdjacency defines an elementary
   * adjacency relation between points of a digital domain.
   * 
   * It thus distinguishes which points are close and which points are
   * further away in this domain. Adjacency relations are used to
   * define a digital topology, in the sense of Rosenfeld or in the
   * sense of Herman. In other words, and adjacency relation define a
   * neighborhood graph on the points of a digital domain.
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CAdjacency
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
  struct CAdjacency
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CAdjacency
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/CAdjacency.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CAdjacency_h

#undef CAdjacency_RECURSES
#endif // else defined(CAdjacency_RECURSES)
