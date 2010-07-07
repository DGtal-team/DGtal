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
#include "DGtal/utils/ConceptUtils.h"
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
   * - \c Point : must be defined in the model.
   *
   * <p> Notations
   * - \c Adj : A type that is a model of CAdjacency
   * - \c adj	: Object of type Adj.
   * - \c p1, \c p2 : an object of type \ref Point.
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td> adjacency test </td> 
   * <td> adj.isAdjacentTo( p1, p2 ) </td> <td> \c p1 and \c p2 of same type Point. </td> <td> \c bool </td>
   * <td> </td> <td> Return 'true' when the two points are adjacent according to the adjacency relation \c adj </td> <td> </td> <td> </td>
   * </tr>
   *
   * <tr> 
   * <td> proper adjacency test </td> 
   * <td> adj.isProperlyAdjacentTo( p1, p2 ) </td> <td> \c p1 and \c p2 of same type Point. </td> <td> \c bool </td>
   * <td> </td> <td> Return 'true' when the two points are adjacent according to the adjacency relation \c adj and if \c p1 different from \c p2 </td> <td> </td> <td> </td>
   * </tr>
   *
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * - MetricAdjacency
   *
   * <p> Notes <br>
   */
  template <typename Adj>
  struct CAdjacency
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename Adj::Point Point;
    BOOST_CONCEPT_USAGE( CAdjacency )
    {
      // check isAdjacentTo
      ConceptUtils::sameType( myBool, myAdj.isAdjacentTo( myP1, myP2 ) );
      // check isProperlyAdjacentTo
      ConceptUtils::sameType( myBool, 
			      myAdj.isProperlyAdjacentTo( myP1, myP2 ) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    Adj myAdj;
    Point myP1;
    Point myP2;
    bool myBool;

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
