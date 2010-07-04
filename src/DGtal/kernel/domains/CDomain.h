#pragma once

/**
 * @file CDomain.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDomain_RECURSES)
#error Recursive header files inclusion detected in CDomain.h
#else // defined(CDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDomain_RECURSES

#if !defined CDomain_h
/** Prevents repeated inclusion of headers. */
#define CDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
#include "DGtal/utils/ConceptUtils.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDomain
  /**
   * Description of \b concept '\b CDomain' <p> Aim: This concept
   * represents a digital domain, i.e. a subset of points of the given
   * digital space.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * - DigitalSpace : the embedding digital space.
   * - Point : the point type of the space
   * - SizeType : the type used for counting elements of the space.
   * - Vector : the vector type of the space
   * - LinearMap : the linear map type of the space
   * 
   * 
   * <p> Notation
   * - \t X : A type that is a model of CDomain
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
   * HyperRectDomain 
   *
   * <p> Notes <br>
   *
   * @todo Complete domain checking.
   */
  template <typename T>
  struct CDomain
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::DigitalSpace DigitalSpace;
    typedef typename T::Point Point;
    typedef typename T::Vector Vector;
    //typedef typename T::LinearMap LinearMap;

    BOOST_CONCEPT_USAGE( CDomain )
    {
      // Domain should have a lowerBound() returning a Point.
      ConceptUtils::sameType( myP, myT.lowerBound() );
      // Domain should have an upperBound() returning a Point.
      ConceptUtils::sameType( myP, myT.upperBound() );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myT;
    Point myP;

    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDomain
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/domains/CDomain.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDomain_h

#undef CDomain_RECURSES
#endif // else defined(CDomain_RECURSES)
