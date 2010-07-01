#pragma once

/**
 * @file CDigitalSet.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et
 * Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CDigitalSet.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalSet_RECURSES)
#error Recursive header files inclusion detected in CDigitalSet.h
#else // defined(CDigitalSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalSet_RECURSES

#if !defined CDigitalSet_h
/** Prevents repeated inclusion of headers. */
#define CDigitalSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSet
  /**
   * Description of \b concept '\b CDigitalSet' <p> Aim: Represents a
   * set of points within the given domain. This set of points is
   * modifiable by the user.
   * 
   * <p> Refinement of  boost::DefaultConstructible, 
   * boost::CopyConstructible, boost::Assignable
   *
   * @todo add boost::Container ?
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CDigitalSet
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
   * @ingroup Concepts
   */
  template <typename Domain>
  struct CDigitalSet : 
    boost::DefaultConstructible,
    boost::CopyConstructible, 
    boost::Assignable
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDigitalSet
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/CDigitalSet.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSet_h

#undef CDigitalSet_RECURSES
#endif // else defined(CDigitalSet_RECURSES)
