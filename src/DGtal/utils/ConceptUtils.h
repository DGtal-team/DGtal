#pragma once

/**
 * @file ConceptUtils.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/02
 *
 * Header file for module ConceptUtils.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConceptUtils_RECURSES)
#error Recursive header files inclusion detected in ConceptUtils.h
#else // defined(ConceptUtils_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConceptUtils_RECURSES

#if !defined ConceptUtils_h
/** Prevents repeated inclusion of headers. */
#define ConceptUtils_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ConceptUtils
/**
 * Description of class 'ConceptUtils' <p>
 *
 * \brief Aim: This utility class gathers several static methods
 * useful for concept checks.
 *
 * This class, as well as its methods are not meant to be
 * realized. They are just used to check concepts.
 */
class ConceptUtils
{
public:

  /**
   * Type deduction will fail unless the arguments have the same type.
   *
   * @param t1 some object of type T.
   * @param t2 some object of type T.
   */
  template <typename T>
  static
  void sameType( const T & t1, const T & t2 );
  

}; // end of class ConceptUtils

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConceptUtils_h

#undef ConceptUtils_RECURSES
#endif // else defined(ConceptUtils_RECURSES)
