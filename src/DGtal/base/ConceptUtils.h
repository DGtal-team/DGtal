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
  /**
   * Defines the tag for 'false'.
   */
  struct TagFalse{};

  /**
   * Defines the tag for 'True'.
   */
  struct TagTrue{};

  /**
   * Defines the tag for 'Unknown'.
   */
  struct TagUnknown{};

  template<typename T1, typename T2>
  struct staticSameType
  { static const int value = 0; };

  template<typename T>
  struct staticSameType<T,T>
  { static const int value = 1; };
  
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

  /**
     Type deduction will fail unless the argument type is exactly TagTrue.
     @param tag the type to check.
  */
  static
  void checkTrue( const TagTrue & tag );

  /**
     Type deduction will fail unless the argument type is exactly TagFalse.
     @param tag the type to check.
  */
  static
  void checkFalse( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is exactly TagUnknown.
     @param tag the type to check.
  */
  static
  void checkUnknown( const TagUnknown & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  static
  void checkTag( const TagUnknown & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  static
  void checkTag( const TagTrue & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  static
  void checkTag( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is the tag TagTrue or TagFalse.
     @param tag the type to check.
  */
  static
  void checkTrueOrFalse( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is the tag TagTrue or TagFalse.
     @param tag the type to check.
  */
  static
  void checkTrueOrFalse( const TagTrue & tag );


}; // end of class ConceptUtils

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConceptUtils_h

#undef ConceptUtils_RECURSES
#endif // else defined(ConceptUtils_RECURSES)
