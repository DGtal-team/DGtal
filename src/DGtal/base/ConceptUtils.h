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
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
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
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Maccros to check for concept
//    
// C++20 doesn't currently allow typedefs in requires clauses. This is a 
// problem because DGtal may use many nested types, which makes it unclear 
// what is checked and what the proper requirements are. Instead, we use a 
// small macro system that enables typedefs to be forwarded.
// 
// Example usage: 
//    template <typename T>
//    class Test { 
//       using CRef = const T&;
//       DGTAL_CONCEPT_CHECK( requires SomeConcept<CRef> ); 
//    };
// It can also be used inside functions:
// 
//    void func() {
//        using Type = int;
//        DGTAL_CONCEPT_CHECK( requires SomeConcept<Type> );
//    }
// 
//
// The preprocessor may peform substitution only if no concatenation (##) is 
// used. Therefore, we need to add one level of indirection for the compiler
// to expand __LINE__ maccro properly
#define DGTAL_VARIABLE_NAME_IMPL(line) concept_check_ ## line 
#define DGTAL_VARIABLE_NAME(line) DGTAL_VARIABLE_NAME_IMPL(line) 
//
// Main macro to check for concept
//
// In short, it creates a variable that is constructed by an immediate call
// to a lambda (IIFE).
// 
// It can be used within both class declaration and functions. 
//
// Notes:
// - The macro is not terminated by semi-colon to avoid warnings with clang.
// - This is a variadic macro. For concepts using multiple types, the ','
//   will parsed as different arguments to the macro; which causes the macro
//   to be called with different number of argument each time. Another option
//   would be to use double parentheses when invoking the macro; but errors 
//   generated when requirements are not met become less clear. 
// - The use of __LINE__ here is to give a unique identifier, __COUNTER__
//   could also be used, this is arbitrary choice.
// - The variable is static which means it costs one byte per class and not 
//   one byte per instance.
// - The variable is also constexpr which further implies the inline property 
//   without naming it. This allows for both direct initialization without 
//   redeclaration in class and to use the macro inside functions (inline 
//   cannot be used at block scope levels).
// - The type is 'auto' which is necessary to force the compiler to instantiate
//   the variable (as it is static, it is instantiated independently) and thus
//   to check the concept.  
// - The x variable is here to implicitly template the lambda, allowing for the
//   requires clause to exist (here, on the operator(), not on the class). This
//   is important because the template keyword is disallowed inside functions. 
#define DGTAL_CONCEPT_CHECK(...) \
  static constexpr auto DGTAL_VARIABLE_NAME(__LINE__) = [](auto x) -> char __VA_ARGS__ { return 0; }(0)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  /**
   * Define the negate of a tag: Negate<TagFalse>::value=TagTrue and
   * Negate<TagFalse>::value=TagFalse.
   */
  template<typename T>
  struct Negate
  { typedef TagUnknown type; };
  template<>
  struct Negate<TagTrue>
  { typedef TagFalse type; };
  template<>
  struct Negate<TagFalse>
  { typedef TagTrue type; };

  /**
     This is a helper class that provides a way to get a reference to
     an object. Calls boost::static_object. Used only for defining
     archetypes. Write DummyObject<X>::get() to get a reference to a
     dummy static object of type X.
  */
  template <class T>
  struct DummyObject : public boost::static_object<T>
  {};

/////////////////////////////////////////////////////////////////////////////
// Namespace ConceptUtils
/**
 * Description of namespace 'ConceptUtils' <p>
 *
 * \brief Aim: Gathers several functions useful for concept checks.
 *
 */
  namespace concepts
  {
namespace ConceptUtils
{
  /**
   * Test if the two types T1 and T2 are equals.
   * value = true if yes, false otherwise.
   */
  template<typename T1, typename T2>
  struct SameType
  { static const bool value = false; };

  template<typename T>
  struct SameType<T,T>
  { static const bool value = true; };
  
  /**
   * Type deduction will fail unless the arguments have the same type.
   */
  template <typename T>
  void sameType( const T & /* t1 */, const T & /* t2 */ ) {}

  /**
   * Test if T is equal to TagTrue.
   */
  template<typename T>
  struct CheckTrue
  { static const bool value = false; };
  template<>
  struct CheckTrue<TagTrue>
  { static const bool value = true; };

  /**
   * Test if T is equal to TagFalse.
   */
  template<typename T>
  struct CheckFalse
  { static const bool value = !CheckTrue<T>::value; };

  /**
   * Test if T is equal to TagUnknown.
   */
  template<typename T>
  struct CheckUnknown
  { static const bool value = false; };
  template<>
  struct CheckUnknown<TagUnknown>
  { static const bool value = true; };  
  
  /**
   * Test if T is equal to TagTrue or TagFalse.
   */
  template<typename T>
  struct CheckTrueOrFalse
  { static const bool value = CheckTrue<T>::value||CheckFalse<T>::value; };
  
  /**
   * Test if T is a tag (ie equal to TagTrue or TagFalse, or TagUnknown)
   */
  template<typename T>
  struct CheckTag
  { static const bool value = CheckTrueOrFalse<T>::value||CheckUnknown<T>::value; };  
  
  /**
     Type deduction will fail unless the argument type is exactly TagTrue.
     @param tag the type to check.
  */
  void checkTrue( const TagTrue & tag );

  /**
     Type deduction will fail unless the argument type is exactly TagFalse.
     @param tag the type to check.
  */
  void checkFalse( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is exactly TagUnknown.
     @param tag the type to check.
  */
  void checkUnknown( const TagUnknown & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  void checkTag( const TagUnknown & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  void checkTag( const TagTrue & tag );

  /**
     Type deduction will fail unless the argument type is a tag (TagTrue, TagFalse or TagUnknown).
     @param tag the type to check.
  */
  void checkTag( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is the tag TagTrue or TagFalse.
     @param tag the type to check.
  */
  void checkTrueOrFalse( const TagFalse & tag );

  /**
     Type deduction will fail unless the argument type is the tag TagTrue or TagFalse.
     @param tag the type to check.
  */
  void checkTrueOrFalse( const TagTrue & tag );


} // end of namespace ConceptUtils
  } //end of namespace concepts.
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConceptUtils_h

#undef ConceptUtils_RECURSES
#endif // else defined(ConceptUtils_RECURSES)
