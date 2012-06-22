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
 * @file IteratorTraits.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/18
 *
 * Header file for module IteratorTraits.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IteratorTraits_RECURSES)
#error Recursive header files inclusion detected in IteratorTraits.h
#else // defined(IteratorTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IteratorTraits_RECURSES

#if !defined IteratorTraits_h
/** Prevents repeated inclusion of headers. */
#define IteratorTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include<iterator>
#include<boost/iterator/iterator_categories.hpp>
#include<boost/iterator/iterator_facade.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// tag classes for type (either Iterator or Circulator)
struct IteratorType {};
struct CirculatorType {};

/////////////////////////////////////////////////////////////////////////////
// tag classes for traversal category (Foward, Bidirectional, RandomAccess) 
// valid for both iterator and circulator
struct ForwardCategory {};
struct BidirectionalCategory : ForwardCategory {};
struct RandomAccessCategory : BidirectionalCategory {}; 

////////////////////////////////////////////////////////////////////////////
namespace detail
{ 

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'HasNestedType' <p>
* \brief Aim: 
*  Checks whether type @IC has a nested type called 'Type' or not.
*  NB: from en.wikipedia.org/wiki/Substitution_failure_is_not_an_error
*  @tparam IC any iterator or circulator
*/
  template <typename IC> 
  struct HasNestedType 
  {
    typedef char yes[1]; 
    typedef char no[2]; 
    
    template <typename C>
    static yes& test(typename C::Type*);

    template <typename C>
    static no& test(...);

    static const bool value = sizeof(test<IC>(0)) == sizeof(yes); 
  };

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IsCirculator' <p>
* \brief Aim: 
*  Checks whether type @IC is a circular or a classical iterator.
*  NB: from en.wikipedia.org/wiki/Substitution_failure_is_not_an_error
*  @tparam IC any iterator or circulator
*/
  template <typename IC, bool flagHasNestedType = false> 
  struct IsCirculator 
  {
    static const bool value = false; 
  };

  template <typename IC> 
  struct IsCirculator<IC,true> 
  {
    //from en.wikipedia.org/wiki/Substitution_failure_is_not_an_error
    typedef char yes[1]; 
    typedef char no[2]; 
    
    static yes& test(CirculatorType);

    static no& test(IteratorType);

    static const bool value = ( sizeof(test(typename IC::Type())) == sizeof(yes) ); 
  };

} //namespace detail

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IsCirculator' <p>
* \brief Aim: 
*  Checks whether type @IC is a circular or a classical iterator.
*
*  @tparam IC any iterator or circulator
*/
template <typename IC> 
struct IsCirculator 
{
  static const bool value = detail::IsCirculator<IC, detail::HasNestedType<IC>::value >::value; 
};

namespace detail
{
/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorCirculatorTypeImpl' <p>
* \brief Aim: 
*  Defines the Iterator or Circulator type as a nested type
*  according to the value of  @a b
*
*  @tparam b a boolean value
*/
  template<bool b = false>
  struct IteratorCirculatorTypeImpl 
  {
    typedef IteratorType Type; 
  };
  template<>
  struct IteratorCirculatorTypeImpl<true> 
  {
    typedef CirculatorType Type; 
  };
}//namespace detail

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorCirculatorType' <p>
* \brief Aim: 
*  Provides the type of  @IC as a nested type.
*
*  @tparam IC any iterator or circulator
*/
template<typename IC>
struct IteratorCirculatorType {
public: 
  typedef typename detail::IteratorCirculatorTypeImpl<IsCirculator<IC>::value >::Type Type; 
};

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorCirculatorTagTraits' <p>
* \brief Aim: 
*  Provides the category of the iterator (resp. circulator)  
* {ForwardCategory,BidirectionalCategory,RandomAccessCategory}
* 
* @tparam C any category
*/

//default
template <typename C>
struct IteratorCirculatorTagTraits {
    typedef  C  Category;
};

//for STL iterators
template <>
struct IteratorCirculatorTagTraits<std::forward_iterator_tag> {
    typedef  ForwardCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<std::bidirectional_iterator_tag> {
    typedef  BidirectionalCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<std::random_access_iterator_tag> {
    typedef  RandomAccessCategory Category;
};

//for boost traversal categories
template <>
struct IteratorCirculatorTagTraits<boost::forward_traversal_tag> {
    typedef  ForwardCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<boost::bidirectional_traversal_tag> {
    typedef  BidirectionalCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<boost::random_access_traversal_tag> {
    typedef  RandomAccessCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<boost::detail::iterator_category_with_traversal<std::input_iterator_tag,boost::forward_traversal_tag> > {
    typedef  ForwardCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<boost::detail::iterator_category_with_traversal<std::input_iterator_tag,boost::bidirectional_traversal_tag> > {
    typedef  BidirectionalCategory Category;
};

template <>
struct IteratorCirculatorTagTraits<boost::detail::iterator_category_with_traversal<std::input_iterator_tag,boost::random_access_traversal_tag> > {
    typedef  RandomAccessCategory Category;
};




/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorTraits' <p>
* \brief Aim: 
*  Provides nested types for both iterators and circulators:   
*  Type, Category, Value, Difference, Pointer and Reference. 
*
* @tparam IC any iterator or circulator
*/

template <typename IC>
struct IteratorCirculatorTraits {

  typedef typename IteratorCirculatorType<IC>::Type
                                                                 Type;

  typedef typename IteratorCirculatorTagTraits
  <typename boost::iterator_category<IC>::type>::Category
                                                                 Category;

  typedef typename IC::value_type                                Value;
  typedef typename IC::difference_type                           Difference;
  typedef typename IC::pointer                                   Pointer;
  typedef typename IC::reference                                 Reference;

};

template <class T>
struct IteratorCirculatorTraits<T*> {
  typedef IteratorType               Type; 
  typedef RandomAccessCategory       Category;
  typedef T                          Value;
  typedef ptrdiff_t                  Difference;
  typedef T*                         Pointer;
  typedef T&                         Reference;
};

template <class T>
struct IteratorCirculatorTraits<T const*>
{
  typedef IteratorType               Type; 
  typedef RandomAccessCategory       Category;
  typedef T                          Value;
  typedef ptrdiff_t                  Difference;
  typedef T const*                   Pointer;
  typedef T const&                   Reference;
};

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/IteratorTraits.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IteratorTraits_h

#undef IteratorTraits_RECURSES
#endif // else defined(IteratorTraits_RECURSES)
