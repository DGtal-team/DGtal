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
// tag classes dedicated for circulators only
struct forward_circulator_tag 
{
  operator std::forward_iterator_tag() { return std::forward_iterator_tag(); }
};
struct bidirectional_circulator_tag : forward_circulator_tag
{
  operator std::bidirectional_iterator_tag() { return std::bidirectional_iterator_tag(); }
};
struct random_access_circulator_tag : bidirectional_circulator_tag
{
  operator std::bidirectional_iterator_tag() { return std::bidirectional_iterator_tag(); }
};

/////////////////////////////////////////////////////////////////////////////
// tag classes for traversal category (Foward, Bidirectional, RandomAccess) 
// valid for both iterator and circulator
struct ForwardCategory {};
struct BidirectionalCategory : ForwardCategory {};
struct RandomAccessCategory : BidirectionalCategory {}; 

/////////////////////////////////////////////////////////////////////////////
// traits classes 

/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorToCirculatorTagTraits' <p>
* \brief Aim: 
*  Transform the STL XXX_iterator_tag into XXX_circulator_tag.
*  The tag is provided by the nested type @a iterator_category
* @tparam T any tag
*/

template <typename T>
struct IteratorToCirculatorTagTraits {
    typedef  T  iterator_category;
};

//usual tags
template <>
struct IteratorToCirculatorTagTraits<boost::forward_traversal_tag> {
    typedef  forward_circulator_tag iterator_category;
};
template <>
struct IteratorToCirculatorTagTraits<boost::bidirectional_traversal_tag> {
    typedef  bidirectional_circulator_tag iterator_category;
};
template <>
struct IteratorToCirculatorTagTraits<boost::random_access_traversal_tag> {
    typedef  random_access_circulator_tag iterator_category;
};

//from ConstIteratorAdapter based on boost::transform_iterator
template <>
struct IteratorToCirculatorTagTraits<boost::detail::iterator_category_with_traversal<std::input_iterator_tag,boost::random_access_traversal_tag> > {
    typedef  random_access_circulator_tag iterator_category;
};



/////////////////////////////////////////////////////////////////////////////
/**
* Description of template class 'IteratorAndCirculatorTagTraits' <p>
* \brief Aim: 
*  Provides the category of the iterator (resp. circulator)  
* {ForwardCategory,BidirectionalCategory,RandomAccessCategory}
* and the type {IteratorType,CirculatorType}
* 
* @tparam C any category
*/

//default
template <typename C>
struct IteratorAndCirculatorTagTraits {
    typedef  IteratorType  type;
    typedef  C  iterator_category;
};

//for DGtal circulators
template <>
struct IteratorAndCirculatorTagTraits<forward_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  ForwardCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<bidirectional_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  BidirectionalCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<random_access_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  RandomAccessCategory iterator_category;
};

//for STL iterators
template <>
struct IteratorAndCirculatorTagTraits<std::forward_iterator_tag> {
    typedef  IteratorType  type;
    typedef  ForwardCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<std::bidirectional_iterator_tag> {
    typedef  IteratorType  type;
    typedef  BidirectionalCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<std::random_access_iterator_tag> {
    typedef  IteratorType  type;
    typedef  RandomAccessCategory iterator_category;
};

//for boost traversal categories
template <>
struct IteratorAndCirculatorTagTraits<boost::forward_traversal_tag> {
    typedef  IteratorType  type;
    typedef  ForwardCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<boost::bidirectional_traversal_tag> {
    typedef  IteratorType  type;
    typedef  BidirectionalCategory iterator_category;
};

template <>
struct IteratorAndCirculatorTagTraits<boost::random_access_traversal_tag> {
    typedef  IteratorType  type;
    typedef  RandomAccessCategory iterator_category;
};
template <>
struct IteratorAndCirculatorTagTraits<boost::detail::iterator_category_with_traversal<std::input_iterator_tag,boost::random_access_traversal_tag> > {
    typedef  IteratorType  type;
    typedef  RandomAccessCategory iterator_category;
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

  typedef typename IteratorAndCirculatorTagTraits
          <typename IC::iterator_category>::type
                                                                 Type;

  typedef typename IteratorAndCirculatorTagTraits
          <typename IC::iterator_category>::iterator_category
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
