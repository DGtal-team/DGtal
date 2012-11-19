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
 * @file IteratorFunctions.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/18
 *
 * Header file for module IteratorFunctions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IteratorFunctions_RECURSES)
#error Recursive header files inclusion detected in IteratorFunctions.h
#else // defined(IteratorFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IteratorFunctions_RECURSES

#if !defined IteratorFunctions_h
/** Prevents repeated inclusion of headers. */
#define IteratorFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include<iterator>
#include "DGtal/base/IteratorTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template functions isNotEmpty

  namespace detail 
  {

    /**
     * Checks if the range of classical iterators [ @a itb , @a ite ) is not empty, 
     * ie. checks if itb != ite
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @tparam any iterator or circulator
     */
    template< typename IC > 
    inline
    bool isNotEmpty( const IC& itb, const IC& ite, IteratorType ) 
    {
      return (itb != ite);
    }

    /**
     * Checks if a circular range is not empty, 
     * ie. checks if the circulators are valid. 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @tparam any iterator or circulator
     */
    template< typename IC > 
    inline
    bool isNotEmpty( const IC& c1, const IC& c2, CirculatorType) 
    {
      // using isValid method does not work with reverse circulator 
      //(generally speaking adapters of circulators that does not have any isValid method)
      //    return ( ( c1.isValid() ) && ( c2.isValid() ) );  
      IC c; //c is not valid
      return ( (c1 != c) && (c2 != c) ); 
    }

  } //end namespace detail

  /**
   * Checks if the range [ @a itb , @a ite ) is empty
   * @param itb begin iterator of the range
   * @param ite end iterator of the range
   * @tparam any iterator or circulator
   */
  template< typename IC> 
  inline
  bool isEmpty( const IC& itb, const IC& ite )
  {
    return !detail::isNotEmpty<IC>( itb, ite, typename IteratorCirculatorTraits<IC>::Type() );
  }

  /**
   * Checks if the range [ @a itb , @a ite ) is not empty
   * @param itb begin iterator of the range
   * @param ite end iterator of the range
   * @tparam any iterator or circulator
   */
  template< typename IC> 
  inline
  bool isNotEmpty( const IC& itb, const IC& ite )
  {
    return detail::isNotEmpty<IC>( itb, ite, typename IteratorCirculatorTraits<IC>::Type() );
  }
  
  /////////////////////////////////////////////////////////
  // template functions for the size and middle of a (sub)range

  /**
   * Moves @a ic at position @ it + @a n 
   * @param ic any (circular)iterator
   * @param n any positive distance
   * @return moved (circular)iterator
   * @tparam any iterator or circulator
   */
  template<typename IC>
  void advanceIterator(IC& ic, unsigned int n);  

  namespace detail
  {
    /**
     * Moves @a ic at position @ it + @a n 
     * @param ic any (circular)iterator
     * @param n any positive distance
     * @param c any instance of ForwardCategory
     * @return (circular)iterator @a it incremented @a n times
     * @tparam any iterator or circulator
     */
    template<typename IC>
    void advanceIterator(IC& ic, unsigned int n, ForwardCategory /*c*/);

    /**
     * Moves @a ic at position @ it + @a n 
     * @param ic any (circular)iterator
     * @param n any positive distance
     * @param c any instance of RandomAccessCategory
     * @return  @a it += @a n
     * @tparam any iterator or circulator
     */
    template<typename IC>
    void advanceIterator(IC& ic, unsigned int n, RandomAccessCategory /*c*/);
  } //end namespace detail

  /**
   * Computes the size of a given range [ @a itb , @a ite ) 
   * @param itb begin iterator of the range
   * @param ite end iterator of the range
   * @return the size 
   * @tparam any iterator or circulator
   */
  template<typename IC>
  unsigned int rangeSize(const IC& itb, const IC& ite);  

  namespace detail
  {
    /**
     * Computes the size of a given range [ @a itb , @a ite ) 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @param t any object of IteratorType 
     * @param c any object of ForwardCategory 
     * @return the size 
     * NB: in O(ite-itb)
     * @tparam any iterator
     */
    template<typename I>
    unsigned int rangeSize(const I& itb, const I& ite, IteratorType /*t*/, ForwardCategory /*c*/); 

    /**
     * Computes the size of a given range [ @a cb, @a ce ). 
     * Note that if @a cb = @a ce then [ @a cb, @a ce ) is assumed to be a whole range.  
     * @param cb begin iterator of the range
     * @param ce end iterator of the range
     * @param t any object of CirculatorType 
     * @param c any object of ForwardCategory 
     * @return the size 
     * NB: linear in the range size
     * @tparam any circulator
     */
    template<typename C>
    unsigned int rangeSize(const C& cb, const C& ce, CirculatorType /*t*/, ForwardCategory /*c*/);

    /**
     * Computes the size of a given range [ @a itb , @a ite ) 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @param t any object of IteratorType 
     * @param c any object of ForwardCategory 
     * @return the size 
     * NB: in O(1)
     * @tparam any iterator
     */
    template<typename I>
    unsigned int rangeSize(const I& itb, const I& ite, IteratorType /*t*/, RandomAccessCategory /*c*/); 

    /**
     * Computes the size of a given range [ @a cb, @a ce ). 
     * Note that if @a cb = @a ce then [ @a cb, @a ce ) is assumed to be a whole range.  
     * @param cb begin iterator of the range
     * @param ce end iterator of the range
     * @param t any object of CirculatorType 
     * @param c any object of ForwardCategory 
     * @return the size 
     * NB: in O(1)
     * @tparam any circulator
     */
    template<typename C>
    unsigned int rangeSize(const C& cb, const C& ce, CirculatorType /*t*/, RandomAccessCategory /*c*/);
 
  } //namespace detail

  /**
   * Computes the middle iterator of a given range [ @a itb , @a ite ) 
   * @param itb begin iterator of the range
   * @param ite end iterator of the range
   * @return the middle iterator of the range [ @a itb , @a ite ) 
   * @tparam any iterator or circulator
   */
  template<typename IC>
  IC rangeMiddle(const IC& itb, const IC& ite);  

  namespace detail
  {
    /**
     * Computes the middle of a given range [ @a itb , @a ite ) 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @param t any object of IteratorType 
     * @param c any object of ForwardCategory 
     * @return the middle iterator of the range [ @a itb , @a ite ) 
     * NB: in O(ite-itb)
     * @tparam any iterator
     */
    template<typename I>
    I rangeMiddle(const I& itb, const I& ite, IteratorType /*t*/, ForwardCategory /*c*/); 

    /**
     * Computes the middle of a given range [ @a cb, @a ce ). 
     * Note that if @a cb = @a ce then [ @a cb, @a ce ) is assumed to be a whole range.  
     * @param cb begin iterator of the range
     * @param ce end iterator of the range
     * @param t any object of CirculatorType 
     * @param c any object of ForwardCategory 
     * @return the middle circulator of the range [ @a cb , @a ce ) 
     * NB: linear in the range size
     * @tparam any circulator
     */
    template<typename C>
    C rangeMiddle(const C& cb, const C& ce, CirculatorType /*t*/, ForwardCategory /*c*/); 

    /**
     * Computes the middle of a given range [ @a itb , @a ite ) 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @param t any object of IteratorType 
     * @param c any object of BidirectionalCategory 
     * @return the middle iterator of the range [ @a itb , @a ite ) 
     * NB: in O(ite-itb)
     * @tparam any iterator
     */
    template<typename I>
    I rangeMiddle(const I& itb, const I& ite, IteratorType /*t*/, BidirectionalCategory /*c*/); 

    /**
     * Computes the middle of a given range [ @a cb, @a ce ). 
     * Note that if @a cb = @a ce then [ @a cb, @a ce ) is assumed to be a whole range.  
     * @param cb begin iterator of the range
     * @param ce end iterator of the range
     * @param t any object of CirculatorType 
     * @param c any object of BidirectionalCategory 
     * @return the middle circulator of the range [ @a cb , @a ce ) 
     * NB: linear in the range size
     * @tparam any circulator
     */
    template<typename C>
    C rangeMiddle(const C& cb, const C& ce, CirculatorType /*t*/, BidirectionalCategory /*c*/); 

    /**
     * Computes the middle of a given range [ @a itb , @a ite ) 
     * @param itb begin iterator of the range
     * @param ite end iterator of the range
     * @param t any object of IteratorType 
     * @param c any object of RandomAccessCategory 
     * @return the middle iterator of the range [ @a itb , @a ite ) 
     * NB: in O(1)
     * @tparam any iterator
     */
    template<typename I>
    I rangeMiddle(const I& itb, const I& ite, IteratorType /*t*/, RandomAccessCategory /*c*/); 

    /**
     * Computes the middle of a given range [ @a cb, @a ce ). 
     * Note that if @a cb = @a ce then [ @a cb, @a ce ) is assumed to be a whole range.  
     * @param cb begin iterator of the range
     * @param ce end iterator of the range
     * @param t any object of CirculatorType 
     * @param c any object of RandomAccessCategory 
     * @return the middle circulator of the range [ @a cb , @a ce ) 
     * NB: in O(1)
     * @tparam any circulator
     */
    template<typename C>
    C rangeMiddle(const C& cb, const C& ce, CirculatorType /*t*/, RandomAccessCategory /*c*/); 
  } // namespace namespace


} // namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/IteratorFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IteratorFunctions_h

#undef IteratorFunctions_RECURSES
#endif // else defined(IteratorFunctions_RECURSES)
