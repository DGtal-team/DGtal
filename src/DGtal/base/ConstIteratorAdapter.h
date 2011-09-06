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
 * @file ConstIteratorAdapter.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Header file for module ConstIteratorAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConstIteratorAdapter_RECURSES)
#error Recursive header files inclusion detected in ConstIteratorAdapter.h
#else // defined(ConstIteratorAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstIteratorAdapter_RECURSES

#if !defined ConstIteratorAdapter_h
/** Prevents repeated inclusion of headers. */
#define ConstIteratorAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstIteratorAdapter
  /**
   * Description of template class 'ConstIteratorAdapter' <p>
   * \brief Any iterator (at least forward) can be adapted
   * so that operator* returns a modified element of a given type
   * instead the element pointed to by the iterator. 
   * @tparam TIterator the type of the iterator to adapt
   * @tparam TModifier the type of the object that transforms
   * the pointed element into an element of another type
   *
   * NB: The dereference operator should be used to get the modified element or to access
   * to its members. The indirection operator has been implemented for completeness sake, 
   * but each time the operator is called, it stores the modified element into a buffer. As a 
   * consequence, the indirection operator can be used without extra costs only if one want 
   * to access to only one member of the modified element. 
   *
   * @see Modifier.h
   */
  template <typename TIterator, typename TModifier>
  class ConstIteratorAdapter //: public TIterator

  {

  //--------------- inner types --------------------------------
  public: 
    
    typedef ConstIteratorAdapter<TIterator, TModifier> Self;
    typedef TIterator Iterator;
    typedef TModifier Modifier;
  
    typedef typename TModifier::Output value_type; 
    typedef value_type* pointer;
    typedef value_type& reference;
    typedef typename iterator_traits<TIterator>::difference_type difference_type;
    typedef typename iterator_traits<TIterator>::iterator_category iterator_category;
  
    typedef CountedPtr<value_type> BufferPtr; 
  
  // ------------------------- Protected Datas ------------------------------
  protected:
    Iterator myCurrentIt;
    Modifier myModifier; 
    BufferPtr myBufferPtr;
  
  // ------------------------- Private Datas --------------------------------
  private:
    
    // ----------------------- Standard services ------------------------------
  public:
      /**
       *  The default constructor default-initializes the members
      */
    ConstIteratorAdapter() 
    : myCurrentIt(), myModifier(), myBufferPtr() { }

      /**
       *  Standard constructor.
       * @param it an iterator to adapt
      */
      explicit
    ConstIteratorAdapter(Iterator it) 
    : myCurrentIt(it), myModifier(), myBufferPtr(new value_type()) { }
    
     /**
       *  Constructor.
       * @param it an iterator to adapt
       * @param m the modifier that transforms
       * the pointed element into an element of another type
      */
      explicit
    ConstIteratorAdapter(Iterator it, Modifier m) 
    : myCurrentIt(it), myModifier(m), myBufferPtr(new value_type(m.get(*it))) { }

    /**
     *  Copy constructor.
     * @param other an iterator adapter
     */
    ConstIteratorAdapter(const ConstIteratorAdapter& other)
    : myCurrentIt(other.myCurrentIt), myModifier(other.myModifier), myBufferPtr(other.myBufferPtr) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ConstIteratorAdapter & operator= ( const ConstIteratorAdapter & other ) 
    {
      if ( this != &other )
        {
          myCurrentIt = other.myCurrentIt;
          myModifier = other.myModifier;
          myBufferPtr = other.myBufferPtr;
        }
      return *this;
    }

    /**
     * Destructor.
     */
    ~ConstIteratorAdapter() {}

    // ----------------------- Interface --------------------------------------
  public:

    /**
     *  @return  member @a myCurrentIt, the underlying iterator.
     */
    Iterator base() const
    { return myCurrentIt; }


    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const { return true;}

    /**
     *  @return the modified element pointed be @a myCurrentIt.
    */
    reference operator*() const 
    { 
      BufferPtr aLocalBufferPtr = myBufferPtr; 
      *aLocalBufferPtr = myModifier.get(*myCurrentIt); //hack to write in the buffer in a const method
      return myBufferPtr.operator*(); 
    }

    /**
     *  @return  pointer to the modified element stored in @a myBufferPtr.
    */
    pointer operator->() const
    {
      BufferPtr aLocalBufferPtr = myBufferPtr; 
      *aLocalBufferPtr = myModifier.get(*myCurrentIt); //hack to write in the buffer in a const method
      return myBufferPtr.operator->(); 
    }

    /**
     *  Pre-increment
     */
    Self& operator++()
    {
      ++myCurrentIt;
      return *this;
    }

      /**
      * Post-increment
      */
    Self operator++(int)
    {
      Self tmp = *this;
      operator++(); 
      return tmp;
    }


     /**
      *  Pre-decrement
      */
    Self& operator--()
    {
      --myCurrentIt;
      return *this;
    }

      /**
      * Post-decrement
      */
    Self operator--(int)
    {
      Self tmp = *this;
      operator--(); 
      return tmp;
    }

        // ----------------------- Random access operators --------------------------------------
  public:

    Self& operator+=( difference_type d ) {
      myCurrentIt += d;
      return *this;
    }
    Self operator+( difference_type d) const {
      Self tmp = *this;
      return tmp += d;
    }
    Self operator-( difference_type d) const {
      Self tmp = *this;
      return tmp += -d;
    }
    Self& operator-=( difference_type d) { return operator+=( -d); }

    difference_type operator-( const Self& other) const {
      return myCurrentIt - other.myCurrentIt;
    }
    reference operator[]( difference_type d) const {
      Self tmp = *this;
      tmp += d;
      return *tmp;
    }

    /**
    *  Equality operator
      */
    bool operator==( const Self& other) const 
    { 
      return (myCurrentIt == other.myCurrentIt);
    }
        /**
    *  difference_type operator
      */
    bool operator!=( const Self& other) const 
    { 
      return !(*this == other); 
    }
    
    // ----------------------- Comparisons operators --------------------------------------
    /**
    *  Less operator
      */
    bool operator<( const Self& other) const 
    { 
      return (myCurrentIt < other.myCurrentIt);
    }
        /**
    *  Less or equal operator
      */
    bool operator<=( const Self& other) const 
    { 
      return (myCurrentIt <= other.myCurrentIt);
    }
            /**
    *  Greater
      */
    bool operator>( const Self& other) const 
    { 
      return (myCurrentIt > other.myCurrentIt);
    }
        /**
    *  Greater or equal operator
      */
    bool operator>=( const Self& other) const 
    { 
      return (myCurrentIt >= other.myCurrentIt);
    }
    
    // ------------------------- Hidden services ------------------------------


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ConstIteratorAdapter

  template <typename TIterator, typename TModifier >
  ConstIteratorAdapter<TIterator,TModifier> operator+( 
    typename iterator_traits<TIterator>::difference_type d, 
    ConstIteratorAdapter<TIterator,TModifier> & object )
  {
    ConstIteratorAdapter<TIterator,TModifier> tmp = object;
    return tmp += d;
  }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/ConstIteratorAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConstIteratorAdapter_h

#undef ConstIteratorAdapter_RECURSES
#endif // else defined(ConstIteratorAdapter_RECURSES)
