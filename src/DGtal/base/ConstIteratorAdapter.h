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
#include "DGtal/base/Circulator.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstIteratorAdapter
  /**
   * Description of template class 'ConstIteratorAdapter' <p>
   * \brief This class adapts any iterator (at least forward)
   * so that operator* returns another element than the one
   * pointed to by the iterator.
   *
   * @tparam TIterator the type of the iterator to adapt
   * (at least forward) 
   *
   * To achieve this goal, the adapter is based on a functor f
   * given at construction so that operator* calls f(*it), 
   * instead of calling directly operator* of the underlying 
   * iterator it.
   *
   * @tparam TFunctor the type of functor that transforms
   * the pointed element into another one
   *
   * @tparam TReturnType the type of the element returned by the underlying functor
   *
   * 
   * NB1: The dereference operator should be used to get the returned element or to access
   * to its members. The indirection operator has been implemented for completeness sake, 
   * but each time the operator is called, the functor is applied and the returned element 
   * is stored into a buffer. As a consequence, the indirection operator can be used 
   * without extra costs only if one want to access to only one of its members. 
   *
   * NB: the underlying functor is stored in the adapter as aliasing pointer
   * in order to avoid copies. As a consequence the pointed object must exist 
   * and must not be deleted during the use of the adapter.
   */
  template <typename TIterator, typename TFunctor, typename TReturnType>
  class ConstIteratorAdapter
  {

    BOOST_CONCEPT_ASSERT(( boost::ForwardIterator<TIterator> )); 

    //--------------- inner types --------------------------------
  public: 
    
    typedef ConstIteratorAdapter<TIterator, TFunctor, TReturnType> Self;
    typedef TIterator Iterator;
    typedef TFunctor Functor;
  
    typedef TReturnType value_type; 
    typedef const value_type* pointer;
    typedef const value_type& reference;
    typedef typename iterator_traits<TIterator>::difference_type difference_type;
    typedef typename iterator_traits<TIterator>::iterator_category iterator_category;

    //value_type should be default-constructible, 
    //since an iterator is default-constructible
    BOOST_CONCEPT_ASSERT(( boost::DefaultConstructible<value_type> ));

  private: 

    typedef const Functor* FunctorPtr; 

    typedef CountedPtr<value_type> BufferPtr; 
  
    // ------------------------- Protected Datas ------------------------------
  protected:
    /**
     * Underlying iterator
     */
    Iterator myCurrentIt;
    /**
     * Aliasing pointer on a (constant) functor
     */
    FunctorPtr myFunctorPtr; 
    /**
     * Pointer on a buffer used to temporarily stored the element 
     * returned by @a myFunctor( *myCurrentIt )
     */
    mutable BufferPtr myBufferPtr;
  
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ----------------------- Standard services ------------------------------
  public:
    /**
     *  The default constructor default-initializes the members
     */
    ConstIteratorAdapter() 
      : myCurrentIt(), myFunctorPtr(), myBufferPtr() { }

    
    /**
     *  Constructor.
     * @param it an iterator to adapt
     * @param f the functor that transforms
     * the pointed element into another element
     */
    ConstIteratorAdapter(const Iterator& it, const Functor& f) 
      : myCurrentIt(it), myFunctorPtr(&f), myBufferPtr(new value_type()) { }

    /**
     *  Copy constructor.
     * @param other an iterator adapter
     */
    ConstIteratorAdapter(const ConstIteratorAdapter& other)
      : myCurrentIt(other.myCurrentIt), 
	myFunctorPtr(other.myFunctorPtr), 
	myBufferPtr(other.myBufferPtr) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ConstIteratorAdapter & operator= ( const ConstIteratorAdapter & other ) 
    {
      if ( this != &other )
        {
          myCurrentIt = other.myCurrentIt;
          myFunctorPtr = other.myFunctorPtr;
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
     *  @return the modified element pointed to by @a myCurrentIt.
     */
    reference operator*() const 
    { 
      *myBufferPtr = myFunctorPtr->operator()(*myCurrentIt); 
      return myBufferPtr.operator*(); 
    }

    /**
     *  @return  pointer to the modified element stored in @a myBufferPtr.
     */
    pointer operator->() const
    {
      *myBufferPtr = myFunctorPtr->operator()(*myCurrentIt);
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

  template <typename TIterator, typename TFunctor, typename TReturnType >
  ConstIteratorAdapter<TIterator,TFunctor,TReturnType> 
  operator+(typename IteratorCirculatorTraits<TIterator>::Difference d, 
	    ConstIteratorAdapter<TIterator,TFunctor,TReturnType> & object )
  {
    ConstIteratorAdapter<TIterator,TFunctor,TReturnType> tmp = object;
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
