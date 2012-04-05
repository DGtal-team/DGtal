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
 * @file IteratorAdapter.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Header file for module IteratorAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IteratorAdapter_RECURSES)
#error Recursive header files inclusion detected in IteratorAdapter.h
#else // defined(IteratorAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IteratorAdapter_RECURSES

#if !defined IteratorAdapter_h
/** Prevents repeated inclusion of headers. */
#define IteratorAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/CUnaryFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class IteratorAdapter
  /**
   * Description of template class 'IteratorAdapter' <p>
   * \brief This class adapts any iterator (at least forward)
   * so that operator* returns a member on the element pointed 
   * to by the iterator, instead the element itself.
   *
   * @tparam TIterator the type of the iterator to adapt
   * (at least forward) 
   *
   * To achieve this goal, the adapter is based on a functor f
   * given at construction so that operator* calls f(*it), 
   * instead of calling directly operator* of the underlying 
   * iterator it. This operation returns a reference (or constant
   * reference) on a member of the element pointed to by the 
   * iterator, which can be read as well as assigned (if the 
   * member is not constant).  
   *
   * @tparam TFunctor the type of functor that transforms
   * the pointed element into another one
   *
   * @tparam TReturnType the type of the element returned by the underlying functor
   *
   * NB: the underlying functor is stored in the adapter as aliasing pointer
   * in order to avoid copies. As a consequence the pointed object must exist 
   * and must not be deleted during the use of the adapter.
   */
  template <typename TIterator, typename TFunctor, typename TReturnType>
  class IteratorAdapter
  {

    BOOST_CONCEPT_ASSERT(( boost::ForwardIterator<TIterator> ));
    typedef typename IteratorCirculatorTraits<TIterator>::Value TArgument;  

    //--------------- inner types --------------------------------
  public: 
    
    typedef IteratorAdapter<TIterator, TFunctor, TReturnType> Self;
    typedef TIterator Iterator;
    typedef TFunctor Functor;
  
    typedef TReturnType value_type; 
    typedef value_type* pointer;
    typedef value_type& reference;
    typedef typename iterator_traits<TIterator>::difference_type difference_type;
    typedef typename iterator_traits<TIterator>::iterator_category iterator_category;

  private: 

    typedef const Functor* FunctorPtr; 
  
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
  
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ----------------------- Standard services ------------------------------
  public:
    /**
     *  The default constructor default-initializes the members
     */
    IteratorAdapter() 
      : myCurrentIt(), myFunctorPtr() { }
    
    /**
     *  Constructor.
     * @param it an iterator to adapt
     * @param f the functor that transforms
     * the pointed element into another element
     */
    IteratorAdapter(const Iterator& it, const Functor& f) 
      : myCurrentIt(it), myFunctorPtr(&f) { }

    /**
     *  Copy constructor.
     * @param other an iterator adapter
     */
    IteratorAdapter(const IteratorAdapter& other)
      : myCurrentIt(other.myCurrentIt), 
	myFunctorPtr(other.myFunctorPtr) { }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    IteratorAdapter & operator= ( const IteratorAdapter & other ) 
    {
      if ( this != &other )
        {
          myCurrentIt = other.myCurrentIt;
          myFunctorPtr = other.myFunctorPtr;
        }
      return *this;
    }

    /**
     * Destructor.
     */
    ~IteratorAdapter() {}

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
     *  @return constant reference on a member of the element pointed to by @a myCurrentIt.
     */
    const reference operator*() const 
    { 
      BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, const TArgument&, const TReturnType& > )); 
      return myFunctorPtr->operator()(*myCurrentIt); 
    }

    /**
     *  @return constant pointer on a member of the element pointed to by @a myCurrentIt.
     */
    const pointer operator->() const
    {
      return & this->operator*(); 
    }

    /**
     *  @return reference on a member of the element pointed to by @a myCurrentIt.
     */
    reference operator*()
    { 
      BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, TArgument&, TReturnType& > )); 
      return myFunctorPtr->operator()(*myCurrentIt); 
    }

    /**
     *  @return pointer on a member of the element pointed to by @a myCurrentIt.
     */
    pointer operator->()
    {
      return & this->operator*(); 
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

  }; // end of class IteratorAdapter

  template <typename TIterator, typename TFunctor, typename TReturnType >
  IteratorAdapter<TIterator,TFunctor,TReturnType> 
  operator+(typename IteratorCirculatorTraits<TIterator>::Difference d, 
	    IteratorAdapter<TIterator,TFunctor,TReturnType> & object )
  {
    IteratorAdapter<TIterator,TFunctor,TReturnType> tmp = object;
    return tmp += d;
  }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/IteratorAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IteratorAdapter_h

#undef IteratorAdapter_RECURSES
#endif // else defined(IteratorAdapter_RECURSES)
