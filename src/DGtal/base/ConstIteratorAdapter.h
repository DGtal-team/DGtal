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
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstAdapterIterator
  /**
   * Description of template class 'ConstAdapterIterator' <p>
   * \brief Aim: Any iterator (at least forward) can be adapted
   * so that operator* returns a modified element
   * @tparam TIterator the type of the iterator to adapt
   * @tparam TModifier the type of the object that transform
   * the value type of TIterator into another type
   */
  template <typename TIterator, typename TModifier>
  class ConstAdapterIterator : public TIterator
  {

  //--------------- inner types --------------------------------
  public: 
    
    typedef ConstAdapterIterator<TIterator, TModifier> Self;
    typedef TIterator Iterator;
    typedef TModifier Modifier;
    typedef typename TModifier::Output Value; 

  // ------------------------- Protected Datas ------------------------------
  protected:
    Iterator myCurrentIt;
    Value myBuffer;
  
  // ------------------------- Private Datas --------------------------------
  private:
    
    // ----------------------- Standard services ------------------------------
  public:
      /**
       *  The default constructor default-initializes member @a myCurrentIt.
      */
    ConstAdapterIterator() : myCurrentIt(), myBuffer { }

      /**
       *  Constructor.
      */
      explicit
    ConstAdapterIterator(Iterator it) : myCurrentIt(it), myBuffer() { }

    /**
     *  Copy constructor.
     */
    ConstAdapterIterator(const ConstAdapterIterator& aIt)
    : myCurrentIt(aIt.myCurrentIt), myBuffer(aIt.myBuffer()) { }

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
          myBuffer = other.myBuffer;
        }
      return *this;
    }

    /**
     * Destructor.
     */
    ~ConstIteratorAdapter();

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
    Value operator*() const { 
      myBuffer = Modifier::get(*myCurrentIt);
      return myBuffer; 
    }

    /**
     *  @return  pointer to the modified element stored in @a myBuffer.
    */
    pointer operator->() const { 
      return &myBuffer; 
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

    // ----------------------- Comparisons operators and equality operators --------------------------------------
    //not defined because of the inheritance 

    // ------------------------- Hidden services ------------------------------


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ConstIteratorAdapter

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/ConstIteratorAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConstIteratorAdapter_h

#undef ConstIteratorAdapter_RECURSES
#endif // else defined(ConstIteratorAdapter_RECURSES)
