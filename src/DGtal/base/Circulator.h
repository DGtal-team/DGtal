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
 * @file Circulator.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/07/05
 *
 * Header file for module Circulator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Circulator_RECURSES)
#error Recursive header files inclusion detected in Circulator.h
#else // defined(Circulator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Circulator_RECURSES

#if !defined Circulator_h
/** Prevents repeated inclusion of headers. */
#define Circulator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include<iterator>

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // tag classes 
struct IteratorType {};
struct CirculatorType {};

  //tags dedicated for circulators only
struct forward_circulator_tag {};
struct bidirectional_circulator_tag {};
struct random_access_circulator_tag {};


  //tags valid for both iterator or circulator
struct ForwardCategory {};
struct BidirectionalCategory {};
struct RandomAccessCategory {};

  /////////////////////////////////////////////////////////////////////////////
  // traits classes 

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of template class 'IteratorCirculatorTagTraits' <p>
   * \brief Aim: 
   *  Provides the category of the iterator (resp. circulator)  
   * {ForwardCategory,BidirectionalCategory,RandomAccessCategory}
   * and the type {IteratorType,CirculatorType}
   * 
   * @tparam C any category
   */

//default
template <typename C>
struct IteratorCirculatorTagTraits {
    typedef  IteratorType  type;
    typedef  C  iterator_category;
};

//for DGtal circulators
template <>
struct IteratorCirculatorTagTraits<forward_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  ForwardCategory iterator_category;
};

template <>
struct IteratorCirculatorTagTraits<bidirectional_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  BidirectionalCategory iterator_category;
};

template <>
struct IteratorCirculatorTagTraits<random_access_circulator_tag> {
    typedef  CirculatorType  type;
    typedef  RandomAccessCategory iterator_category;
};

//for STL iterators
template <>
struct IteratorCirculatorTagTraits<std::forward_iterator_tag> {
    typedef  IteratorType  type;
    typedef  ForwardCategory iterator_category;
};

template <>
struct IteratorCirculatorTagTraits<std::bidirectional_iterator_tag> {
    typedef  IteratorType  type;
    typedef  BidirectionalCategory iterator_category;
};

template <>
struct IteratorCirculatorTagTraits<std::random_access_iterator_tag> {
    typedef  IteratorType  type;
    typedef  RandomAccessCategory iterator_category;
};

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of template class 'IteratorCirculatorTraits' <p>
   * \brief Aim: 
   *  Provides definition types for both iterators and circulators:   
   *  Type, Category, Value, Difference, Pointer and Reference. 
   *
   * @tparam IC any iterator or circulator
   */

template <typename IC>
struct IteratorCirculatorTraits {

  typedef typename IteratorCirculatorTagTraits<typename IC::iterator_category>::type
                                                                 Type;

  typedef typename IteratorCirculatorTagTraits<typename IC::iterator_category>::iterator_category
                                                                 Category;

  typedef typename iterator_traits<IC>::value_type               Value;
  typedef typename iterator_traits<IC>::difference_type          Difference;
  typedef typename iterator_traits<IC>::pointer                  Pointer;
  typedef typename iterator_traits<IC>::reference                Reference;

};



  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of template class 'CirculatorTagTraits' <p>
   * \brief Aim: 
   *  Transform std::forward_iterator_tag into forward_circulator_tag
   *  Transform std::bidirectional_iterator_tag into bidirectional_circulator_tag
   *  Transform std::random_access_iterator_tag into random_access_circulator_tag
   * 
   * @tparam T any tag
   */

template <typename T>
struct CirculatorTagTraits {
    typedef  T  iterator_category;
};

template <>
struct CirculatorTagTraits<std::forward_iterator_tag> {
    typedef  forward_circulator_tag iterator_category;
};


template <>
struct CirculatorTagTraits<std::bidirectional_iterator_tag> {
    typedef  bidirectional_circulator_tag iterator_category;
};


template <>
struct CirculatorTagTraits<std::random_access_iterator_tag> {
    typedef  random_access_circulator_tag iterator_category;
};

  /////////////////////////////////////////////////////////////////////////////
  // template class Circulator
  /**
   * Description of template class 'Circulator' <p>
   * \brief Aim: 
   *  Provides an adapter for STL iterators that can 
   *  iterate through the underlying data structure as in a loop. 
   *  The increment (resp. decrement if at least bidirectionnal) 
   *  operator encapsulates the validity test and the assignement
   *  to the first (resp. last) iterator of a given range. 
   *  For instance, the pre-increment operator does:  
   *  @code
	      ++myCurrentIt;
        if (myCurrentIt == myEndIt) myCurrentIt = myBeginIt;
	      return *this;
   *  @endcode
   * whereas the pre-decrement operator does (if at least bidirectionnal): 
   *  @code
        if (myCurrentIt == myBeginIt) myCurrentIt = myEndIt;
        --myCurrentIt;
   *  @endcode
   *
   * @tparam TIterator any forward, bidirectionnal or random access iterator 
  */

  template <typename TIterator>
  class Circulator
  {

    // ----------------------- Types ------------------------------
	public:

      typedef TIterator					                                  Iterator;
      typedef Circulator<TIterator>                               Self;

      typedef typename CirculatorTagTraits<
           typename iterator_traits<TIterator>::iterator_category >::iterator_category 
                                                                     iterator_category;

      typedef typename iterator_traits<TIterator>::value_type        value_type;
      typedef typename iterator_traits<TIterator>::difference_type   difference_type;
      typedef typename iterator_traits<TIterator>::pointer           pointer;
      typedef typename iterator_traits<TIterator>::reference         reference;



    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     * Default-initializes member [myCurrentIt].
     */
    Circulator() : myCurrentIt() {}


    /**
     * Constructor.
     * This object can iterate through the underlying data structure 
     * as in a loop, because it knows the begin and end iterator of the range. 
     * @param i any iterator
     * @param itb begin iterator
     * @param ite end iterator
     */
    explicit
    Circulator(const Iterator& i,
               const Iterator& itb, 
               const Iterator& ite) 
     : myCurrentIt(i), myBeginIt(itb), myEndIt(ite) {}

    /**
     * Destructor.
     */
    ~Circulator() {}

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Circulator ( const Circulator & other )
    : myCurrentIt(other.myCurrentIt), myBeginIt(other.myBeginIt), myEndIt(other.myEndIt) {}

    /**
     *  Copy of circulators that adapts other iterator types (not const / const).
     * @param other the object to clone.
    */
    template<typename other_Iterator>
    Circulator ( const Circulator<other_Iterator>& other )
    : myCurrentIt(other.base()), myBeginIt(other.begin()), myEndIt(other.end()) {}

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const 
    { return (myBeginIt != myEndIt) ; }


    // ----------------------- Interface --------------------------------------
  public:

    /**
     *  @return  member [myCurrentIt], the underlying iterator.
    */
    Iterator base() const
    { return myCurrentIt; }

    /**
     *  @return  member [myBeginIt], begin iterator of the underlying range.
    */
    Iterator begin() const
    { return myBeginIt; }

    /**
     *  @return  member [myEndIt], end iterator of the underlying range.
    */
    Iterator end() const
    { return myEndIt; }

    /**
     *  @return  *myCurrentIt.
    */
    reference operator*() const { 
     ASSERT( isValid() ); 
     return *myCurrentIt; 
    }

    /**
     *  @return  pointer to myCurrentIt
    */
    pointer operator->() const { 
     ASSERT( isValid() ); 
     return myCurrentIt.operator->(); 
    }


    // ----------------------- Incrementation/Decrementation --------------------------------------
  public:

     /**
      *  Pre-increment
      */
      Self& operator++()
      {
        ASSERT( isValid() ); 
	      ++myCurrentIt;
        if (myCurrentIt == myEndIt) myCurrentIt = myBeginIt;
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
        ASSERT( isValid() ); 
        if (myCurrentIt == myBeginIt) myCurrentIt = myEndIt;
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

    // ----------------------- Equality operators --------------------------------------
  public:

    bool operator==( const Self& other) const { 
      ASSERT( (myBeginIt == other.begin())&&(myEndIt == other.end()) ); 
      return myCurrentIt == other.base(); 
    }
    bool operator!=( const Self& other) const { return !(*this == other); }

    template<typename OtherIterator>
    bool operator==( const OtherIterator& other) const { 
      ASSERT( (myBeginIt == other.begin())&&(myEndIt == other.end()) ); 
      return myCurrentIt == other.base(); 
    }
    template<typename OtherIterator>
    bool operator!=( const OtherIterator& other) const { return !(*this == other); }


    // ----------------------- Random access operators --------------------------------------
  public:

    Self& operator+=( difference_type d ) {
        ASSERT( isValid() ); 
        typename Iterator::difference_type j = myCurrentIt - myBeginIt;
        typename Iterator::difference_type n = myEndIt - myBeginIt;
        ASSERT( n > 0 );
        ASSERT( (j >= 0) && (j < n) );
        typename Iterator::difference_type e = n - j;        
        if (d < e) j += d;
        else j = d - e; 
        ASSERT( (j >= 0) && (j < n) );
        myCurrentIt = myBeginIt + j;
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

    difference_type operator-( const Self& c) const {
        ASSERT( isValid() );
        ASSERT( c.isValid() );
        return myCurrentIt - c.myCurrentIt;
    }
    reference operator[]( difference_type d) const {
        Self tmp = *this;
        tmp += d;
        return *tmp;
    }

    // ----------------------- Comparisons operators --------------------------------------
    // Contrary to iterators, random access circulators have no comparison operators. 


    // ------------------------- Protected Datas --------------------------------
  protected:

    Iterator myCurrentIt; 
    Iterator myBeginIt; 
    Iterator myEndIt; 

    // ------------------------- Private Datas --------------------------------
  private:



    // ------------------------- Hidden services ------------------------------
  protected:



  private:


  }; // end of class Circulator


  /////////////////////////////////////////////////////////////////////////////
  // template functions isNotEmpty

namespace detail {

  template< typename IC > 
  inline
  bool isNotEmpty( const IC& itb, const IC& ite, IteratorType ) {
    return itb != ite;
  }

  template< typename IC > 
  inline
  bool isNotEmpty( const IC& c1, const IC& c2, CirculatorType) {
    return ( ( c1.isValid() ) && ( c2.isValid() ) );
  }

} 


template< typename IC> 
inline
bool isNotEmpty( const IC& itb, const IC& ite ){
  return detail::isNotEmpty<IC>( itb, ite, typename IteratorCirculatorTraits<IC>::Type() );
}


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/geometry/2d/Circulator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Circulator_h

#undef Circulator_RECURSES
#endif // else defined(Circulator_RECURSES)
