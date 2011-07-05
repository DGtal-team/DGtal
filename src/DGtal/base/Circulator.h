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
  // template class Circulator
  /**
   * Description of template class 'Circulator' <p>
   * \brief Aim: 
   *  Provides an adapter for Bidirectional iterators that can 
   *  iterate through the underlying data structure as in a loop. 
   *  The increment (resp. decrement) operator encapsulates the 
   *  validity test and the assignement to the first (resp. last)
   *  iterator. 
   * For instance, the pre-increment operator does:  
   *  @code
	      ++myCurrentIt;
        if (myCurrentIt == myEndIt) myCurrentIt = myBeginIt;
	      return *this;
   *  @endcode
   * whereas the pre-decrement operator does: 
   *  @code
        if (myCurrentIt == myBeginIt) myCurrentIt = myEndIt;
        --myCurrentIt;
   *  @endcode
   *
   *  Note that decrementing the end iterator must results in
   *  an iterator pointing to the last element of the 
   *  underlying data structure, as in STL bidirectionnal iterators. 
   *
  */

  template <typename BidirectionnalIterator>
  class Circulator
   : public iterator<
          typename iterator_traits<BidirectionnalIterator>::iterator_category,
		      typename iterator_traits<BidirectionnalIterator>::value_type,
		      typename iterator_traits<BidirectionnalIterator>::difference_type,
		      typename iterator_traits<BidirectionnalIterator>::pointer,
          typename iterator_traits<BidirectionnalIterator>::reference >
  {

    // ----------------------- Types ------------------------------
	public:


      typedef BidirectionnalIterator					       iterator_type;
      typedef typename iterator_traits<BidirectionnalIterator>::difference_type   difference_type;
      typedef typename iterator_traits<BidirectionnalIterator>::reference   reference;
      typedef typename iterator_traits<BidirectionnalIterator>::pointer     pointer;



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
    Circulator(const iterator_type& i,
               const iterator_type& itb, 
               const iterator_type& ite) 
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
    template<typename other_iterator_type>
    Circulator ( const Circulator<other_iterator_type>& other )
    : myCurrentIt(other.base()), myBeginIt(other.begin()), myEndIt(other.end()) {}

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const 
    { return (myCurrentIt != myEndIt) ; }


    // ----------------------- Interface --------------------------------------
  public:

    /**
     *  @return  member [myCurrentIt], the underlying iterator.
    */
    iterator_type base() const
    { return myCurrentIt; }

    /**
     *  @return  member [myBeginIt], begin iterator of the underlying range.
    */
    iterator_type begin() const
    { return myBeginIt; }

    /**
     *  @return  member [myEndIt], end iterator of the underlying range.
    */
    iterator_type end() const
    { return myEndIt; }

    /**
     *  @return  *myCurrentIt.
    */
    reference operator*() const
    { return *myCurrentIt; }

    /**
     *  @return  pointer to what is returned by the dereference operator
    */
    pointer operator->() const
    { return &(operator*()); }


    // ----------------------- Incrementation/Decrementation --------------------------------------
  public:

     /**
      *  Pre-increment
      */
      Circulator& operator++()
      {
	      ++myCurrentIt;
        if (myCurrentIt == myEndIt) myCurrentIt = myBeginIt;
	      return *this;
      }

      /**
      * Post-increment
      */
      Circulator operator++(int)
      {
	      Circulator tmp = *this;
	      operator++(); 
	      return tmp;
      }


     /**
      *  Pre-decrement
      */
      Circulator& operator--()
      {
        if (myCurrentIt == myBeginIt) myCurrentIt = myEndIt;
        --myCurrentIt;
	      return *this;
      }

      /**
      * Post-decrement
      */
      Circulator operator--(int)
      {
	      Circulator tmp = *this;
	      operator--(); 
	      return tmp;
      }

    // ----------------------- Comparisons operators --------------------------------------
  public:

  /**
   *  Comparison operators between Circulators are based
   *  on their underlying iterators returned by the base() method. 
   *  Note however that their begin and end iterators are expected
   *  to be the same for a valid comparison. 
   *  @param  other  any Circulator to compare
   *  @return  a bool
  */

    template<typename any_iterator_type>
    inline bool
    operator==(const Circulator<any_iterator_type>& other) const
    { 
      ASSERT( (myBeginIt == other.begin())&&(myEndIt == other.end()) ); 
      return myCurrentIt == other.base(); 
    }

    template<typename any_iterator_type>
    inline bool
    operator!=(const Circulator<any_iterator_type>& other) const
    { 
      return !(*this == other); 
    }

    // ------------------------- Protected Datas --------------------------------
  protected:

    iterator_type myCurrentIt; 
    iterator_type myBeginIt; 
    iterator_type myEndIt; 

    // ------------------------- Private Datas --------------------------------
  private:



    // ------------------------- Hidden services ------------------------------
  protected:



  private:





  }; // end of class Circulator


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/geometry/2d/Circulator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Circulator_h

#undef Circulator_RECURSES
#endif // else defined(Circulator_RECURSES)
