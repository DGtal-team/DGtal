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
 * @file ConstRangeAdapter.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/06
 *
 *
 * Header file for module ConstRangeAdapter.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(ConstRangeAdapter_RECURSES)
#error Recursive header files inclusion detected in ConstRangeAdapter.h
#else // defined(ConstRangeAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstRangeAdapter_RECURSES

#if !defined ConstRangeAdapter_h
/** Prevents repeated inclusion of headers. */
#define ConstRangeAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/ConstIteratorAdapter.h"
#include "boost/concept_check.hpp"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{


  ///////////////////////////////////////////////////////////////////////////////
  // class ConstRangeAdapter
  ///////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Aim: model of CConstBidirectionalRange that adapts any range of elements
   * bounded by two iterators [itb, ite) and provides services to 
   * (circularly)iterate over it (in a read-only manner). 
   *
   * @tparam TIterator the type of the iterator to adapt
   * (at least bidirectional) 
   *
   * Moreover, the provided (circular)iterator is adapted 
   * with a functor f given at construction so that 
   * operator* calls f(*it), instead of calling directly 
   * operator* of the underlying iterator it.
   *
   * @tparam TFunctor the type of functor that transforms
   * the pointed element into another one
   *
   * @tparam TReturnType the type of the element returned by the underlying functor. 
   *
   * NB: the underlying functor is stored in the range as aliasing pointer
   * in order to avoid copies. As a consequence the pointed object must exist 
   * and must not be deleted during the use of the range.
   *
   * @see ConstIteratorAdapter BasicFunctors.h BasicPointFunctors.h SCellsFunctors.h
   */
  template <typename TIterator, typename TFunctor, typename TReturnType>
  class ConstRangeAdapter
  {

    BOOST_CONCEPT_ASSERT(( boost::BidirectionalIterator<TIterator> )); 

    // ------------------------- inner types --------------------------------
  public: 
  
    typedef ConstIteratorAdapter<TIterator,TFunctor,TReturnType> ConstIterator; 
    typedef std::reverse_iterator<ConstIterator> ConstReverseIterator;

    typedef Circulator<ConstIterator> ConstCirculator;
    typedef std::reverse_iterator<ConstCirculator> ConstReverseCirculator;

    // ------------------------- standard services --------------------------------

    /**
     * Standard constructor from two iterators
     * and one functor.
     * @param itb begin iterator.
     * @param ite end iterator.
     * @param aFunctor functor used to adapt on-the-fly the elements of the range
     *
     */
    ConstRangeAdapter(const TIterator& itb, const TIterator& ite, 
		      const TFunctor& aFunctor )
      : myBegin(itb), myEnd(ite), myFunctor(&aFunctor) {}

    /**
     * Copy constructor.
     * @param other the iterator to clone.
     */
    ConstRangeAdapter( const ConstRangeAdapter & other )
      : myBegin(other.myBegin), myEnd(other.myEnd), myFunctor(other.myFunctor) {}
  
    /**
     * Assignment.
     * @param other the iterator to copy.
     * @return a reference on 'this'.
     */
    ConstRangeAdapter& operator= ( const ConstRangeAdapter & other )
    {  
      if ( this != &other )
	{
	  myBegin = other.myBegin;
	  myEnd = other.myEnd;
	  myFunctor = other.myFunctor;
	}
      return *this;
    }

    /**
     * Destructor. Does nothing.
     */
    ~ConstRangeAdapter() {}

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const { return true; }
  
    // ------------------------- display --------------------------------
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const 
    {
      typedef typename IteratorCirculatorTraits<ConstIterator>::Value Value; 
      out << "[ConstRangeAdapter]" << std::endl; 
      out << "\t"; 
      std::copy( myBegin, myEnd, ostream_iterator<Value>(out, ", ") ); 
      out << std::endl; 
    }

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const
    {
      return "ConstRangeAdapter";
    }
  

    // ------------------------- private data --------------------------------
  private: 
    /**
     * Begin underlying iterator
     */
    TIterator myBegin; 
    /**
     * End underlying iterator
     */
    TIterator myEnd; 
    /**
     * Aliasing pointer on the underlying functor
     */
    const TFunctor* myFunctor; 

    // ------------------------- iterator services --------------------------------
  public:

    /**
     * Iterator service.
     * @return begin iterator
     */
    ConstIterator begin() const {
      return ConstIterator( myBegin, *myFunctor );
    }

    /**
     * Iterator service.
     * @return end iterator
     */
    ConstIterator end() const {
      return ConstIterator( myEnd, *myFunctor );
    }

    /**
     * Iterator service.
     * @return rbegin iterator
     */
    ConstReverseIterator rbegin() const {
      return ConstReverseIterator(this->end());
    }

    /**
     * Iterator service.
     * @return rend iterator
     */
    ConstReverseIterator rend() const {
      return ConstReverseIterator(this->begin());
    }

    /**
     * Circulator service.
     * @return a circulator
     */
    ConstCirculator c() const {
      return ConstCirculator( this->begin(), this->begin(), this->end() );
    }

    /**
     * Circulator service.
     * @return a reverse circulator
     */
    ConstReverseCirculator rc() const {
      return ConstReverseCirculator( this->c() );
    }

  }; //end class ConstRangeAdapter

} // namespace DGtal

  ///////////////////////////////////////////////////////////////////////////////


#endif // !defined ConstRangeAdapter_h

#undef ConstRangeAdapter_RECURSES
#endif // else defined(ConstRangeAdapter_RECURSES)
