#pragma once

/**
 * @file HyperRectDomain_Iterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomain_Iterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_Iterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain_Iterator.h
#else // defined(HyperRectDomain_Iterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_Iterator_RECURSES

#if !defined HyperRectDomain_Iterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_Iterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain_Iterator
/**
 * Description of class 'HyperRectDomain_Iterator' <p>
 * Aim:
 */
template<typename TPoint>
class HyperRectDomain_Iterator
{
public:
    typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
    typedef TPoint value_type;
    typedef ptrdiff_t difference_type;
    typedef TPoint* pointer;
    typedef TPoint& reference;


    HyperRectDomain_Iterator ( const TPoint & p, const TPoint& lower,const TPoint &upper )
      : myPoint ( p ), mylower ( lower ), myupper ( upper ),  myCurrentPos ( 0 ),
	myUsePermutation(false)
  { 
    ASSERT(lower<upper); 
    ASSERT(lower<=p && p<=upper); 
  }

    HyperRectDomain_Iterator ( const TPoint & p, const TPoint& lower,const TPoint &upper,
			       std::initializer_list<unsigned int> permutation )
      : myPoint ( p ),  myCurrentPos ( 0 ), mylower ( lower ), myupper ( upper ),
	myUsePermutation(true)
    {
      ASSERT(lower<upper); 
      ASSERT(lower<=p && p<=upper);
      ASSERT(permutation.size()==TPoint::Dimension);
      myPermutation.reserve(permutation.size());
      uninitialized_copy(permutation.begin(), permutation.end(), myPermutation);
      // TODO: check the validity of the permutation ?      
    }

    const TPoint & operator*() const
    {  
      ASSERT(mylower<=myPoint && myPoint<myupper); // we must be between [begin,end[
      return myPoint;
    }

    /**
    * Operator ==
    *
    */
    bool operator== ( const HyperRectDomain_Iterator<TPoint> &it ) const
    {
        return ( myPoint == ( *it ) );
    }

    /**
    * Operator !=
    *
    */
    bool operator!= ( const HyperRectDomain_Iterator<TPoint> &aIt ) const
    {
        return ( myPoint != ( *aIt ) );
    }

    /**
    * Implements the next() method to scan the domain points dimension by dimension
    * (lexicographic order).
    *
    **/
    void nextLexicographicOrder()
    {
      ASSERT(mylower<=myPoint && myPoint<myupper); // we must be between [begin,end[

      myPoint.at ( myCurrentPos ) ++;
      if ( ( myCurrentPos < myPoint.dimension()-1 ) && 
	   ( myPoint.at ( myCurrentPos )  >  myupper.at ( myCurrentPos ) ) )
	{
	  do
	    {
	      myPoint.at ( myCurrentPos ) = mylower.at ( myCurrentPos );
	      myCurrentPos++;
	      if (myCurrentPos < myPoint.dimension())
		myPoint.at ( myCurrentPos ) ++;
	    }
	  while ( ( myCurrentPos < myPoint.dimension()-1 ) &&
		  ( myPoint.at ( myCurrentPos )  >  myupper.at ( myCurrentPos ) ) );	    
	  myCurrentPos = 0;
	}
    }

    void nextPermutationOrder()
    { // TODO
    }

    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomain_Iterator<TPoint> &operator++()
    {
      if (myUsePermutation) nextPermutationOrder();
      else                  nextLexicographicOrder();
      return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain_Iterator<TPoint> &operator++ ( int )
    {
        HyperRectDomain_Iterator<TPoint> tmp = *this;
        ++*this;
        return tmp;
    }


    /**
    * Implements the prev() method to scan the domain points dimension by dimension
    * (lexicographic order).
    *
    **/
    void prev()
    {
      ASSERT(mylower<=myPoint && myPoint<myupper); // we must be between [begin,end[

      myPoint.at ( myCurrentPos ) --;
      if ( ( myCurrentPos < myPoint.dimension()-1 ) && 
	   ( myPoint.at ( myCurrentPos )  <  mylower.at ( myCurrentPos ) ) )
	{
	  do
	    {
	      myPoint.at ( myCurrentPos ) = myupper.at ( myCurrentPos );
	      myCurrentPos++;
	      if (myCurrentPos < myPoint.dimension())
		myPoint.at ( myCurrentPos ) --;
	    }
	  while ( ( myCurrentPos < myPoint.dimension()-1 ) &&
		  ( myPoint.at ( myCurrentPos )  <  mylower.at ( myCurrentPos ) ) );	    
	  myCurrentPos = 0;
	}
    }

    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomain_Iterator<TPoint> &operator--()
    {
        this->prev();
        return *this;
    }

    /**
    * Operator ++ (it++)
    */
    HyperRectDomain_Iterator<TPoint> &operator-- ( int )
    {
        HyperRectDomain_Iterator<TPoint> tmp = *this;
        --*this;
        return tmp;
    }


private:
    ///Current Point in the domain
    TPoint myPoint;
    ///Copies of the Domain limits
    TPoint mylower, myupper;
    ///Second index of the iterator position
    std::size_t myCurrentPos;
    ///Vector of permutation on dimension, to fix the order in which dimensions
    /// are considered.
    std::vector<unsigned int> myPermutation;
    /// True iff we use the vector of permutation, otherwise dimensions are
    /// considered in increasing order.
    bool myUsePermutation;
};


} //namespace
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_Iterator_h

#undef HyperRectDomain_Iterator_RECURSES
#endif // else defined(HyperRectDomain_Iterator_RECURSES)
