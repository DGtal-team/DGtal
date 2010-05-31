#pragma once

/**
 * @file HyperRectDomain1DIterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomain1DIterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain1DIterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain1DIterator.h
#else // defined(HyperRectDomain1DIterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain1DIterator_RECURSES

#if !defined HyperRectDomain1DIterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain1DIterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain1DIterator
/**
 * Description of class 'HyperRectDomain1DIterator' <p>
 * Aim:
 */
template<typename TPointType>
class HyperRectDomain1DIterator
{

public:

    typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
    typedef TPointType value_type;
    typedef ptrdiff_t difference_type;
    typedef TPointType* pointer;
    typedef TPointType& reference;


    HyperRectDomain1DIterator ( const TPointType & p , const std::size_t aDim )
            : myPoint ( p ),  myDimension ( aDim )
    {
    }

    const TPointType & operator*() const
    {
        return myPoint;
    }

    /**
    * Operator ==
    *
    */
    bool operator== ( const HyperRectDomain1DIterator<TPointType> &it ) const
    {
        return ( myPoint.at ( myDimension ) == ( *it ).at ( myDimension ) );
    }

    /**
    * Operator !=
    *
    */
    bool operator!= ( const HyperRectDomain1DIterator<TPointType> &aIt ) const
    {
        return ( myPoint.at ( myDimension ) != ( *aIt ).at ( myDimension ) );
    }

    /**
    * Implements the next() method to scan the domain points dimension by dimension
    * (lexicographic order).
    *
    **/
    void next()
    {
        myPoint.at ( myDimension ) ++;
        ///\todo check boundaries?
    }


    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomain1DIterator<TPointType> &operator++()
    {
        this->next();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain1DIterator<TPointType> &operator++ ( int )
    {
        HyperRectDomain1DIterator<TPointType> tmp = *this;
        ++*this;
        return tmp;
    }


    /**
    * Implements the prev() method to scan the domain points at dimension k.
    *
    **/
    void prev()
    {
        myPoint.at ( myDimension ) --;
        ///\todo check boundaries?
    }

    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomain1DIterator<TPointType> &operator--()
    {
        this->prev();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain1DIterator<TPointType> &operator-- ( int )
    {
        HyperRectDomain1DIterator<TPointType> tmp = *this;
        --*this;
        return tmp;
    }

private:
    ///Current Point in the domain
    TPointType myPoint;
    ///Dimension on which the iterator must iterate
    std::size_t myDimension;

};

} //namespace
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain1DIterator_h

#undef HyperRectDomain1DIterator_RECURSES
#endif // else defined(HyperRectDomain1DIterator_RECURSES)
