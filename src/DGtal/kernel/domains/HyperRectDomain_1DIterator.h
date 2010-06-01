#pragma once

/**
 * @file HyperRectDomain_1DIterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomain_1DIterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_1DIterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain_1DIterator.h
#else // defined(HyperRectDomain_1DIterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_1DIterator_RECURSES

#if !defined HyperRectDomain_1DIterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_1DIterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain_1DIterator
/**
 * Description of class 'HyperRectDomain_1DIterator' <p>
 * Aim:
 */
template<typename TPointType>
class HyperRectDomain_1DIterator
{

public:

    typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
    typedef TPointType value_type;
    typedef ptrdiff_t difference_type;
    typedef TPointType* pointer;
    typedef TPointType& reference;


    HyperRectDomain_1DIterator ( const TPointType & p , const std::size_t aDim )
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
    bool operator== ( const HyperRectDomain_1DIterator<TPointType> &it ) const
    {
        return ( myPoint.at ( myDimension ) == ( *it ).at ( myDimension ) );
    }

    /**
    * Operator !=
    *
    */
    bool operator!= ( const HyperRectDomain_1DIterator<TPointType> &aIt ) const
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
    HyperRectDomain_1DIterator<TPointType> &operator++()
    {
        this->next();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain_1DIterator<TPointType> &operator++ ( int )
    {
        HyperRectDomain_1DIterator<TPointType> tmp = *this;
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
    HyperRectDomain_1DIterator<TPointType> &operator--()
    {
        this->prev();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain_1DIterator<TPointType> &operator-- ( int )
    {
        HyperRectDomain_1DIterator<TPointType> tmp = *this;
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

#endif // !defined HyperRectDomain_1DIterator_h

#undef HyperRectDomain_1DIterator_RECURSES
#endif // else defined(HyperRectDomain_1DIterator_RECURSES)
