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
//LICENSE-END
#pragma once

/**
 * @file HyperRectDomain_SpanIterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomain_SpanIterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_SpanIterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain_SpanIterator.h
#else // defined(HyperRectDomain_SpanIterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_SpanIterator_RECURSES

#if !defined HyperRectDomain_SpanIterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_SpanIterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain_SpanIterator
/**
 * Description of class 'HyperRectDomain_SpanIterator' <p>
 * Aim:
 */
template<typename TPoint>
class HyperRectDomain_SpanIterator
{

public:

    typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
    typedef TPoint value_type;
    typedef ptrdiff_t difference_type;
    typedef TPoint* pointer;
    typedef TPoint& reference;


    HyperRectDomain_SpanIterator ( const TPoint & p , const std::size_t aDim )
            : myPoint ( p ),  myDimension ( aDim )
    {
    }

    const TPoint & operator*() const
    {
        return myPoint;
    }

    /**
    * Operator ==
    *
    */
    bool operator== ( const HyperRectDomain_SpanIterator<TPoint> &it ) const
    {
        return ( myPoint.at ( myDimension ) == ( *it ).at ( myDimension ) );
    }

    /**
    * Operator !=
    *
    */
    bool operator!= ( const HyperRectDomain_SpanIterator<TPoint> &aIt ) const
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
    HyperRectDomain_SpanIterator<TPoint> &operator++()
    {
        this->next();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain_SpanIterator<TPoint> &operator++ ( int )
    {
        HyperRectDomain_SpanIterator<TPoint> tmp = *this;
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
    HyperRectDomain_SpanIterator<TPoint> &operator--()
    {
        this->prev();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomain_SpanIterator<TPoint> &operator-- ( int )
    {
        HyperRectDomain_SpanIterator<TPoint> tmp = *this;
        --*this;
        return tmp;
    }

private:
    ///Current Point in the domain
    TPoint myPoint;
    ///Dimension on which the iterator must iterate
    std::size_t myDimension;

};

} //namespace
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_SpanIterator_h

#undef HyperRectDomain_SpanIterator_RECURSES
#endif // else defined(HyperRectDomain_SpanIterator_RECURSES)
