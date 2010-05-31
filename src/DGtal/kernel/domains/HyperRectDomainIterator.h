#pragma once

/**
 * @file HyperRectDomainIterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomainIterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomainIterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomainIterator.h
#else // defined(HyperRectDomainIterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomainIterator_RECURSES

#if !defined HyperRectDomainIterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomainIterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomainIterator
/**
 * Description of class 'HyperRectDomainIterator' <p>
 * Aim:
 */
template<typename TPointType>
class HyperRectDomainIterator
{

public:

    typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
    typedef TPointType value_type;
    typedef ptrdiff_t difference_type;
    typedef TPointType* pointer;
    typedef TPointType& reference;


    HyperRectDomainIterator ( const TPointType & p, const TPointType& lower,const TPointType &upper )
            : myPoint ( p ),  myCurrentPos ( 0 ), mylower ( lower ), myupper ( upper )
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
    bool operator== ( const HyperRectDomainIterator<TPointType> &it ) const
    {
        return ( myPoint == ( *it ) );
    }

    /**
    * Operator !=
    *
    */
    bool operator!= ( const HyperRectDomainIterator<TPointType> &aIt ) const
    {
        return ( myPoint != ( *aIt ) );
    }

    /**
    * Implements the next() method to scan the domain points dimension by dimension
    * (lexicographic order).
    *
    **/
    void next()
    {
        if ( myPoint.at ( myCurrentPos )  < myupper.at ( myCurrentPos ) )
            myPoint.at ( myCurrentPos ) ++;
        else
        {
            while ( ( myCurrentPos < myPoint.dimension() ) &&
                    ( myPoint.at ( myCurrentPos )  >=  myupper.at ( myCurrentPos ) ) )
            {
                myPoint.at ( myCurrentPos ) = mylower.at ( myCurrentPos );
                myCurrentPos++;
            }

            if ( myCurrentPos < myPoint.dimension() )
            {
                myPoint.at ( myCurrentPos ) ++;
                myCurrentPos = 0;
            }
            else
            {
                myPoint = myupper;
            }
        }
    }

    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomainIterator<TPointType> &operator++()
    {
        this->next();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomainIterator<TPointType> &operator++ ( int )
    {
        HyperRectDomainIterator<TPointType> tmp = *this;
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
        if ( myPoint.at ( myCurrentPos )  > mylower.at ( myCurrentPos ) )
            myPoint.at ( myCurrentPos ) --;
        else
        {
            while ( ( myCurrentPos >= 0 ) &&
                    ( myPoint.at ( myCurrentPos )  <=  mylower.at ( myCurrentPos ) ) )
            {
                myPoint.at ( myCurrentPos ) = myupper.at ( myCurrentPos );
                myCurrentPos++;
            }

            if ( myCurrentPos >= 0 )
            {
                myPoint.at ( myCurrentPos ) --;
                myCurrentPos = 0;
            }
            else
            {
                myPoint = mylower;
            }
        }

    }

    /**
    * Operator ++ (++it)
    *
    */
    HyperRectDomainIterator<TPointType> &operator--()
    {
        this->prev();
        return *this;
    }

    /**
    * Operator ++ (it++)
    *
    */
    HyperRectDomainIterator<TPointType> &operator-- ( int )
    {
        HyperRectDomainIterator<TPointType> tmp = *this;
        --*this;
        return tmp;
    }


private:
    ///Current Point in the domain
    TPointType myPoint;
    ///Copies of the Domain limits
    TPointType mylower, myupper;
    ///Second index of the iterator position
    std::size_t myCurrentPos;
};


} //namespace
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomainIterator_h

#undef HyperRectDomainIterator_RECURSES
#endif // else defined(HyperRectDomainIterator_RECURSES)
