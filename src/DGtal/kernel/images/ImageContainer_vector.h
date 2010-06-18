#pragma once

/**
 * @file ImageContainer_vector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainer_vector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainer_vector_RECURSES)
#error Recursive header files inclusion detected in ImageContainer_vector.h
#else // defined(ImageContainer_vector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainer_vector_RECURSES

#if !defined ImageContainer_vector_h
/** Prevents repeated inclusion of headers. */
#define ImageContainer_vector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ImageContainer_vector
/**
 * Description of class 'ImageContainer_vector' <p>
 * \todo Documentation
 * Aim:
 */

template <class TPoint, typename TValue>
class ImageContainer_vector: public vector<TValue>
{
public:

    typedef typename vector<TValue>::size_type TSizeType;
    typedef typename vector<TValue>::iterator Iterator;
    typedef typename vector<TValue>::const_iterator ConstIterator;

    ImageContainer_vector(const TPoint &aPointA,
                          const TPoint &aPointB );

    ~ImageContainer_vector();


    TValue operator()(const TPoint &aPoint);

    TValue operator()(const Iterator &it) {
        return (*it);
    };


    void allocate(const std::size_t aSize) {
        this->resize( aSize );
    }

    /////////////////////////// Custom Iterators ////////////////////:
    class SpanIterator
    {

    public:

        typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
        typedef TValue value_type;
        typedef ptrdiff_t difference_type;
        typedef TValue* pointer;
        typedef TValue& reference;

        SpanIterator( const TPoint & p ,
                      const std::size_t aDim ,
                      ImageContainer_vector<TPoint,TValue> &aMap )
                :   myDimension ( aDim ), myMap ( aMap )
        {
            myPos = linearized(p);

            //We compute the shift quantity
            shift = 1;
            for (unsigned int k=0; k < myDimension  ; k++)
                shift *= myUpperBound.at(k) - myLowerBound.at(k);
        }

        const TValue & operator*() const
        {
            return myMap[ myPos ];
        }

        /**
        * Operator ==
        *
        */
        bool operator== ( const SpanIterator &it ) const
        {
            return ( myPos == it.pos() );
        }


        TSizeType pos()
        {
            return myPos;
        }


        /**
        * Operator !=
        *
        */
        bool operator!= ( const SpanIterator &it ) const
        {
            return ( myPos != it.pos() );

        }

        /**
        * Implements the next() method
        		*
        **/
        void next()
        {
            myPos += shift;
        }

        /**
         * Implements the prev() method
         *
         **/
        void prev()
        {
            ASSERT((long int) myPos-shift>0);
            myPos -= shift;
        }

        /**
        * Operator ++ (++it)
        *
        */
        SpanIterator &operator++()
        {
            this->next();
            return *this;
        }

        /**
        * Operator ++ (it++)
        *
        */
        SpanIterator &operator++ ( int )
        {
            SpanIterator tmp = *this;
            ++*this;
            return tmp;
        }


        /**
        * Operator -- (--it)
        *
        */
        SpanIterator &operator--()
        {
            this->prev();
            return *this;
        }

        /**
        * Operator -- (it--)
        *
        */
        SpanIterator &operator-- ( int )
        {
            SpanIterator tmp = *this;
            --*this;
            return tmp;
        }

    private:
        ///Current Point in the domain
        TSizeType myPos;

        /// Copy of the underlying images
        ImageContainer_vector<TPoint,TValue> &myMap;

        ///Dimension on which the iterator must iterate
        std::size_t myDimension;

        ///Padding variable
        TSizeType shift;

    };



    SpanIterator span_begin(const TPoint &aPoint, const std::size_t aDimension)
    {
        return SpanIterator ( aPoint, aDimension, this);
    }

    SpanIterator span_end(const std::size_t aDimension)
    {
			return SpanIterator( this->myUpperBound, aDimension, this);
    }

    TValue operator()(const SpanIterator &it) {
        return (*it);
		};


private:

    /**
     *  Linearized a point and return the vector position.
     * \param aPoint the point to convert to an index
     * \return the index of \param aPoint in the container
     */
    TSizeType linearized(const TPoint &aPoint) const;

    TPoint myLowerBound;
    TPoint myUpperBound;
};

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/images/ImageContainer_vector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainer_vector_h

#undef ImageContainer_vector_RECURSES
#endif // else defined(ImageContainer_vector_RECURSES)
