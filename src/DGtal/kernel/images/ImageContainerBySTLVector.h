#pragma once

/**
 * @file ImageContainerBySTLVector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainerBySTLVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerBySTLVector_RECURSES)
#error Recursive header files inclusion detected in ImageContainerBySTLVector.h
#else // defined(ImageContainerBySTLVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerBySTLVector_RECURSES

#if !defined ImageContainerBySTLVector_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerBySTLVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class ImageContainerBySTLVector
  /**
   * Description of class 'ImageContainerBySTLVector' <p>
   * \todo Documentation
   * Aim:
   */

  template <typename Point, typename ValueType>
    class ImageContainerBySTLVector: public vector<ValueType>
  {
  public:

    typedef typename vector<ValueType>::size_type SizeType;
    typedef typename vector<ValueType>::iterator Iterator;
    typedef typename vector<ValueType>::const_iterator ConstIterator;

    ImageContainerBySTLVector(const Point &aPointA,
			      const Point &aPointB );

    ~ImageContainerBySTLVector();


    ValueType operator()(const Point &aPoint);

    ValueType operator()(const Iterator &it)
    {
      return (*it);
    };

    void setValue(const Point &aPoint, const ValueType &aValue);



    void setValue(Iterator &it, const ValueType &aValue)
    {
      (*it) = aValue;
    }

    void allocate(const std::size_t aSize) {
      this->resize( aSize );
    }

    /////////////////////////// Custom Iterators ////////////////////:
    class SpanIterator
    {

      friend class ImageContainerBySTLVector<Point,ValueType>;

    public:

      typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
      typedef ValueType value_type;
      typedef ptrdiff_t difference_type;
      typedef ValueType* pointer;
      typedef ValueType& reference;

    SpanIterator( const Point & p ,
		  const std::size_t aDim ,
		  ImageContainerBySTLVector<Point,ValueType> *aMap ) :   myDimension ( aDim ), myMap ( aMap )
      {
	myPos = aMap->linearized(p);

	//We compute the myShift quantity
	myShift = 1;
	for (unsigned int k=0; k < myDimension  ; k++)
	  myShift *= (aMap->myUpperBound.at(k) - aMap->myLowerBound.at(k));
      }

      const ValueType & operator*() const
      {
	return (*myMap)[ myPos ];
      }

      void setValue(const ValueType &v)
      {
	(*myMap)[ myPos ] = v;
      }

      /**
       * Operator ==
       *
       */
      bool operator== ( const SpanIterator &it ) const
      {
	return ( myPos == it.myPos );
      }

      /**
       * Operator !=
       *
       */
      bool operator!= ( const SpanIterator &it ) const
      {
	return ( myPos != it.myPos );

      }

      /**
       * Implements the next() method
       *
       **/
      void next()
      {
	myPos += myShift;
      }

      /**
       * Implements the prev() method
       *
       **/
      void prev()
      {
	ASSERT((long int) myPos-myShift>0);
	myPos -= myShift;
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
      SizeType myPos;

      /// Copy of the underlying images
      ImageContainerBySTLVector<Point,ValueType> *myMap;

      ///Dimension on which the iterator must iterate
      std::size_t myDimension;

      ///Padding variable
      SizeType myShift;

    };


    void setValue(SpanIterator &it, const ValueType &aValue)
    {
      it.setValue(aValue);
    }

    SpanIterator span_begin(const Point &aPoint, const std::size_t aDimension)
    {
      return SpanIterator ( aPoint, aDimension, this);
    }

    SpanIterator span_end(const Point &aPoint,const std::size_t aDimension)
    {
      Point tmp = aPoint;
      tmp.at( aDimension ) = myLowerBound.at( aDimension ) +
	myUpperBound.at( aDimension ) - 
	myLowerBound.at( aDimension ) + 1;
      return SpanIterator( tmp, aDimension, this);
    }

    ValueType operator()(const SpanIterator &it)
    {
      return (*it);
    };


  private:

    /**
     *  Linearized a point and return the vector position.
     * @param aPoint the point to convert to an index
     * @return the index of @param aPoint in the container
     */
    SizeType linearized(const Point &aPoint) const;

    Point myLowerBound;
    Point myUpperBound;
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/images/ImageContainerBySTLVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerBySTLVector_h

#undef ImageContainerBySTLVector_RECURSES
#endif // else defined(ImageContainerBySTLVector_RECURSES)
