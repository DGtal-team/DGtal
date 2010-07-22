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
   *
   * Aim: Model of CImageContainer implementing the association Point<->Value
   * using a std::vector. This class provides built-in iterators and fast SpanIterators 
   * to perform 1D scans.
   *
   * @example testImage.cpp
   * @example testImageContainerBenchmark.cpp
   */

  template <typename Domain, typename ValueType>
  class ImageContainerBySTLVector: public vector<ValueType>
  {
  public:

    typedef typename Domain::Point Point;
    typedef typename vector<ValueType>::size_type SizeType;
    typedef typename vector<ValueType>::iterator Iterator;
    typedef typename vector<ValueType>::const_iterator ConstIterator;

    ImageContainerBySTLVector(const Point &aPointA,
			      const Point &aPointB );

    ~ImageContainerBySTLVector();

    /** 
     * Get the value of an image at a given position.
     *
     * @param aPoint  position in the image.
     * @return the value at aPoint.
     */
    ValueType operator()(const Point &aPoint) const;

    /** 
     * Get the value of an image at a given position given 
     * by a ConstIterator.
     *
     * @param it  position in the image.
     * @return the value at aPoint.
     */
    ValueType operator()(ConstIterator &it) const
    {
      return (*it);
    };

 
    /** 
     * Set a value on an Image at aPoint.
     * 
     * @param aPoint location of the point to associate with aValue.
     * @param aValue the value.
     */   
    void setValue(const Point &aPoint, const ValueType &aValue);

    /** 
     * Set a value on an Image at a position specified by an Iterator.
     * 
     * @param it  iterator on the location.
     * @param aValue the value.
     */   
    void setValue(Iterator &it, const ValueType &aValue)
    {
      (*it) = aValue;
    }

    /////////////////////////// Custom Iterators ////////////////////:
    /** 
     * Specific SpanIterator on ImageContainerBySTLVector.
     * 
     * @tparam Domain the HyperRectDomain on which the iterator iterates.
     * @tparam ValueType 
     */
    class SpanIterator
    {

      friend class ImageContainerBySTLVector<Domain,ValueType>;

    public:

      typedef std::bidirectional_iterator_tag iterator_category; ///\todo construct a RANDOM-ACCESS iterator
      typedef ValueType value_type;
      typedef ptrdiff_t difference_type;
      typedef ValueType* pointer;
      typedef ValueType& reference;

      /** 
       * Constructor.
       * 
       * @param p starting point of the SpanIterator
       * @param aDim specifies the dimension along which the iterator will iterate
       * @param aMap pointer to the imageContainer
       */    SpanIterator( const Point & p ,
			   const std::size_t aDim ,
			   ImageContainerBySTLVector<Domain,ValueType> *aMap ) :  myMap ( aMap ), myDimension ( aDim )
      {
	myPos = aMap->linearized(p);

	//We compute the myShift quantity
	myShift = 1;
	for (unsigned int k=0; k < myDimension  ; k++)
	  myShift *= (aMap->myUpperBound.at(k) - aMap->myLowerBound.at(k));
      }


      /** 
       * operator* on SpanIterators.
       * 
       * @return the value associated to the current position.
       */      
      const ValueType & operator*() const
      {
	return (*myMap)[ myPos ];
      }

      /**
       * Operator ==.
       *
       * @return true if this and it are equals.
       */
      bool operator== ( const SpanIterator &it ) const
      {
	return ( myPos == it.myPos );
      }

      /**
       * Operator !=
       *
       * @return true if this and it are different.
       */
      bool operator!= ( const SpanIterator &it ) const
      {
	return ( myPos != it.myPos );

      }

      /**
       * Implements the next() method: we move on step forward.
       *
       **/
      void next()
      {
	myPos += myShift;
      }

      /**
       * Implements the prev() method: we move on step backward.
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
      ImageContainerBySTLVector<Domain,ValueType> *myMap;

      ///Dimension on which the iterator must iterate
      std::size_t myDimension;

      ///Padding variable
      SizeType myShift;

    };

    /** 
     * Set a value on an Image at a position specified by an SpanIterator.
     * 
     * @param it  iterator on the location.
     * @param aValue the value.
     */     
    void setValue(SpanIterator &it, const ValueType &aValue)
    {
      it.setValue(aValue);
    }


    /** 
     * Create a begin() SpanIterator at a given position in a given 
     * direction.
     * 
     * @param aPoint the starting point of the SpanIterator.
     * @param aDimension the dimension on which the iterator iterates.
     * 
     * @return a SpanIterator
     */
    SpanIterator span_begin(const Point &aPoint, const std::size_t aDimension)
    {
      return SpanIterator ( aPoint, aDimension, this);
    }

    /** 
     * Create an end() SpanIterator at a given position in a given 
     * direction.
     * 
     * @param aPoint a point belonging to the current image dimension (not 
     * necessarily the point used in the span_begin() method.
     * @param aDimension the dimension on which the iterator iterates.
     * 
     * @return a SpanIterator
     */
    SpanIterator span_end(const Point &aPoint,const std::size_t aDimension)
    {
      Point tmp = aPoint;
      tmp.at( aDimension ) = myLowerBound.at( aDimension ) +
	myUpperBound.at( aDimension ) - 
	myLowerBound.at( aDimension ) + 1;
      return SpanIterator( tmp, aDimension, this);
    }

    /** 
     * Returns the value of the image at a given SpanIterator position.
     *
     * @param it position given by a SpanIterator.
     * @return an object of type ValueType.
     */
    ValueType operator()(const SpanIterator &it)
    {
      return (*it);
    };


    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * @return the validity of the Image
     */
    bool isValid() const
    {
      return (this != NULL);
    }


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

  /**
   * Overloads 'operator<<' for displaying objects of class 'Image'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Image' to write.
   * @return the output stream after the writing.
   */
  template <typename Domain, typename V>
  inline
  std::ostream&
  operator<< ( std::ostream & out, const ImageContainerBySTLVector<Domain,V> & object )
  {
    object.selfDisplay ( out );
    return out;
  }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/images/ImageContainerBySTLVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerBySTLVector_h

#undef ImageContainerBySTLVector_RECURSES
#endif // else defined(ImageContainerBySTLVector_RECURSES)
