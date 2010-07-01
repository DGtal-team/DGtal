#pragma once

/**
 * @file Image.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/09
 *
 * Header file for module Image.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Image_RECURSES)
#error Recursive header files inclusion detected in Image.h
#else // defined(Image_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Image_RECURSES

#if !defined Image_h
/** Prevents repeated inclusion of headers. */
#define Image_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/CImageContainer.h"
#include "DGtal/kernel/images/CValueType.h"
#include "DGtal/kernel/images/ImageContainerBySTLVector.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class Image
  /**
   * Description of class 'Image' <p>
   * \brief Aim: implementation of a generic image.
   *
   *
   * \todo spaniterator dans le container_map
   * \todo Documentation
   * @example test_Image.cpp
   */

  template <typename Domain, typename ValueType, 
	    typename ImageContainer =  ImageContainerBySTLVector<typename Domain::Point, ValueType > >
  class Image
  {
    // ----------------------- Standard services ------------------------------
  public:


    typedef typename Domain::Point Point;
    typedef typename ImageContainer::Iterator Iterator;
    typedef typename ImageContainer::ConstIterator ConstIterator;
    typedef typename ImageContainer::SpanIterator SpanIterator;

    BOOST_CONCEPT_ASSERT((CImageContainer<Point,ValueType,ImageContainer>));

    /**
     * Constuctor as the bounding box of two points.
     *
     * @param aPointA first point.
     * @param aPointB second point.
     */
    Image( const typename Domain::Point &aPointA,
           const typename Domain::Point &aPointB );

    /**
     * Destructor.x
     */
    ~Image();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     *  Generic function to get the value of an image at a point.
     *
     * \param aPoint the point
     * \return the value at the point \param aPoint
     */
    ValueType operator()(const Point &aPoint);


    /**
     *  Generic function to get the value of an image at position given
     * by a build-in iterator.
     * \param aIt the iteror
     * \return the value referenced by the iterator
     */
    ValueType operator()(const Iterator &aIt);


    /**
     *  Generic function to get the value of an image at a point \param aPoint
     *
     */
    ValueType operator()(const SpanIterator &aIt);


    /** 
     * Set a value in a Image position.
     * 
     * @param aPoint the position as a Point
     * @param aVal the value to store
     */    
    void setValue(const Point &aPoint, const ValueType aVal)
    {
      myImageMap.setValue(aPoint,aVal);
    }

     /** 
     * Set a value in a Image position given by a built-in iterator
     * 
     * @param aIt the built-in iterator 
     * @param aVal the value to store
     */
    void setValue(Iterator &aIt, const ValueType aVal)
    {
      myImageMap.setValue(aIt,aVal);
      
    }

    /** 
     * Set a value in a Image position given by a span iterator
     * 
     * @param aIt the span iterator 
     * @param aVal the value to store
     */
    void setValue(SpanIterator &aIt, const ValueType aVal)
    {
      myImageMap.setValue(aIt,aVal);
    }


    // ----------------------- Built-in terators  from the container--------------------------------

    /** 
     * ImageContainer built-in begin() iterator
     * @return the built-in begin() iterator
     */
    Iterator begin() {
      return myImageMap.begin();
    }

    /** 
     * ImageContainer built-in end()  iterator
     * @return the built-in end()  iterator
     */
    Iterator end() {
      return myImageMap.end();
    }

    /** 
     * ImageContainer span iterator  starting at aPoint along  dimension.
     *
     * \param aPoint starting point
     * \param aDimension direction of the span iterator
     * @return the begin span iterator
     */
    SpanIterator span_begin(const Point &aPoint, const std::size_t aDimension) {
      return myImageMap.span_begin(aPoint,aDimension);

    }

    /** 
     * ImageContainer span "end" iterator  starting at aPoint along  dimension.
     *
     * \param aPoint starting point
     * \param aDimension direction of the span iterator
     * @return the  end  span iterator
     */
    SpanIterator span_end(const Point &aPoint, const std::size_t aDimension) {
      return myImageMap.span_end(aPoint,aDimension);
    }
    /** 
     * ImageContainer built-in rend()  iterator
     * @return the built-in rend()  iterator
     */
    Iterator rend() {
      return myImageMap.rend();
    }

    /** 
     * ImageContainer built-in rbegin() iterator
     * @return the built-in rbegin() iterator
     */
    Iterator rbegin() {
      return myImageMap.rend();
    }

    /** 
     * ImageContainer built-in begin() const iterator
     * @return the built-in begin() const iterator
     */
    ConstIterator begin() const {
      return myImageMap.begin();
    }

    /** 
     * ImageContainer built-in end() const iterator
     * @return the built-in end() const iterator
     */
    ConstIterator end() const {
      return myImageMap.end();
    }
    /** 
     * ImageContainer built-in rend() const iterator
     * @return the built-in rend() const iterator
     */
    ConstIterator rend() const {
      return myImageMap.rend();
    }

    /** 
     * ImageContainer built-in rbegin() const iterator
     * @return the built-in rbegin() const iterator
     */
    ConstIterator rbegin() const {
      return myImageMap.rend();
    }

    //-----------------------------------------------------------------------
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

  protected:

    ImageContainer myImageMap; ///Image Container
    ValueType myValue;

  private:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Image & operator= ( const Image & other );

    // ------------------------- Internals ------------------------------------
  private:



  }; // end of class Image


  /**
   * Overloads 'operator<<' for displaying objects of class 'Image'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Image' to write.
   * @return the output stream after the writing.
   */
  template <typename Domain, typename T, typename TCont>
  inline
  std::ostream&
  operator<< ( std::ostream & out, const Image<Domain,T,TCont> & object );




///////////////////////////////////////////////////////////////////////////////
// Inline methods.
#include "DGtal/kernel/images/Image.ih"


} // namespace DGtal
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Image_h

#undef Image_RECURSES
#endif // else defined(Image_RECURSES)
