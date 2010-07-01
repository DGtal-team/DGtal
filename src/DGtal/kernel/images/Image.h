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
#include "DGtal/kernel/images/ImageContainerConcept.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class Image
  /**
   * Description of class 'Image' <p>
   * Aim:
   *
   * \todo spaniterator dans le container_map
   * \todo Documentation
   */

  template <typename THyperRectDomain, typename TValue, typename TContainer >
  class Image
  {
    // ----------------------- Standard services ------------------------------
  public:


    typedef typename THyperRectDomain::TPoint TPoint;
    typedef typename TContainer::Iterator Iterator;
    typedef typename TContainer::ConstIterator ConstIterator;
    typedef typename TContainer::SpanIterator SpanIterator;

    BOOST_CONCEPT_ASSERT((DGtal::ImageContainerConcept<TPoint,TValue,TContainer>));


    /**
     * Constuctor as the bounding box of two points.
     *
     * @param aPointA first point.
     * @param aPointB second point.
     */
    Image( const typename THyperRectDomain::TPoint &aPointA,
           const typename THyperRectDomain::TPoint &aPointB );

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
    TValue operator()(const TPoint &aPoint);


    /**
     *  Generic function to get the value of an image at position given
     * by a build-in iterator.
     * \param aIt the iteror
     * \return the value referenced by the iterator
     */
    TValue operator()(const Iterator &aIt);


    /**
     *  Generic function to get the value of an image at a point \param aPoint
     *
     */
    TValue operator()(const SpanIterator &aIt);


    /** 
     * Set a value in a Image position.
     * 
     * @param aPoint the position as a Point
     * @param aVal the value to store
     */    void setValue(const TPoint &aPoint, const TValue aVal)
    {
      myImageMap.setValue(aPoint,aVal);
    }

     /** 
     * Set a value in a Image position given by a built-in iterator
     * 
     * @param aIt the built-in iterator 
     * @param aVal the value to store
     */
    void setValue(Iterator &aIt, const TValue aVal)
    {
      myImageMap.setValue(aIt,aVal);
      
    }

    /** 
     * Set a value in a Image position given by a span iterator
     * 
     * @param aIt the span iterator 
     * @param aVal the value to store
     */
    void setValue(SpanIterator &aIt, const TValue aVal)
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
    SpanIterator span_begin(const TPoint &aPoint, const std::size_t aDimension) {
      return myImageMap.span_begin(aPoint,aDimension);

    }

    /** 
     * ImageContainer span "end" iterator  starting at aPoint along  dimension.
     *
     * \param aPoint starting point
     * \param aDimension direction of the span iterator
     * @return the  end  span iterator
     */
    SpanIterator span_end(const TPoint &aPoint, const std::size_t aDimension) {
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

    THyperRectDomain myDomain; ///Local copie of the HyperRectDomain (to have generic iterators) \todo should be removed in specialized classes
    TContainer myImageMap; ///Image Container

    TValue myValue;
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
  template <class THyperRectDomain, typename T, class TCont>
  inline
  std::ostream&
  operator<< ( std::ostream & out, const Image<THyperRectDomain,T,TCont> & object );




///////////////////////////////////////////////////////////////////////////////
// Inline methods.
#include "DGtal/kernel/images/Image.ih"


} // namespace DGtal
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Image_h

#undef Image_RECURSES
#endif // else defined(Image_RECURSES)
