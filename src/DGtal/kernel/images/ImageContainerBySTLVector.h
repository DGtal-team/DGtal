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
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/images/CValueType.h"
#include "DGtal/kernel/domains/CDomain.h"

#include "DGtal/io/DGtalBoard.h"

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
   * @see testImage.cpp
   * @see testImageContainerBenchmark.cpp
   */

  template <typename TDomain, typename TValueType>
  class ImageContainerBySTLVector: public vector<TValueType>
  {
  public:

    BOOST_CONCEPT_ASSERT(( CValueType<TValueType> ));
    BOOST_CONCEPT_ASSERT(( CDomain<TDomain> ));
			
    typedef TValueType ValueType;
    typedef TDomain Domain;

    // static constants
    static const typename Domain::Dimension staticDimension = Domain::staticDimension;
    
    typedef typename Domain::Point Point;
    typedef typename Domain::Vector Vector;
    typedef typename Domain::Dimension Dimension;
    typedef typename Domain::Integer Integer;
    typedef typename Domain::Size Size;
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

    /**
     * Returns the extent of an Image.
     *
     * @return the image extent as a Vector.
     */
    Vector extent() const;

    /**
     * @return the image lower point.
     */
    Point lowerBound() const
    {
      return myLowerBound;
    };

    /**
     * @return the image upper point.
     */
    Point upperBound() const
    {
      return myUpperBound;
    };

    /////////////////////////// Custom Iterators ////////////////////:
    /**
     * Specific SpanIterator on ImageContainerBySTLVector.
     *
     * @tparam Domain the HyperRectDomain on which the iterator iterates.
     * @tparam ValueType
     */
    class SpanIterator
    {

      friend class ImageContainerBySTLVector<Domain, ValueType>;

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
       */
      SpanIterator( const Point & p ,
		    const Dimension aDim ,
		    ImageContainerBySTLVector<Domain, ValueType> *aMap ) :  myMap ( aMap ), myDimension ( aDim )
      {
	myPos = aMap->linearized(p);

	//We compute the myShift quantity
	myShift = 1;
	for (unsigned int k = 0; k < myDimension  ; k++)
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
	ASSERT((long int) myPos - myShift > 0);
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
      Size myPos;

      /// Copy of the underlying images
      ImageContainerBySTLVector<Domain, ValueType> *myMap;

      ///Dimension on which the iterator must iterate
      Dimension  myDimension;

      ///Padding variable
      Size myShift;

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
    SpanIterator spanBegin(const Point &aPoint, const Dimension aDimension)
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
    SpanIterator spanEnd(const Point &aPoint, const Dimension aDimension)
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



  private:

    /**
     *  Linearized a point and return the vector position.
     * @param aPoint the point to convert to an index
     * @return the index of @param aPoint in the container
     */
    Size linearized(const Point &aPoint) const;

    Point myLowerBound;
    Point myUpperBound;

    // ------------- realization CDrawableWithDGtalBoard --------------------
  private:

    /**
     * Default style.
     */
    struct DefaultDrawStyle : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
	aBoard.setPenColorRGBi(60, 60, 60);
	aBoard.setLineStyle(DGtalBoard::Shape::SolidStyle);
      }
    };

  public:

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithDGtalBoard* defaultStyle() const;

    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;


    /**
     * Draw the object on a DGtalBoard board.
     * @param board the output board where the object is drawn.
     * @param minValue the minimum value contained in the image (used in the colormap settings)
     * @param maxValue the maximum value contained in the image (used in the colormap settings)
     * @tparam Coloramp any models of CColormap.
     */
    template<typename Colormap>
    void selfDraw(DGtalBoard & board, const ValueType & minValue, const ValueType & maxValue ) const;

    // ----------------------- Serializarion methods ------------------------------------
  private:

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      using boost::serialization::make_nvp;
      ar & make_nvp("UpperBound", myUpperBound);
      ar & make_nvp("LowerBound", myLowerBound);
      ar & make_nvp("Image", boost::serialization::base_object< std::vector< ValueType > >(*this));
    }

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
  operator<< ( std::ostream & out, const ImageContainerBySTLVector<Domain, V> & object )
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
