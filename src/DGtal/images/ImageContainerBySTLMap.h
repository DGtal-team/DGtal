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
 * @file ImageContainerBySTLMap.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/15
 *
 * Header file for module ImageContainerBySTLMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerBySTLMap_RECURSES)
#error Recursive header files inclusion detected in ImageContainerBySTLMap.h
#else // defined(ImageContainerBySTLMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerBySTLMap_RECURSES

#if !defined ImageContainerBySTLMap_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerBySTLMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>

#include "DGtal/base/Common.h"
#include "DGtal/images/CValue.h"
#include "DGtal/kernel/domains/CDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class ImageContainerBySTLMap
  /**
   * Description of class 'ImageContainerBySTLMap' <p>
   * Aim: Model of CImageContainer implementing the association Point<->Value
   * using a std::map.
   *
   *
   * @see testImage.cpp
   */

  template <typename Domain, typename Value>
  class ImageContainerBySTLMap: public map<typename Domain::Point,Value>
  {

  public:

    BOOST_CONCEPT_ASSERT(( CValue<Value> ));
    BOOST_CONCEPT_ASSERT(( CDomain<Domain> ));    
    
    typedef typename Domain::Point Point;
    typedef typename Domain::Dimension Dimension;
    typedef typename map<Point,Value>::size_type TSize;
    typedef typename map<Point,Value>::iterator Iterator;
    typedef typename map<Point,Value>::const_iterator ConstIterator;
    typedef typename map<Point,Value>::reverse_iterator ReverseIterator;
    typedef typename map<Point,Value>::const_reverse_iterator
    ConstReverseIterator;


    /**
     * @return the domain associated to the image.
     */
    Domain domain() const
    {
      return Domain(myLowerBound, myUpperBound);
    }
    

    ///\todo create span iterators
    class SpanIterator: public Iterator
    {
      friend class ImageContainerBySTLMap<Domain,Value>;

    public:
      SpanIterator( const Point & p ,
		    const Dimension aDim ,
		    ImageContainerBySTLMap<Domain,Value> *aMap ) :   
	myStartingPoint( p ),  myDimension ( aDim ),   myMap ( aMap )
      {
	myPos = myMap->find( p );
      }


      const Value & operator*() const
      {
	return (*myPos).second;
      }
      
      /**
       * Implements the next() method
       *
       **/
      void next()
      {
	while ((myPos != myMap->end()) && 
	       ( (*myPos).first.at(myDimension) != myStartingPoint.at(myDimension)))
	  {
	    myPos++;
	  }
      }

      /**
       * Implements the prev() method
       *
       **/
      void prev()
      {
	while ((myPos != myMap->end()) && 
	       ( (*myPos).first.at(myDimension) != myStartingPoint.at(myDimension)))
	  {
	    myPos--;
	  }
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
      
      ///Copie of starting point
      Point myStartingPoint;
      
      ///Current  position in the built-in iterator
      Iterator myPos;

      /// Copy of the underlying images
      ImageContainerBySTLMap<Domain,Value> *myMap;

      ///Dimension on which the iterator must iterate
      Dimension myDimension;

    };

    ImageContainerBySTLMap(const Point &,
			   const Point & ) {};

    ~ImageContainerBySTLMap() {};

    Value operator()(const Point &aPoint) throw( std::bad_alloc )
    {
      Iterator it = this->find( aPoint );
      if ( it == this->end() )
	throw std::bad_alloc();
      else
	return (*it).second; // this->operator[]( aPoint );
    }


    Value operator()(const Iterator &it) throw( std::bad_alloc )
    {
      if ( it == this->end() )
	throw std::bad_alloc();
      else
	return (*it).second;
    }
    

    void setValue(const Point &aPoint, const Value &aValue)
    {
      insert( aPoint, aValue ) ;
    }


    void setValue(SpanIterator &it, const Value &aValue)
    {
      ASSERT("NOT-YET-IMPLEMENTED");
    }

    void setValue(Iterator &it, const Value &aValue)
    {
      it->second = aValue;
    }


    SpanIterator span_begin(const Point &aPoint, const Dimension aDimension)
    {
      return SpanIterator ( aPoint, aDimension, this);
    }

    SpanIterator span_end(const Point &aPoint,const Dimension aDimension)
    {
      Point tmp = aPoint;
      tmp.at( aDimension ) = myLowerBound.at( aDimension ) +
	myUpperBound.at( aDimension ) - 
	myLowerBound.at( aDimension ) + 1;
      return SpanIterator( tmp, aDimension, this);
    }

    void allocate(const std::size_t aSize) {};

  private:
    
    Point myLowerBound;
    Point myUpperBound;
  };

} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerBySTLMap_h

#undef ImageContainerBySTLMap_RECURSES
#endif // else defined(ImageContainerBySTLMap_RECURSES)
