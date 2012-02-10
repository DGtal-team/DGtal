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
 * @file Image.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/07
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
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/CValue.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // Template class Image
  /**
   * Description of template class 'Image' <p>
   * \brief Aim: implements association bewteen points lying in a
   * digital domain and values. 
   *
   * This class is a lightweight proxy on ImageContainers (models of
   * CImage). Image class is also a model of CImage.
   *
   * @tparam TImageContainer an image container type (model of CImage).
   *
   *
   */
  template < typename TImageContainer >
  class Image
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));
 

                                                
    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Value Value;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Iterator Iterator;
    typedef typename TImageContainer::ConstIterator ConstIterator;
    typedef typename TImageContainer::ReverseIterator ReverseIterator;
    typedef typename TImageContainer::ConstReverseIterator ConstReverseIterator;
    typedef typename TImageContainer::Point Point;

    ///Pointer to the image container data.
    typedef CowPtr<TImageContainer> ImagePointer;


    /** 
     * Default constructor.
     * 
     */
    Image();

    /** 
     * Constructor from a domain. 
     * Create an instance of the container.
     * 
     * @param aDomain a digital domain.
     */
    Image(const Domain &aDomain):
      myImageContainer(new ImageContainer(aDomain))
    { }
    
    //copy
    Image(const ImageContainer &anImageContainer):
      myImageContainer(new ImageContainer(anImageContainer.domain()))
    { }

    //Smart ref
    Image(const CowPtr<ImageContainer> &anImageContainer):
      myImageContainer(anImageContainer)
    { }

    //Acquired
    Image(ImageContainer *anImageContainer):
      myImageContainer(anImageContainer)
    { }

    /**
     * Destructor.
     */
    ~Image();

    // ----------------------- Interface --------------------------------------
  public:
    
    /////////////////// Domains //////////////////
  
    /** 
     * Returns a reference to the image underlying domain.
     * 
     * @return a refernce to the domain. 
     */
    const Domain & domain() const
    {
      return myImageContainer->domain();
    }
        
    /////////////////// Accessors //////////////////

   
    /**
     * Get the value of an image at a given position given
     * by a Point.
     *
     * @pre the point must be in the domain
     *
     * @param aPoint the point.
     * @return the value at aPoint.
     */
    Value operator()(const Point & aPoint) const
    {
      return myImageContainer->operator()(aPoint);
    }


    /////////////////// Set values //////////////////

    /**
     * Set a value on an Image at a position specified by a Point.
     *
     * @pre @c it must be a point in the image domain.
     *
     * @param aPoint the point.
     * @param aValue the value.
     */
    void setValue(const Point &aPoint, const Value &aValue)
    {
      myImageContainer->setValue(aPoint,aValue);
    }
    
    /////////////////// Iterators //////////////////
    
    /*
     * Proxy for the begin method.
     *
     */
    ConstIterator begin() const
    {
      return myImageContainer->begin();
    }
        
    /*
     * Proxy for the begin method.
     *
     */
    Iterator begin()
    {
      return myImageContainer->begin();
    }
    
    /*
     * Proxy for the end method.
     *
     */
    ConstIterator end() const
    {
      return myImageContainer->end();
    }

    /*
     * Proxy for the end method.
     *
     */
    Iterator end()
    {
      return myImageContainer->end();
    }

    /*
     * Proxy for the rbegin method.
     *
     */
    ConstReverseIterator rbegin() const
    {
      return myImageContainer->rbegin();
    }

    /*
     * Proxy for the rbegin method.
     *
     */
    ReverseIterator rbegin()
    {
      return myImageContainer->rbegin();
    }

    /*
     * Proxy for the rend method.
     *
     */
    ConstReverseIterator rend() const
    {
      return myImageContainer->end();
    }
    /*
     * Proxy for the rend method.
     *
     */
    ReverseIterator rend()
    {
      return myImageContainer->rend();
    }

    

    /////////////////// API //////////////////
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    /**
     * @return the validity of the Image
     */
    bool isValid() const
    {
      return (myImageContainer->isValid() );
    }
    
    /** 
     * Construct a Iterator on the image at a position specified
     * by @c aPoint
     * 
     * @param aPoint a point to construct a Iterator on. 
     * 
     * @return a Iterator on @c aPoint
     */
    Iterator getIterator(const Point &aPoint)
    {
      return myImageContainer->getIterator(aPoint);
    }


    /** 
     * 
     * @return a const reference to the image container data.
     */
    const ImagePointer getPointer() const
    {
      return ImagePointer(myImageContainer);
    }

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  protected:

    ///Image instance
    ImagePointer myImageContainer;
    
 
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
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const Image<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/Image.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Image_h

#undef Image_RECURSES
#endif // else defined(Image_RECURSES)
