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
 * @file ImageCache.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/24
 *
 * Header file for module ImageCache.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageCache_RECURSES)
#error Recursive header files inclusion detected in ImageCache.h
#else // defined(ImageCache_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageCache_RECURSES

#if !defined ImageCache_h
/** Prevents repeated inclusion of headers. */
#define ImageCache_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/Alias.h"

#include "DGtal/images/ImageFactoryFromImage.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class ImageCache
/**
 * Description of template class 'ImageCache' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCache
{

    // ----------------------- Types ------------------------------

public:
    typedef ImageCache<TImageContainer, TImageFactory> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    typedef TImageFactory ImageFactory;
    
    ///New types
    enum ReadPolicy{LAST, FIFO, LRU, NEIGHBORS};        // read policy
    enum WritePolicy{WT, WB};                           // write policy

    // ----------------------- Standard services ------------------------------

public:

    ImageCache(Alias<ImageFactory> anImageFactory, ReadPolicy AReadSelector=LAST):
            myImageFactoryPtr(anImageFactory), myReadPolicy(AReadSelector), myImagePtr(NULL)
    {
    }

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    */
    ImageCache & operator= ( const ImageCache & other )
    {
        if (&other != this)
        {
            myImageFactoryPtr =  other.myImageFactoryPtr;
            myReadPolicy = other.myReadPolicy;    
            myImagePtr = other.myImagePtr;
        }
        return *this;
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCache() {}

    // ----------------------- Interface --------------------------------------
public:

    /////////////////// Domains //////////////////


    /////////////////// Accessors //////////////////

    
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
        return (myImagePtr->isValid() );
    }
    
    /**
     * Get the value of an image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool read(const Point & aPoint, Value &aValue) const
    {
        //if (readSelector == LAST) // TODO : FIFO, LRU, NEIGHBORS
        {
          if (myImagePtr==NULL)
              return false;
          
          if (myImagePtr->domain().isInside(aPoint))
          {
              aValue = myImagePtr->operator()(aPoint);
              return true;
          }
          else
              return false;
        }
        
        return false;
    }
    
    /**
     * Set a value on an Image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool write(const Point & aPoint, const Value &aValue)
    {
        //if (readSelector == LAST) // TODO : FIFO, LRU, NEIGHBORS
        {
          if (myImagePtr==NULL)
              return false;
          
          if (myImagePtr->domain().isInside(aPoint))
          {
              myImagePtr->setValue(aPoint, aValue);
              return true;
          }
          else
              return false;
        }
        
        return false;
    }
    
    /**
     * Update the cache according to the cache policy
     */
    void update(const Domain &aDomain)
    {
        //if (readSelector == LAST) // TODO : FIFO, LRU, NEIGHBORS
        if (myImagePtr)
          myImageFactoryPtr->detachImage(myImagePtr);
        
        myImagePtr = myImageFactoryPtr->request(aDomain);
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    ImageCache() {}
    
    // ------------------------- Private Datas --------------------------------
protected:

    /// Alias on the image container
    ImageContainer * myImagePtr;
    
    /// Alias on the image factory
    ImageFactory * myImageFactoryPtr;
    
private:
  
    /// Cache policy
    ReadPolicy myReadPolicy;

    // ------------------------- Internals ------------------------------------
private:

}; // end of class ImageCache


/**
 * Overloads 'operator<<' for displaying objects of class 'ImageCache'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ImageCache' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer, typename TImageFactory>
std::ostream&
operator<< ( std::ostream & out, const ImageCache<TImageContainer, TImageFactory> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageCache.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageCache_h

#undef ImageCache_RECURSES
#endif // else defined(ImageCache_RECURSES)
