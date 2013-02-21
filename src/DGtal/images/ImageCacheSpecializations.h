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
 * @file ImageCacheSpecializations.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/02/06
 *
 * Header file for module ImageCacheSpecializations.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageCacheSpecializations_RECURSES)
#error Recursive header files inclusion detected in ImageCacheSpecializations.h
#else // defined(ImageCacheSpecializations_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageCacheSpecializations_RECURSES

#if !defined ImageCacheSpecializations_h
/** Prevents repeated inclusion of headers. */
#define ImageCacheSpecializations_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/Alias.h"

#include "DGtal/images/ImageCache.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

#if(0)
/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializationsRead
/**
 * Description of template class 'ImageCacheSpecializationsRead' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory, DGtal::ReadPolicy AReadSelector>
class ImageCacheSpecializationsRead
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializationsRead(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializationsRead() {}
    
    /**
     * Get the alias on the image that contains the point aPoint
     * or NULL if no image in the cache contains the point aPoint.
     *
     * @return the alias on the image container or NULL pointer.
     */
    TImageContainer * getPage(const Point & aPoint);
    
    /**
     * Get the alias on the next image that we have to detach
     * or NULL if no image have to be detached.
     *
     * @return the alias on the image container or NULL pointer.
     */
    TImageContainer * getNextPageToDetach();
    
    /**
     * Update the cache according to the cache policy
     */
    void updateCache(const Domain &aDomain);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheSpecializationsRead

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializationsWrite
/**
 * Description of template class 'ImageCacheSpecializationsWrite' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory, DGtal::WritePolicy AWriteSelector>
class ImageCacheSpecializationsWrite
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializationsWrite(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializationsWrite() {}
    
    /**
    * Set a value on an image at a given position given
    * by aPoint.
    *
    * @param anImageContainer the image.
    * @param aPoint the point.
    * @param aValue the value.
    */
    void writeOnPage(TImageContainer * anImageContainer, const Point & aPoint, const Value &aValue);
    
    /**
    * Flush the image on disk according to the cache policy
    *
    * @param anImageContainer the image.
    */
    void flushPage(TImageContainer * anImageContainer);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheSpecializationsWrite
#endif

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializationsRead
/**
 * Description of template class 'ImageCacheSpecializationsRead' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheSpecializationsRead<TImageContainer, TImageFactory, DGtal::CACHE_READ_POLICY_LAST>
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializationsRead(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory), myCacheImagesPtr(NULL)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializationsRead() {}
    
    /**
     * Get the alias on the image that contains the point aPoint
     * or NULL if no image in the cache contains the point aPoint.
     *
     * @return the alias on the image container or NULL pointer.
     */
    TImageContainer * getPage(const Point & aPoint);
    
    /**
     * Get the alias on the next image that we have to detach
     * or NULL if no image have to be detached.
     *
     * @return the alias on the image container or NULL pointer.
     */
    TImageContainer * getNextPageToDetach();
    
    /**
     * Update the cache according to the cache policy
     */
    void updateCache(const Domain &aDomain);
    
protected:
    
    /// Alias on the images cache
    ImageContainer * myCacheImagesPtr;
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheSpecializationsRead

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializationsWrite
/**
 * Description of template class 'ImageCacheSpecializationsWrite' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheSpecializationsWrite<TImageContainer, TImageFactory, DGtal::CACHE_WRITE_POLICY_WT>
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializationsWrite(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializationsWrite() {}
    
    /**
    * Set a value on an image at a given position given
    * by aPoint.
    *
    * @param anImageContainer the image.
    * @param aPoint the point.
    * @param aValue the value.
    */
    void writeOnPage(TImageContainer * anImageContainer, const Point & aPoint, const Value &aValue);
    
    /**
    * Flush the image on disk according to the cache policy
    *
    * @param anImageContainer the image.
    */
    void flushPage(TImageContainer * anImageContainer);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheSpecializationsWrite

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializationsWrite
/**
 * Description of template class 'ImageCacheSpecializationsWrite' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheSpecializationsWrite<TImageContainer, TImageFactory, DGtal::CACHE_WRITE_POLICY_WB>
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializationsWrite(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializationsWrite() {}
    
    /**
    * Set a value on an image at a given position given
    * by aPoint.
    *
    * @param anImageContainer the image.
    * @param aPoint the point.
    * @param aValue the value.
    */
    void writeOnPage(TImageContainer * anImageContainer, const Point & aPoint, const Value &aValue);
    
    /**
    * Flush the image on disk according to the cache policy
    *
    * @param anImageContainer the image.
    */
    void flushPage(TImageContainer * anImageContainer);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheSpecializationsWrite

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageCacheSpecializations.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageCacheSpecializations_h

#undef ImageCacheSpecializations_RECURSES
#endif // else defined(ImageCacheSpecializations_RECURSES)
