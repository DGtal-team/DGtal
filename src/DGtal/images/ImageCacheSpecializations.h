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
  
/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializations
/**
 * Description of template class 'ImageCacheSpecializations' <p>
 * \brief Aim: todo
 */
template <typename TImageCache, typename TImageContainer, typename TImageFactory, DGtal::ReadPolicy AReadSelector, DGtal::WritePolicy AWriteSelector>
class ImageCacheSpecializations
{
public:
    typedef TImageCache ImageCache;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializations(Alias<ImageCache> anImageCache):
      myImageCache(anImageCache)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializations() {}
    
    /**
     * Get the value of an image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool readSpecializations(const Point & aPoint, Value &aValue);
    
    /**
     * Set a value on an Image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool writeSpecializations(const Point & aPoint, const Value &aValue);
    
    /**
     * Update the cache according to the cache policy
     */
    void updateSpecializations(const Domain &aDomain);
    
protected:
    
    /// Alias on the cache object (not specialized)
    ImageCache * myImageCache;
    
}; // end of class ImageCacheSpecializations

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializations
/**
 * Description of template class 'ImageCacheSpecializations' <p>
 * \brief Aim: todo
 */
template <typename TImageCache, typename TImageContainer, typename TImageFactory>
class ImageCacheSpecializations<TImageCache, TImageContainer, TImageFactory, DGtal::CACHE_READ_POLICY_LAST, DGtal::CACHE_WRITE_POLICY_WT>
{
public:
    typedef TImageCache ImageCache;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializations(Alias<ImageCache> anImageCache):
      myImageCache(anImageCache), myCacheImagesPtr(NULL)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializations() {}
    
    /**
     * Get the value of an image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool readSpecializations(const Point & aPoint, Value &aValue);
    
    /**
     * Set a value on an Image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool writeSpecializations(const Point & aPoint, const Value &aValue);
    
    /**
     * Update the cache according to the cache policy
     */
    void updateSpecializations(const Domain &aDomain);
    
protected:
    
    /// Alias on the images cache
    ImageContainer * myCacheImagesPtr;
    
    /// Alias on the cache object (not specialized)
    ImageCache * myImageCache;
    
}; // end of class ImageCacheSpecializations

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheSpecializations
/**
 * Description of template class 'ImageCacheSpecializations' <p>
 * \brief Aim: todo
 */
template <typename TImageCache, typename TImageContainer, typename TImageFactory>
class ImageCacheSpecializations<TImageCache, TImageContainer, TImageFactory, DGtal::CACHE_READ_POLICY_LAST, DGtal::CACHE_WRITE_POLICY_WB>
{
public:
    typedef TImageCache ImageCache;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheSpecializations(Alias<ImageCache> anImageCache):
      myImageCache(anImageCache), myCacheImagesPtr(NULL)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheSpecializations() {}
    
    /**
     * Get the value of an image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool readSpecializations(const Point & aPoint, Value &aValue);
    
    /**
     * Set a value on an Image from cache at a given position given
     * by aPoint only if aPoint belongs to an image from cache.
     *
     * @param aPoint the point.
     * @param aValue the value.
     * 
     * @return 'true' if aPoint belongs to an image from cache, 'false' otherwise.
     */
    bool writeSpecializations(const Point & aPoint, const Value &aValue);
    
    /**
     * Update the cache according to the cache policy
     */
    void updateSpecializations(const Domain &aDomain);
    
protected:
    
    /// Alias on the images cache
    ImageContainer * myCacheImagesPtr;
    
    /// Alias on the cache object (not specialized)
    ImageCache * myImageCache;
    
}; // end of class ImageCacheSpecializations

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageCacheSpecializations.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageCacheSpecializations_h

#undef ImageCacheSpecializations_RECURSES
#endif // else defined(ImageCacheSpecializations_RECURSES)
