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
 * @file ImageCachePolicies.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/02/06
 *
 * Header file for module ImageCachePolicies.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageCachePolicies_RECURSES)
#error Recursive header files inclusion detected in ImageCachePolicies.h
#else // defined(ImageCachePolicies_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageCachePolicies_RECURSES

#if !defined ImageCachePolicies_h
/** Prevents repeated inclusion of headers. */
#define ImageCachePolicies_h

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
// Template class ImageCacheReadPolicyLast
/**
 * Description of template class 'ImageCacheReadPolicyLast' <p>
 * \brief Aim: implements a 'LAST' read policy cache.
 * 
 * @tparam TImageContainer an image container type (model of CImage).
 * @tparam TImageFactory an image factory.
 * 
 * The policy is done with 3 functions:
 * 
 *  - getPage :                 for getting the alias on the image that contains the a point or NULL if no image in the cache contains that point
 *  - getNextPageToDetach :     for getting the alias on the next image that we have to detach or NULL if no image have to be detached
 *  - updateCache :             for updating the cache according to the cache policy
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheReadPolicyLast
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheReadPolicyLast(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory), myCacheImagesPtr(NULL)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheReadPolicyLast() {}
    
    /**
     * Get the alias on the image that contains the point aPoint
     * or NULL if no image in the cache contains the point aPoint.
     * 
     * @param aPoint the point.
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
     * Update the cache according to the cache policy.
     *
     * @param aDomain the domain.
     */
    void updateCache(const Domain &aDomain);
    
protected:
    
    /// Alias on the images cache
    ImageContainer * myCacheImagesPtr;
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheReadPolicyLast

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheWritePolicyWT
/**
 * Description of template class 'ImageCacheWritePolicyWT' <p>
 * \brief Aim: implements a 'WT (direct)' write policy cache.
 * 
 * @tparam TImageContainer an image container type (model of CImage).
 * @tparam TImageFactory an image factory.
 * 
 * The policy is done with 2 functions:
 * 
 *  - writeOnPage :     for setting a value on an image at a given position given by a point
 *  - flushPage :       for flushing the image on disk according to the cache policy
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheWritePolicyWT
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheWritePolicyWT(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheWritePolicyWT() {}
    
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
    * Flush the image on disk according to the cache policy.
    *
    * @param anImageContainer the image.
    */
    void flushPage(TImageContainer * anImageContainer);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheWritePolicyWT

/////////////////////////////////////////////////////////////////////////////
// Template class ImageCacheWritePolicyWB
/**
 * Description of template class 'ImageCacheWritePolicyWB' <p>
 * \brief Aim: implements a 'WB (delayed)' write policy cache.
 * 
 * @tparam TImageContainer an image container type (model of CImage).
 * @tparam TImageFactory an image factory.
 * 
 * The policy is done with 2 functions:
 * 
 *  - writeOnPage :     for setting a value on an image at a given position given by a point
 *  - flushPage :       for flushing the image on disk according to the cache policy
 */
template <typename TImageContainer, typename TImageFactory>
class ImageCacheWritePolicyWB
{
public:
    typedef TImageFactory ImageFactory;
    
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    ImageCacheWritePolicyWB(Alias<ImageFactory> anImageFactory):
      myImageFactory(anImageFactory)
    {
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageCacheWritePolicyWB() {}
    
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
    * Flush the image on disk according to the cache policy.
    *
    * @param anImageContainer the image.
    */
    void flushPage(TImageContainer * anImageContainer);
    
protected:
    
    /// Alias on the image factory
    ImageFactory * myImageFactory;
    
}; // end of class ImageCacheWritePolicyWB

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageCachePolicies.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageCachePolicies_h

#undef ImageCachePolicies_RECURSES
#endif // else defined(ImageCachePolicies_RECURSES)
