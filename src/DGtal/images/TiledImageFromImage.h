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
 * @file TiledImageFromImage.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/24
 *
 * Header file for module TiledImageFromImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TiledImageFromImage_RECURSES)
#error Recursive header files inclusion detected in TiledImageFromImage.h
#else // defined(TiledImageFromImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TiledImageFromImage_RECURSES

#if !defined TiledImageFromImage_h
/** Prevents repeated inclusion of headers. */
#define TiledImageFromImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/Alias.h"

#include "DGtal/images/ImageFactoryFromImage.h"
#include "DGtal/images/ImageCache.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class TiledImageFromImage
/**
 * Description of template class 'TiledImageFromImage' <p>
 * \brief Aim: implements a tiled image from a "bigger/original" one.
 * 
 *
 * The tiled image is create here from an existing image and with three parameters.
 * The first parameter is an alias on the image factory (see ImageFactoryFromImage).
 * The second parameter is an alias on a read policy.
 * The third parameter is an alias on a write policy.
 * The fourth parameter is to set how many tiles we want for each dimension.
 * 
 *
 * @tparam TImageContainer an image container type (model of CImage).
 * @tparam TImageFactoryFromImage an image factory type (model of CImageFactory).
 * @tparam TImageCacheReadPolicy an image cache read policy type (model of CImageCacheReadPolicy).
 * @tparam TImageCacheWritePolicy an image cache write policy type (model of CImageCacheWritePolicy).
 */
template <typename TImageContainer, typename TImageFactoryFromImage, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
class TiledImageFromImage
{

    // ----------------------- Types ------------------------------

public:
    typedef TiledImageFromImage<TImageContainer, TImageFactoryFromImage, TImageCacheReadPolicy, TImageCacheWritePolicy> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    typedef TImageFactoryFromImage ImageFactoryFromImage;
    typedef typename ImageFactoryFromImage::OutputImage OutputImage;
    
    typedef TImageCacheReadPolicy ImageCacheReadPolicy;
    typedef TImageCacheWritePolicy ImageCacheWritePolicy;
    typedef ImageCache<OutputImage, ImageFactoryFromImage, ImageCacheReadPolicy, ImageCacheWritePolicy > MyImageCache;
    
    ///New types

    // ----------------------- Standard services ------------------------------

public:
  
    /**
     * Constructor.
     * @param anImage alias on the underlying image container.
     * @param anImageFactoryFromImage alias on the image factory (see ImageFactoryFromImage).
     * @param aReadPolicy alias on a read policy.
     * @param aWritePolicy alias on a write policy.
     * @param N how many tiles we want for each dimension.
     */
    TiledImageFromImage(Alias<ImageContainer> anImage,
                        Alias<ImageFactoryFromImage> anImageFactoryFromImage,
                        Alias<ImageCacheReadPolicy> aReadPolicy,
                        Alias<ImageCacheWritePolicy> aWritePolicy,
                        typename ImageContainer::Domain::Integer N):
      myImagePtr(&anImage),  myN(N), myImageFactoryFromImage(&anImageFactoryFromImage)
    {
        myImageCache = new MyImageCache(myImageFactoryFromImage, aReadPolicy, aWritePolicy);
        
        for(typename ImageContainer::Domain::Integer i=0; i<ImageContainer::Domain::dimension; i++)
          mySize[i] = (myImagePtr->domain().upperBound()[i]-myImagePtr->domain().lowerBound()[i]+1)/myN;
    }

    /**
     * Destructor.
     */
    ~TiledImageFromImage()
    {
        delete myImageCache;
    }

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
     bool isValid() const
    {
        return (myImagePtr->isValid());
    }
    
    /**
     * Get the domain containing aPoint.
     *
     * @param aPoint the point.
     * @return the domain containing aPoint.
     */
    const Domain findSubDomain(const Point & aPoint) const
    {
      ASSERT(myImagePtr->domain().isInside(aPoint));
      
      typename ImageContainer::Domain::Integer i;
      
      Point low;
      for(i=0; i<ImageContainer::Domain::dimension; i++)
      {
        low[i] = aPoint[i]/mySize[i];
        if (!(aPoint[i]%mySize[i])) low[i]--;
      }
      
      Point dMin, dMax;
      for(i=0; i<ImageContainer::Domain::dimension; i++)
      {
        dMin[i] = (low[i]*mySize[i])+1;
        dMax[i] = (low[i]*mySize[i])+mySize[i];
      }
      
      Domain di = Domain(dMin, dMax);
      return di;      
    }
    
    /**
     * Get the value of an image (from cache) at a given position given by aPoint.
     *
     * @param aPoint the point.
     * @return the value at aPoint.
     */
    Value operator()(const Point & aPoint) const
    {
      ASSERT(myImagePtr->domain().isInside(aPoint));

      typename OutputImage::Value aValue;
        
      if (myImageCache->read(aPoint, aValue))
        return aValue;
      else
        {
          myImageCache->update(findSubDomain(aPoint));
          myImageCache->read(aPoint, aValue);
          return aValue;
        }
      
      // Unspecified behavior, returning the default constructed value.
      return aValue;
    }
    
    /**
     * Set a value on an image (in cache) at a position specified by a aPoint.
     *
     * @param aPoint the point.
     * @param aValue the value.
     */
    void setValue(const Point &aPoint, const Value &aValue)
    {
        ASSERT(myImagePtr->domain().isInside(aPoint));
          
        if (myImageCache->write(aPoint, aValue))
          return;
        else
        {
          myImageCache->update(findSubDomain(aPoint));
          myImageCache->write(aPoint, aValue);
        }
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    TiledImageFromImage() {}
    
    // ------------------------- Private Datas --------------------------------
protected:

    /// Alias on the image container
    ImageContainer * myImagePtr;
    
    /// Number of tiles per dimension
    typename ImageContainer::Domain::Integer myN;
    
    /// Width of a tile (for each dimension)
    Point mySize;
    
    /// ImageFactory pointer
    ImageFactoryFromImage *myImageFactoryFromImage;
    
    /// ImageCache pointer
    MyImageCache *myImageCache;
    
private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class TiledImageFromImage


/**
 * Overloads 'operator<<' for displaying objects of class 'TiledImageFromImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'TiledImageFromImage' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer, typename TImageFactoryFromImage, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
std::ostream&
operator<< ( std::ostream & out, const TiledImageFromImage<TImageContainer, TImageFactoryFromImage, TImageCacheReadPolicy, TImageCacheWritePolicy> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/TiledImageFromImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TiledImageFromImage_h

#undef TiledImageFromImage_RECURSES
#endif // else defined(TiledImageFromImage_RECURSES)
