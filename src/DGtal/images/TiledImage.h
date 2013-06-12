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
 * @file TiledImage.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/24
 *
 * Header file for module TiledImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TiledImage_RECURSES)
#error Recursive header files inclusion detected in TiledImage.h
#else // defined(TiledImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TiledImage_RECURSES

#if !defined TiledImage_h
/** Prevents repeated inclusion of headers. */
#define TiledImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/CImageFactory.h"
#include "DGtal/images/CImageCacheReadPolicy.h"
#include "DGtal/images/CImageCacheWritePolicy.h"
#include "DGtal/base/Alias.h"

#include "DGtal/images/ImageFactoryFromImage.h"
#include "DGtal/images/ImageCache.h"

#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/images/DefaultImageRange.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class TiledImage
/**
 * Description of template class 'TiledImage' <p>
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
template <typename TImageContainer, typename TDomain, typename TImageFactoryFromImage, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
class TiledImage
{

    // ----------------------- Types ------------------------------

public:
    typedef TiledImage<TImageContainer, TDomain, TImageFactoryFromImage, TImageCacheReadPolicy, TImageCacheWritePolicy> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));
    BOOST_CONCEPT_ASSERT(( CImageFactory<TImageFactoryFromImage> ));
    BOOST_CONCEPT_ASSERT(( CImageCacheReadPolicy<TImageCacheReadPolicy> ));
    BOOST_CONCEPT_ASSERT(( CImageCacheWritePolicy<TImageCacheWritePolicy> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    //typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    typedef typename TImageContainer::ConstRange ConstRange;
    typedef typename TImageContainer::Range Range;
    
    typedef TDomain Domain;
    
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
    TiledImage(ConstAlias<Domain> aDomain,
                        Alias<ImageFactoryFromImage> anImageFactoryFromImage,
                        Alias<ImageCacheReadPolicy> aReadPolicy,
                        Alias<ImageCacheWritePolicy> aWritePolicy,
                        typename Domain::Integer N):
      myDomain(aDomain),  myN(N), myImageFactoryFromImage(anImageFactoryFromImage)
    {
        myImageCache = new MyImageCache(myImageFactoryFromImage, aReadPolicy, aWritePolicy);
        
        for(typename Domain::Integer i=0; i<Domain::dimension; i++)
          mySize[i] = (myDomain->upperBound()[i]-myDomain->lowerBound()[i]+1)/myN;
    }

    /**
     * Destructor.
     */
    ~TiledImage()
    {
        delete myImageCache;
    }

    // ----------------------- Interface --------------------------------------
public:

    /////////////////// Domains //////////////////
    
    /**
     * Returns a reference to the underlying image domain.
     *
     * @return a reference to the domain.
     */
    const Domain & domain() const
    {
        return *myDomain;
    }

    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    /*ConstRange constRange() const
    {
        //return myImagePtr->constRange();
    }*/

    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    /*Range range()
    {
        //return myImagePtr->range();
    }*/

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
        return (myDomain->isValid() && myImageFactoryFromImage->isValid());
    }
    
    /**
     * Get the domain containing aPoint.
     *
     * @param aPoint the point.
     * @return the domain containing aPoint.
     */
    const Domain findSubDomain(const Point & aPoint) const
    {
      ASSERT(myDomain->isInside(aPoint));
      
      typename Domain::Integer i;
      
      Point low;
      for(i=0; i<Domain::dimension; i++)
      {
        if ( (aPoint[i]-myDomain->lowerBound()[i]) < mySize[i] )
          low[i] = 0;
        else
          low[i] = (aPoint[i]-myDomain->lowerBound()[i])/mySize[i];
      }
      
      Point dMin, dMax;
      for(i=0; i<Domain::dimension; i++)
      {
        dMin[i] = (low[i]*mySize[i])+myDomain->lowerBound()[i];
        dMax[i] = (low[i]*mySize[i])+myDomain->lowerBound()[i]+(mySize[i]-1);
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
      ASSERT(myDomain->isInside(aPoint));

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
        ASSERT(myDomain->isInside(aPoint));
          
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
    TiledImage() {}
    
    // ------------------------- Private Datas --------------------------------
protected:

    /// Alias on the image domain
    const Domain *myDomain;
    
    /// Number of tiles per dimension
    typename Domain::Integer myN;
    
    /// Width of a tile (for each dimension)
    Point mySize;
    
    /// ImageFactory pointer
    ImageFactoryFromImage *myImageFactoryFromImage;
    
    /// ImageCache pointer
    MyImageCache *myImageCache;
    
private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class TiledImage


/**
 * Overloads 'operator<<' for displaying objects of class 'TiledImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'TiledImage' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer, typename TDomain, typename TImageFactoryFromImage, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
std::ostream&
operator<< ( std::ostream & out, const TiledImage<TImageContainer, TDomain, TImageFactoryFromImage, TImageCacheReadPolicy, TImageCacheWritePolicy> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/TiledImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TiledImage_h

#undef TiledImage_RECURSES
#endif // else defined(TiledImage_RECURSES)
