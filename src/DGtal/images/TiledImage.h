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

#include "DGtal/images/ImageCache.h"

#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/images/DefaultImageRange.h"

#include <ctime> // TEMP_MT
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class TiledImage
/**
 * Description of template class 'TiledImage' <p>
 * \brief Aim: implements a tiled image from a "bigger/original" one from an ImageFactory.
 * 
 * @tparam TImageContainer an image container type (model of CImage).
 * @tparam TImageFactory an image factory type (model of CImageFactory).
 * @tparam TImageCacheReadPolicy an image cache read policy class (model of CImageCacheReadPolicy).
 * @tparam TImageCacheWritePolicy an image cache write policy class (model of CImageCacheWritePolicy).
 * 
 * @note It is important to take into account that read and write policies are passed as aliases in the TiledImage constructor,
 * so for example, if two TiledImage instances are successively created with the same read policy instance,
 * the state of the cache for a given time is therefore the same for the two TiledImage instances !
 */
template <typename TImageContainer, typename TImageFactory, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
class TiledImage
{

    // ----------------------- Types ------------------------------

public:
    typedef TiledImage<TImageContainer, TImageFactory, TImageCacheReadPolicy, TImageCacheWritePolicy> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));
    BOOST_CONCEPT_ASSERT(( CImageFactory<TImageFactory> ));
    BOOST_CONCEPT_ASSERT(( CImageCacheReadPolicy<TImageCacheReadPolicy> ));
    BOOST_CONCEPT_ASSERT(( CImageCacheWritePolicy<TImageCacheWritePolicy> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename ImageContainer::Domain Domain;
    typedef typename ImageContainer::Point Point;
    typedef typename ImageContainer::Value Value;
    typedef typename ImageContainer::ConstRange ConstRange;
    typedef typename ImageContainer::Range Range;
    
    typedef TImageFactory ImageFactory;
    typedef typename ImageFactory::OutputImage OutputImage;
    
    typedef TImageCacheReadPolicy ImageCacheReadPolicy;
    typedef TImageCacheWritePolicy ImageCacheWritePolicy;
    typedef ImageCache<OutputImage, ImageFactory, ImageCacheReadPolicy, ImageCacheWritePolicy > MyImageCache;
    
    ///New types

    // ----------------------- Standard services ------------------------------

public:
  
    /**
     * Constructor.
     * @param anImageFactory alias on the image factory (see ImageFactoryFromImage or ImageFactoryFromHDF5).
     * @param aReadPolicy alias on a read policy.
     * @param aWritePolicy alias on a write policy.
     * @param N how many tiles we want for each dimension.
     */
    TiledImage(Alias<ImageFactory> anImageFactory,
                        Alias<ImageCacheReadPolicy> aReadPolicy,
                        Alias<ImageCacheWritePolicy> aWritePolicy,
                        typename Domain::Integer N):
      myN(N), myImageFactory(anImageFactory)
    {
        myImageCache = new MyImageCache(myImageFactory, aReadPolicy, aWritePolicy);    
        
        m_lowerBound = myImageFactory->domain().lowerBound();
        m_upperBound = myImageFactory->domain().upperBound();
        
        for(typename Domain::Integer i=0; i<Domain::dimension; i++)
          mySize[i] = (m_upperBound[i]-m_lowerBound[i]+1)/myN;
        
        setbuf(stdout, NULL); // TEMP_MT
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
        return myImageFactory->domain();
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
        return (myImageFactory->isValid() && myImageCache->isValid());
    }
    
    /**
     * Get the domain containing aPoint.
     *
     * @param aPoint the point.
     * @return the domain containing aPoint.
     */
    const Domain findSubDomain(const Point & aPoint) const
    {
      ASSERT(myImageFactory->domain().isInside(aPoint));
      
      typename Domain::Integer i;
      
      Point low;
      for(i=0; i<Domain::dimension; i++)
      {
        /*if ( (aPoint[i]-m_lowerBound[i]) < mySize[i] )
          low[i] = 0;
        else*/
          low[i] = (aPoint[i]-m_lowerBound[i])/mySize[i];
      }
      
      Point dMin, dMax;
      for(i=0; i<Domain::dimension; i++)
      {
        dMin[i] = (low[i]*mySize[i])+m_lowerBound[i];
        dMax[i] = dMin[i] + (mySize[i]-1);
        
        if (dMax[i] > m_upperBound[i]) // last tile
          dMax[i] = m_upperBound[i];
      }
      
      Domain di(dMin, dMax);
      return di;      
    }
    
    /**
     * Get the value of an image (from cache) at a given position given by aPoint.
     *
     * @param aPoint the point.
     * @return the value at aPoint.
     */
    Value operator()(const Point & aPoint)// const // TEMP_MT
    {
      ASSERT(myImageFactory->domain().isInside(aPoint));

      typename OutputImage::Value aValue;

      if (myImageCache->read(aPoint, aValue))
        return aValue;
      else
      {
        //trace.beginBlock("incCacheMissRead");
        myImageCache->incCacheMissRead();
        Domain d;

        t = clock();
        d = findSubDomain(aPoint);
        t = clock() - t; //if (t) trace.info() << "findSubDomain took " << t <<" ticks.\n";
        
        t = clock();
        myImageCache->update(d);
        t = clock() - t; //if (t) trace.info() << "update took " << t <<" ticks.\n";
        
        t = clock();
        myImageCache->read(aPoint, aValue);
        t = clock() - t; //if (t) trace.info() << "read took " << t <<" ticks.\n";
        //trace.endBlock();
        
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
        ASSERT(myImageFactory->domain().isInside(aPoint));
          
        if (myImageCache->write(aPoint, aValue))
          return;
        else
        {
          myImageCache->incCacheMissWrite();
          myImageCache->update(findSubDomain(aPoint));
          myImageCache->write(aPoint, aValue);
        }
    }
    
    /**
     * Get the cacheMissRead value.
     */
    const unsigned int getCacheMissRead() const
    {
        return myImageCache->getCacheMissRead();
    }
    
    /**
     * Get the cacheMissWrite value.
     */
    const unsigned int getCacheMissWrite() const
    {
        return myImageCache->getCacheMissWrite();
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    TiledImage() {}
    
    // ------------------------- Private Datas --------------------------------
protected:
    
    /// Number of tiles per dimension
    typename Domain::Integer myN;
    
    /// Width of a tile (for each dimension)
    Point mySize;
    
    /// ImageFactory pointer
    ImageFactory *myImageFactory;
    
    /// ImageCache pointer
    MyImageCache *myImageCache;
    
    /// domain lower and upper bound
    Point m_lowerBound, m_upperBound;

    // ------------------------- Internals ------------------------------------
private:
    /// for clock counting
    unsigned t; // TEMP_MT

}; // end of class TiledImage


/**
 * Overloads 'operator<<' for displaying objects of class 'TiledImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'TiledImage' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer, typename TImageFactory, typename TImageCacheReadPolicy, typename TImageCacheWritePolicy>
std::ostream&
operator<< ( std::ostream & out, const TiledImage<TImageContainer, TImageFactory, TImageCacheReadPolicy, TImageCacheWritePolicy> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/TiledImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TiledImage_h

#undef TiledImage_RECURSES
#endif // else defined(TiledImage_RECURSES)
