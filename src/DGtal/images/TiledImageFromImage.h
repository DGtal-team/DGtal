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
 * @tparam TImageContainer an image container type (model of CImage).
 * 
 * The tiled image is create here from an existing image and with two parameters (nX and nY) in order to create the list of subdomains which describe all the tiles.
 * The last parameter is the size of the cache (in that case, a FIFO cache).
 */
template <typename TImageContainer>
class TiledImageFromImage
{

    // ----------------------- Types ------------------------------

public:
    typedef TiledImageFromImage<TImageContainer> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    typedef ImageFactoryFromImage<TImageContainer > MyImageFactoryFromImage;
    typedef typename MyImageFactoryFromImage::OutputImage OutputImage;
    
    typedef ImageCacheReadPolicyFIFO<OutputImage, MyImageFactoryFromImage> MyImageCacheReadPolicyFIFO;
    typedef ImageCacheWritePolicyWT<OutputImage, MyImageFactoryFromImage> MyImageCacheWritePolicyWT;
    typedef ImageCache<OutputImage, MyImageFactoryFromImage, MyImageCacheReadPolicyFIFO, MyImageCacheWritePolicyWT > MyImageCache;
    
    ///New types

    // ----------------------- Standard services ------------------------------

public:

    TiledImageFromImage(Alias<ImageContainer> anImage, typename ImageContainer::Domain::Integer nX, typename ImageContainer::Domain::Integer nY, int sizeCache=10):
      myImagePtr(anImage)
    {
        myImageFactoryFromImage = new MyImageFactoryFromImage(myImagePtr);
        
        myImageCache = new MyImageCache(myImageFactoryFromImage, sizeCache);
        
        _sizeX = (myImagePtr->domain().upperBound()[0]-myImagePtr->domain().lowerBound()[0]+1)/nX;
        _sizeY = (myImagePtr->domain().upperBound()[1]-myImagePtr->domain().lowerBound()[1]+1)/nY;
        
        myDi = new std::vector<Domain>;
        
        typename ImageContainer::Domain::Integer x0, y0;
        typename ImageContainer::Domain::Integer x1, y1;
        Domain di;
        
        for(typename ImageContainer::Domain::Integer j=0; j<nY; ++j)
          for(typename ImageContainer::Domain::Integer i=0; i<nX; ++i)
          {
            x0 = myImagePtr->domain().lowerBound()[0]+(_sizeX*i);
            y0 = myImagePtr->domain().lowerBound()[1]+(_sizeY*j);
            x1 = myImagePtr->domain().lowerBound()[0]+(_sizeX*i)+(_sizeX-1);
            y1 = myImagePtr->domain().lowerBound()[1]+(_sizeY*j)+(_sizeY-1);
            di = Domain(Point(x0, y0),Point(x1, y1)); trace.info() << "di: " << di << std::endl;
            myDi->push_back(di);
          }
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~TiledImageFromImage()
    {
      delete myImageCache;
      delete myImageFactoryFromImage;
      
      delete myDi;
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
    /**
     * @return the validity of the Image
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
      
      typename ImageContainer::Domain::Integer xP, yP;
      typename ImageContainer::Domain::Integer xLow, yLow;
      
      xP = aPoint[0];
      yP = aPoint[1];
      
      xLow = xP/_sizeX; if (!(xP%_sizeX)) xLow--;
      yLow = yP/_sizeY; if (!(yP%_sizeY)) yLow--;
      
      Domain di = Domain(Point((xLow*_sizeX)+1, (yLow*_sizeY)+1),Point((xLow*_sizeX)+_sizeX, (yLow*_sizeY)+_sizeY));
      trace.info() << "di_findSubDomain_NEW: " << di << std::endl;
      //return di;
      
      for(std::size_t i=0; i<myDi->size(); ++i)
      {
        if ((*myDi)[i].isInside(aPoint))
        {
          trace.info() << "di_findSubDomain_OLD: " << (*myDi)[i] << std::endl;
          return (*myDi)[i];
        }
      }
      
      // compiler warning... should never happen
      VERIFY_MSG(true, "Shoud never happen (aPoint must be in one subdomain)");
      return myImagePtr->domain();        
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
    }
    
    /**
     * Set a value on an image (from cache) at a position specified by a aPoint.
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
    
    /// Domains list
    std::vector<Domain> * myDi;
    
    /// Width of a tile
    typename ImageContainer::Domain::Size _sizeX;
    
    /// Height of a tile
    typename ImageContainer::Domain::Size _sizeY;
    
    /// ImageFactory pointer
    MyImageFactoryFromImage *myImageFactoryFromImage;
    
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
template <typename TImageContainer>
std::ostream&
operator<< ( std::ostream & out, const TiledImageFromImage<TImageContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/TiledImageFromImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TiledImageFromImage_h

#undef TiledImageFromImage_RECURSES
#endif // else defined(TiledImageFromImage_RECURSES)
