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
#include "DGtal/base/Alias.h"

#include "DGtal/images/ImageFactoryFromImage.h"
#include "DGtal/images/ImageCache.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class TiledImage
/**
 * Description of template class 'TiledImage' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer>
class TiledImage
{

    // ----------------------- Types ------------------------------

public:
    typedef TiledImage<TImageContainer> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;
    
    typedef ImageFactoryFromImage<TImageContainer > MyImageFactoryFromImage;
    typedef typename MyImageFactoryFromImage::OutputImage OutputImage;
    
    typedef ImageCache<OutputImage > MyImageCache;
    
    ///New types

    // ----------------------- Standard services ------------------------------

public:

    TiledImage(Alias<ImageContainer> anImage, 
               Alias<std::vector<Domain> > Di):
      myImagePtr(anImage), myDi(Di)
    {
        myFactImage = new MyImageFactoryFromImage(myImagePtr);
        myImageCache = new MyImageCache(MyImageCache::LAST);
    }

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    */
    TiledImage & operator= ( const TiledImage & other )
    {
        if (&other != this)
        {
            myImagePtr = other.myImagePtr;
            myDi = other.myDi;
            myFactImage = other.myFactImage;
            myImageCache = other.myImageCache;
        }
        return *this;
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~TiledImage()
    {
      delete myFactImage;
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
    /**
     * @return the validity of the Image
     */
    bool isValid() const
    {
        return (myImagePtr->isValid() );
    }
    
    /**
     * Get the first domain containing aPoint.
     *
     * @param aPoint the point.
     * @return the domain containing aPoint.
     */
    Domain findSubDomain(const Point & aPoint) const
    {
      ASSERT( myImagePtr->domain().isInside(aPoint));
      
      for(std::size_t i=0; i<myDi.size(); ++i)
        if (myDi[i].isInside(aPoint))
          return myDi[i];
      
      //Compiler warning... should never happen
      VERIFY_MSG(true, "Shoud never happen (aPoint must be in one subdomain)");
      return myImagePtr->domain();        
    }
    
    /**
     * Get the value of an image (from cache) at a given position given
     * by aPoint.
     *
     * @param aPoint the point.
     * @return the value at aPoint.
     */
    Value operator()(const Point & aPoint) const
    {
      ASSERT( myImagePtr->domain().isInside(aPoint));

      typename OutputImage::Value aValue;
        
      if (myImageCache->read(aPoint, aValue))
        return aValue;
      else
        {
          myImageCache->update(myFactImage->requestImage(findSubDomain(aPoint)));
          myImageCache->read(aPoint, aValue);
          return aValue;
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

    /// Alias on the image container
    ImageContainer * myImagePtr;
    
    /// Domains list
    std::vector<Domain> myDi;
    
    /// ImageFactory pointer
    MyImageFactoryFromImage *myFactImage;
    
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
template <typename TImageContainer>
std::ostream&
operator<< ( std::ostream & out, const TiledImage<TImageContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/TiledImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TiledImage_h

#undef TiledImage_RECURSES
#endif // else defined(TiledImage_RECURSES)
