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
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class TiledImage
/**
 * Description of template class 'TiledImage' <p>
 * \brief Aim: implements a tiled image from a "bigger/original" one from an ImageFactory.
 * 
 * @tparam TImageContainer an image container type (model of ).
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
    
    typedef TImageFactory ImageFactory;
    typedef typename ImageFactory::OutputImage OutputImage;
    
    typedef TImageCacheReadPolicy ImageCacheReadPolicy;
    typedef TImageCacheWritePolicy ImageCacheWritePolicy;
    typedef ImageCache<OutputImage, ImageFactory, ImageCacheReadPolicy, ImageCacheWritePolicy > MyImageCache;
    
    typedef Self ConstRange;
    typedef Self Range;

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
            
        clock = new(Clock); // TEMP_MT
        myTicksUpdate = myTicksFindSubDomain = myTicksRead = 0;
    }

    /**
     * Destructor.
     */
    ~TiledImage()
    {
        // pb here (because Range/ConstRange) !!!
        
        //delete myImageCache;
        
        //delete clock; // TEMP_MT
    }

    // ----------------------- Interface --------------------------------------
public:

    /////////////////// Domains ///////////////////
    
    /**
     * Returns a reference to the underlying image domain.
     *
     * @return a reference to the domain.
     */
    const Domain & domain() const
    {
        return myImageFactory->domain();
    }
    
    const Domain domainCoords() const
    {
        Point lowerBoundCords, upperBoundCoords;
        
        for(typename Domain::Integer i=0; i<Domain::dimension; i++)
        {
          lowerBoundCords[i] = 0;
          upperBoundCoords[i] = myN;
          
          if (((m_upperBound[i]-m_lowerBound[i]+1) % myN) == 0)
            upperBoundCoords[i]--;
        }
        
        return Domain(lowerBoundCords, upperBoundCoords);
    }
    
    /////////////////////////// Custom Iterator /////////////
    
    /**
     * Specific TiledIterator on TiledImage.
     */

    class TiledIterator : public 
      std::iterator<std::bidirectional_iterator_tag, Value, ptrdiff_t, Value*, Value&> 
    {

      friend class TiledImage<ImageContainer, ImageFactory, ImageCacheReadPolicy, ImageCacheWritePolicy>;

    public:
      
      /*typedef std::bidirectional_iterator_tag iterator_category; // ???
      typedef Value value_type;
      typedef ptrdiff_t difference_type; // ???
      typedef Value* pointer;
      typedef Value& reference;*/
      
      TiledIterator();
      
      /**
       * Constructor.
       *
       * @param aBlockIterator
       * @param aTiledImage pointer to the TiledImage
       */
      TiledIterator ( typename Domain::Iterator aBlockIterator,
                     const TiledImage<ImageContainer, ImageFactory, ImageCacheReadPolicy, ImageCacheWritePolicy> *aTiledImage ) : myBlocksIterator ( aBlockIterator ), myTiledImage ( aTiledImage )
      {
        if ( myBlocksIterator != myTiledImage->domainCoords().end() )
        {
          myTile = myTiledImage->findTileFromCoords( (*myBlocksIterator) );
          myTileRangeIterator = myTile->range().begin();
        }
      }
      
      /**
       * operator *
       *
       * @return the value associated to the current TiledIterator position.
       */
      /*inline
      const Value & operator*() const
      {
        return (*myTileRangeIterator);
      }*/
      
      inline
      Value & operator*()
      {
        return (*myTileRangeIterator);
      }
      
      inline
      void setValue ( const Value aVal )
      {
        (*myTileRangeIterator) = aVal;
        myTiledImage->myImageFactory->flushImage(myTile); // TEMP
      }
      
      /**
       * Operator ==
       *
       * @return true if this and it are equals.
       */
      inline
      bool operator== ( const TiledIterator &it ) const
      {
        return ( ( this->myBlocksIterator == it.myBlocksIterator ) && ( this->myTileRangeIterator == it.myTileRangeIterator ) );
      }

      /**
       * Operator !=
       *
       * @return true if this and it are different.
       */
      inline
      bool operator!= ( const TiledIterator &it ) const
      {
        if ( myBlocksIterator == myTiledImage->domainCoords().end() )
          return false;
        
        return ( ( this->myBlocksIterator != it.myBlocksIterator ) || ( this->myTileRangeIterator != it.myTileRangeIterator ) );
      }

      /**
      * Implements the next() method to scan the domain points dimension by dimension
      * (lexicographic order).
      **/
      inline
      void nextLexicographicOrder()
      {
        myTileRangeIterator++;
          
        if ( myTileRangeIterator != myTile->range().end() )
          return;
        else
        {
          myBlocksIterator++;
          
          if ( myBlocksIterator == myTiledImage->domainCoords().end() )
            return;
          
          myTile = myTiledImage->findTileFromCoords( (*myBlocksIterator) );
          myTileRangeIterator = myTile->range().begin();
        }
      }
      
      /**
       * Operator ++ (++it)
       *
       */
      inline
      TiledIterator &operator++()
      {
        nextLexicographicOrder();
        return *this;
      }
      
      /**
       * Operator ++ (it++)
       *
       */
      inline
      TiledIterator operator++ ( int )
      {
        TiledIterator tmp = *this;
        nextLexicographicOrder();
        return tmp;
      }
      
      /**
      * Implements the prev() method to scan the domain points dimension by dimension
      * (lexicographic order).
      **/
      inline
      void prevLexicographicOrder()
      {
        // -- IF we are at the end... (reverse, --)
        if ( myBlocksIterator == myTiledImage->domainCoords().end() )
        {
          myBlocksIterator--;
          
          myTile = myTiledImage->findTileFromCoords( (*myBlocksIterator) );
          
          myTileRangeIterator = myTile->range().end();
          myTileRangeIterator--;
          
          return;
        }
        // -- IF we are at the end... (reverse, --)
        
        // ---
          
        if ( myTileRangeIterator != myTile->range().begin() )
        {
          myTileRangeIterator--;
          return;
        }
        else
        {
          if ( myBlocksIterator == myTiledImage->domainCoords().begin() )
            return;
          
          myBlocksIterator--;
          
          myTile = myTiledImage->findTileFromCoords( (*myBlocksIterator) );
          
          myTileRangeIterator = myTile->range().end();
          myTileRangeIterator--;
        }
      }

      /**
      * Operator -- (--it)
      *
      */
      inline
      TiledIterator &operator--()
      {
        prevLexicographicOrder();
        return *this;
      }

      /**
      * Operator -- (it--)
      */
      inline
      TiledIterator operator-- ( int )
      {
        TiledIterator tmp = *this;
        prevLexicographicOrder();
        return tmp;
      }

    private:
      /// TiledImage pointer
      const TiledImage *myTiledImage;
      
      /// Alias on the current tile
      ImageContainer * myTile;
      
      typename ImageContainer::Range::/*Output*/Iterator myTileRangeIterator;
      
      typename Domain::Iterator myBlocksIterator;
    };

    typedef std::reverse_iterator<TiledIterator> ReverseTiledIterator;
    
    typedef TiledIterator ConstIterator;
    typedef ReverseTiledIterator ConstReverseIterator;
    
    typedef TiledIterator OutputIterator;
    typedef ReverseTiledIterator ReverseOutputIterator;

    TiledIterator begin() const
    {
      return TiledIterator( this->domainCoords().begin(), this );
    }
    
    TiledIterator end() const
    {
      return TiledIterator( this->domainCoords().end(), this );
    }
    
    ReverseTiledIterator rbegin() const
    {
      return ReverseTiledIterator( end() );
    }
    
    // TODO -> aPoint
    ReverseTiledIterator rbegin(const Point& aPoint) const
    {
      return ReverseTiledIterator( end() );
    }

    ReverseTiledIterator rend() const
    {
      return ReverseTiledIterator( begin() );
    }
    
    /**
      * OutputIterator service.
      * @return an output itertor on the first elements
      */
    OutputIterator outputIterator()
    {
      return OutputIterator( begin() );
    }

    /**
    * ReverseOutputIterator service.
    * @return an output itertor on the first elements
    */
    ReverseOutputIterator routputIterator()
    {
      return ReverseOutputIterator ( end() );
    }
      
    /**
      * ReverseOutputIterator service.
      * @param aPoint a point
      * @return an output itertor on the point
      */
    // TODO -> aPoint
    ReverseOutputIterator routputIterator ( const Point &aPoint )
    {
      return ReverseOutputIterator ( end() ) ;
    }
    
    /////////////////////////// Ranges  /////////////////////
    
    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    ConstRange constRange() const
    {
      return *this;
    }

    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    Range range()
    {
      return *this;
    }

    /////////////////// API ///////////////////////

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
    
    const Domain findSubDomainFromCoords(const Point & aCoord) const
    {
      ASSERT(domainCoords().isInside(aCoord));
      
      typename Domain::Integer i;
      
      Point dMin, dMax;
      for(i=0; i<Domain::dimension; i++)
      {
        dMin[i] = (aCoord[i]*mySize[i])+m_lowerBound[i];
        dMax[i] = dMin[i] + (mySize[i]-1);
        
        if (dMax[i] > m_upperBound[i]) // last tile
          dMax[i] = m_upperBound[i];
      }
      
      Domain di(dMin, dMax);
      return di;      
    }
    
    ImageContainer * findTileFromCoords(const Point & aCoord) const
    {
      ASSERT(domainCoords().isInside(aCoord));
      
      Domain d = findSubDomainFromCoords( aCoord );
      ImageContainer *tile = myImageCache->getPage(d);
      if (!tile)
      {
        myImageCache->incCacheMissRead();
        myImageCache->update(d);
        tile = myImageCache->getPage(d);
      }
      
      return tile;
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
      bool res;

      clock->startClock();
      res = myImageCache->read(aPoint, aValue);
      myTicksRead += clock->stopClock();
      
      if (res)
        return aValue;
      else
      {
        //trace.beginBlock("incCacheMissRead");
          myImageCache->incCacheMissRead();
          Domain d;
          
          clock->startClock();
          d = findSubDomain(aPoint);
          myTicksFindSubDomain += clock->stopClock();
          
          clock->startClock();
          myImageCache->update(d);
          myTicksUpdate += clock->stopClock();
          
          clock->startClock();
          myImageCache->read(aPoint, aValue);
          myTicksRead += clock->stopClock();
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
    
    /**
     * Clear the cache and reset the cache misses
     */
    void clearCacheAndResetCacheMisses()
    {
      myImageCache->clearCacheAndResetCacheMisses();
    }
    
    /**
     * Clear the ticks value.
     */
    void clearTicksUpdateCache() // TEMP_MT
    {
        myImageCache->clearTicksUpdateCache();
    }
    
    /**
     * Get the ticks value.
     */
    const long getTicksUpdateCache() const // TEMP_MT
    {
        return myImageCache->getTicksUpdateCache();
    }
    
    /**
     * Clear the ticks value.
     */
    void clearTicksUpdate() // TEMP_MT
    {
        myTicksUpdate = 0;
    }
    
    /**
     * Get the ticks value.
     */
    const long getTicksUpdate() const // TEMP_MT
    {
        return myTicksUpdate;
    }
    
    /**
     * Clear the ticks value.
     */
    void clearTicksFindSubDomain() // TEMP_MT
    {
        myTicksFindSubDomain = 0;
    }
    
    /**
     * Get the ticks value.
     */
    const long getTicksFindSubDomain() const // TEMP_MT
    {
        return myTicksFindSubDomain;
    }
    
    /**
     * Clear the ticks value.
     */
    void clearTicksRead() // TEMP_MT
    {
        myTicksRead = 0;
    }
    
    /**
     * Get the ticks value.
     */
    const long getTicksRead() const // TEMP_MT
    {
        return myTicksRead;
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    TiledImage() {}
    
    /// for clock counting
    long myTicksUpdate, myTicksFindSubDomain, myTicksRead;
    Clock *clock; // TEMP_MT
    
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
