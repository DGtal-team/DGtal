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
 * @file ImageFactoryForImage.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/01/23
 *
 * Header file for module ImageFactoryForImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageFactoryForImage_RECURSES)
#error Recursive header files inclusion detected in ImageFactoryForImage.h
#else // defined(ImageFactoryForImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageFactoryForImage_RECURSES

#if !defined ImageFactoryForImage_h
/** Prevents repeated inclusion of headers. */
#define ImageFactoryForImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/base/Alias.h"
#include "DGtal/images/ImageAdapter.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class ImageFactoryForImage
/**
 * Description of template class 'ImageFactoryForImage' <p>
 * \brief Aim: todo
 */
template <typename TImageContainer>
class ImageFactoryForImage
{

    // ----------------------- Types ------------------------------

public:
    typedef ImageFactoryForImage<TImageContainer> Self; 
    
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Domain Domain;
    typedef typename TImageContainer::Value Value;
    
    ///New types
    typedef ImageAdapter<TImageContainer, Domain, DefaultFunctor, Value, DefaultFunctor, DefaultFunctor > OutputImage;

    // ----------------------- Standard services ------------------------------

public:

    ImageFactoryForImage(Alias<ImageContainer> anImage):
            myImagePtr(anImage)
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageFactoryForImage Ctor fromRef " << std::endl;
#endif
    }

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    */
    ImageFactoryForImage & operator= ( const ImageFactoryForImage & other )
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageFactoryForImage assignment " << std::endl;
#endif
        if (&other != this)
        {
            myImagePtr = other.myImagePtr;
        }
        return *this;
    }

    /**
     * Destructor.
     * Does nothing
     */
    ~ImageFactoryForImage() {}

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
     * Returns the pointer on the Image container data for the Domain aDomain.
     * @return an ImagePtr.
     */
    OutputImage * requestImage(const Domain &aDomain)
    {
        DefaultFunctor idD;
        DefaultFunctor idV;
        DefaultFunctor idVm1;
        
        OutputImage* adaptImage = new OutputImage(*myImagePtr, aDomain, idD, idV, idVm1);
        
        return adaptImage;
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    ImageFactoryForImage() {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageFactoryForImage Ctor default " << std::endl;
#endif
    }
    
    // ------------------------- Private Datas --------------------------------
protected:

    /// Alias on the image container
    ImageContainer * myImagePtr;

private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class ImageFactoryForImage


/**
 * Overloads 'operator<<' for displaying objects of class 'ImageFactoryForImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ImageFactoryForImage' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer>
std::ostream&
operator<< ( std::ostream & out, const ImageFactoryForImage<TImageContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageFactoryForImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageFactoryForImage_h

#undef ImageFactoryForImage_RECURSES
#endif // else defined(ImageFactoryForImage_RECURSES)
