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
 * @file ImageAdapter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/07
 *
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/09/04
 *
 * Header file for module ImageAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageAdapter_RECURSES)
#error Recursive header files inclusion detected in ImageAdapter.h
#else // defined(ImageAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageAdapter_RECURSES

#if !defined ImageAdapter_h
/** Prevents repeated inclusion of headers. */
#define ImageAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/images/CImage.h"
#include "DGtal/kernel/domains/CDomain.h"

#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/images/DefaultImageRange.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// Template class ImageAdapter
/**
 * Description of template class 'ImageAdapter' <p>
 * \brief Aim: implements a restricted image with a given domain (i.e. a subdomain).
 *
 * This class is (like Image class) a lightweight proxy on ImageContainers (models of CImage).
 * It uses a given Domain (i.e. a subdomain) but work directly (for read and write) thanks to an alias (i.e. a pointer) on the original ImageContainer in argument.
 * ImageAdapter class is also a model of CImage.
 * 
 * Caution :
 *  - ImageAdapter Domain Space dimension must be the same than the original ImageContainer Domain Space dimension,
 *  - the type of value of Point for the ImageAdapter Domain must also be the same than the type of value of Point for the original ImageContainer.
 *
 * @tparam TDomain a domain.
 * @tparam TImageContainer an image container type (model of CImage).
 *
 *
 */
template <typename TImageContainer, typename TDomain, typename TFunctorD, typename TFunctorV, typename TFunctorVm1>
class ImageAdapter
{

    // ----------------------- Types ------------------------------

public:
    typedef ImageAdapter<TImageContainer, TDomain, TFunctorD, TFunctorV, TFunctorVm1> Self; 

    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( CImage<TImageContainer> ));
    BOOST_CONCEPT_ASSERT(( CDomain<TDomain> ));
    BOOST_STATIC_ASSERT(TDomain::Space::dimension == TImageContainer::Domain::Space::dimension);
    BOOST_STATIC_ASSERT((boost::is_same< typename TDomain::Point, typename TImageContainer::Point>::value));

    ///Types copied from the container
    typedef TImageContainer ImageContainer;
    typedef typename TImageContainer::Point Point;
    typedef typename TImageContainer::Value Value;

    ///Pointer to the image container data.
    typedef TImageContainer* ImagePointer;

    typedef TDomain Domain;

    typedef DefaultConstImageRange<Self> ConstRange; 
    typedef DefaultImageRange<Self> Range; 

    // ----------------------- Standard services ------------------------------

public:

    ImageAdapter(ImageContainer &anImage, const Domain &aDomain, const TFunctorD &aFD, const TFunctorV &aFV, const TFunctorVm1 &aFVm1):
            myImagePointer(&anImage), mySubDomain(aDomain), myFD(&aFD), myFV(&aFV), myFVm1(&aFVm1)
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageAdapter Ctor fromRef "<<std::endl;
#endif
    }

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    */
    ImageAdapter & operator= ( const ImageAdapter & other )
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageAdapter assignment "<<std::endl;
#endif
        if (&other != this)
        {
            myImagePointer = other.myImagePointer;
            mySubDomain = other.mySubDomain;
            myFD = other.myFD;
            myFV = other.myFV;
            myFVm1 = other.myFVm1;
        }
        return *this;
    }


    /**
     * Destructor.
     * Does nothing
     */
    ~ImageAdapter() {}

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
        return mySubDomain;
    }

    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    ConstRange constRange() const
    {
        return ConstRange( *this );
    }

    /**
     * Returns the range of the underlying image
     * to iterate over its values
     *
     * @return a range.
     */
    Range range()
    {
        return Range( *this );
    }

    /////////////////// Accessors //////////////////


    /**
     * Get the value of an image at a given position given
     * by a Point.
     *
     * @pre the point must be in the domain
     *
     * @param aPoint the point.
     * @return the value at aPoint.
     */
    Value operator()(const Point & aPoint) const
    {
        ASSERT(this->domain().isInside(aPoint));
        
        //return myImagePointer->operator()(aPoint);
        return myFV->operator()(myImagePointer->operator()(myFD->operator()(aPoint)));
    }


    /////////////////// Set values //////////////////

    /**
     * Set a value on an Image at a position specified by a Point.
     *
     * @pre @c it must be a point in the image domain.
     *
     * @param aPoint the point.
     * @param aValue the value.
     */
    void setValue(const Point &aPoint, const Value &aValue)
    {
        ASSERT(this->domain().isInside(aPoint));
        
        //myImagePointer->setValue(aPoint, aValue);
        myImagePointer->setValue(myFD->operator()(aPoint), myFVm1->operator()(aValue));
    }



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
        return (myImagePointer->isValid() );
    }


    /**
     * Returns the pointer on the Image container data.
     * @return a const ImagePointer.
     */
    const ImageContainer * getPointer() const
    {
        return myImagePointer;
    }

    // ------------------------- Protected Datas ------------------------------
private:
    /**
     * Default constructor.
     */
    ImageAdapter() {
#ifdef DEBUG_VERBOSE
        trace.warning() << "ImageAdapter Ctor default "<<std::endl;
#endif
    }
    
    // ------------------------- Private Datas --------------------------------
protected:

    /// Owning pointer on the image container
    ImagePointer myImagePointer;
    
    /**
     * The image SubDomain
     */
    Domain mySubDomain;
    
    /**
     * Aliasing pointer on the underlying Domain functor
     */
    const TFunctorD* myFD;

    /**
     * Aliasing pointer on the underlying Value functor
     */
    const TFunctorV* myFV;
    
    /**
     * Aliasing pointer on the underlying "m-1" Value functor
     */
    const TFunctorVm1* myFVm1;


private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class ImageAdapter


/**
 * Overloads 'operator<<' for displaying objects of class 'ImageAdapter'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ImageAdapter' to write.
 * @return the output stream after the writing.
 */
template <typename TImageContainer, typename TDomain, typename TFunctorD, typename TFunctorV, typename TFunctorVm1>
std::ostream&
operator<< ( std::ostream & out, const ImageAdapter<TImageContainer, TDomain, TFunctorD, TFunctorV, TFunctorVm1> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageAdapter_h

#undef ImageAdapter_RECURSES
#endif // else defined(ImageAdapter_RECURSES)
