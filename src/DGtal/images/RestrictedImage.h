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
 * @file RestrictedImage.h
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
 * Header file for module RestrictedImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RestrictedImage_RECURSES)
#error Recursive header files inclusion detected in RestrictedImage.h
#else // defined(RestrictedImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RestrictedImage_RECURSES

#if !defined RestrictedImage_h
/** Prevents repeated inclusion of headers. */
#define RestrictedImage_h

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
// Template class RestrictedImage
/**
 * Description of template class 'RestrictedImage' <p>
 * \brief Aim: implements a restricted image with a given domain (i.e. a subdomain).
 *
 * This class is (like Image class) a lightweight proxy on ImageContainers (models of CImage).
 * It uses a given Domain (i.e. a subdomain) but work directly (for read and write) thanks to an alias (i.e. a pointer) on the original ImageContainer in argument.
 * RestrictedImage class is also a model of CImage.
 * 
 * Caution :
 *  - RestrictedImage Domain Space dimension must be the same than the original ImageContainer Domain Space dimension,
 *  - the type of value of Point for the RestrictedImage Domain must also be the same than the type of value of Point for the original ImageContainer.
 *
 * @tparam TDomain a domain.
 * @tparam TImageContainer an image container type (model of CImage).
 *
 *
 */
template < typename TDomain, typename TImageContainer >
class RestrictedImage
{

    // ----------------------- Types ------------------------------

public:
    typedef RestrictedImage<TDomain,TImageContainer> Self; 

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

    RestrictedImage(const Domain &aDomain, ImageContainer &anImage):
            mySubDomain(aDomain), myImagePointer(&anImage)
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "RestrictedImage Ctor fromRef "<<std::endl;
#endif
    }

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    */
    RestrictedImage & operator= ( const RestrictedImage & other )
    {
#ifdef DEBUG_VERBOSE
        trace.warning() << "RestrictedImage assignment "<<std::endl;
#endif
        if (&other != this)
        {
            myImagePointer = other.myImagePointer;
        }
        return *this;
    }


    /**
     * Destructor.
     * Does nothing
     */
    ~RestrictedImage() {}

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
        return myImagePointer->operator()(aPoint);
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
        myImagePointer->setValue(aPoint,aValue);
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
    RestrictedImage() {
#ifdef DEBUG_VERBOSE
        trace.warning() << "RestrictedImage Ctor default "<<std::endl;
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


private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class RestrictedImage


/**
 * Overloads 'operator<<' for displaying objects of class 'RestrictedImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'RestrictedImage' to write.
 * @return the output stream after the writing.
 */
template <typename TD, typename TIC>
std::ostream&
operator<< ( std::ostream & out, const RestrictedImage<TD, TIC> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/RestrictedImage.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RestrictedImage_h

#undef RestrictedImage_RECURSES
#endif // else defined(RestrictedImage_RECURSES)
