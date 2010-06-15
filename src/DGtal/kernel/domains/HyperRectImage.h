#pragma once

/**
 * @file HyperRectImage.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/09
 *
 * Header file for module HyperRectImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectImage_RECURSES)
#error Recursive header files inclusion detected in HyperRectImage.h
#else // defined(HyperRectImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectImage_RECURSES

#if !defined HyperRectImage_h
/** Prevents repeated inclusion of headers. */
#define HyperRectImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////
#include <string.h>

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectImage
/**
 * Description of class 'HyperRectImage' <p>
 * Aim:
 *
 *
 *  \todo ajouter un parametre template avec le Container (vector, map, ..) et faire des specialisations spécifiques des iterateurs.
 *
 */

template <class THyperRectDomain, typename TValue>
class HyperRectImage
{

    typedef typename THyperRectDomain::TPoint TPoint;
    typedef typename std::vector<TValue> TContainer;
		typedef typename std::size_t  TSizeType;
		
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Constuctor as the bounding box of two points.
     *
     * @param aPointA first point.
     * @param aPointB second point.
     */
    HyperRectImage( const typename THyperRectDomain::TPoint &aPointA,
                    const typename THyperRectDomain::TPoint &aPointB );

    /**
     * Destructor.x
     */
    ~HyperRectImage();

    // ----------------------- Interface --------------------------------------
public:

    TValue operator()(TPoint &aPoint);


    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

protected:

    THyperRectDomain myDomain; ///Local copie of the HyperRectDomain
    TContainer myImageMap; ///Image Container

private:

		 /**
     *  Linearized a point and return the vector position.
     * \param aPoint the point to convert to an index
     * \return the index of \param aPoint in the container
     */
    TSizeType linearized(TPoint &aPoint);


    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    * Forbidden by default.
    */
    HyperRectImage & operator= ( const HyperRectImage & other );

    // ------------------------- Internals ------------------------------------
private:



}; // end of class HyperRectImage


/**
 * Overloads 'operator<<' for displaying objects of class 'HyperRectImage'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'HyperRectImage' to write.
 * @return the output stream after the writing.
 */
template <class THyperRectDomain, typename T>
inline
std::ostream&
operator<< ( std::ostream & out, const HyperRectImage<THyperRectDomain,T> & object );


} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Inline methods.
#include "DGtal/kernel/domains/HyperRectImage.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectImage_h

#undef HyperRectImage_RECURSES
#endif // else defined(HyperRectImage_RECURSES)
