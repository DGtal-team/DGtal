#pragma once

/**
 * @file Image.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/09
 *
 * Header file for module Image.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Image_RECURSES)
#error Recursive header files inclusion detected in Image.h
#else // defined(Image_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Image_RECURSES

#if !defined Image_h
/** Prevents repeated inclusion of headers. */
#define Image_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Image
/**
 * Description of class 'Image' <p>
 * Aim:
 *
 * \todo Iterateurs dans le cas générique (const et non const) ( (*It) = Point-> Indice -> Valeur)
 * \todo spécialisé si TContainer=vector pour avoir des iterateurs customs (*It)= Indice -> Valeur
 * \todo Documentation
 */

template <class THyperRectDomain, typename TValue, class TContainer >
class Image
{
    // ----------------------- Standard services ------------------------------
public:
  
  typedef typename THyperRectDomain::TPoint TPoint;
  
  typedef typename TContainer::Iterator Iterator;
  typedef typename TContainer::ConstIterator ConstIterator;
    /**
     * Constuctor as the bounding box of two points.
     *
     * @param aPointA first point.
     * @param aPointB second point.
     */
    Image( const typename THyperRectDomain::TPoint &aPointA,
           const typename THyperRectDomain::TPoint &aPointB );

    /**
     * Destructor.x
     */
    ~Image();

    // ----------------------- Interface --------------------------------------
public:

    /**
    *  Generic function to get the value of an image at a point \param aPoint
    *
    */
    TValue operator()(const TPoint &aPoint);


    /**
     *  Generic function to get the value of an image at a point \param aPoint
     *
     */
    TValue operator()(const Iterator &aIt);
    


    // ----------------------- Built-in terators  from the container--------------------------------

    Iterator begin() {
        return myImageMap.begin();
    }

    Iterator end() {
        return myImageMap.end();
    }

    Iterator rend() {
        return myImageMap.rend();
    }

    Iterator rbegin() {
        return myImageMap.rend();
    }

    ConstIterator begin() const {
        return myImageMap.begin();
    }

    ConstIterator end() const {
        return myImageMap.end();
    }

    ConstIterator rend() const {
        return myImageMap.rend();
    }

    ConstIterator rbegin() const {
        return myImageMap.rend();
    }

    //-----------------------------------------------------------------------
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

    THyperRectDomain myDomain; ///Local copie of the HyperRectDomain (to have generic iterators) \todo should be removed in specialized classes
    TContainer myImageMap; ///Image Container

private:

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    * Forbidden by default.
    */
    Image & operator= ( const Image & other );

    // ------------------------- Internals ------------------------------------
private:



}; // end of class Image


/**
 * Overloads 'operator<<' for displaying objects of class 'Image'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Image' to write.
 * @return the output stream after the writing.
 */
template <class THyperRectDomain, typename T, class TCont>
inline
std::ostream&
operator<< ( std::ostream & out, const Image<THyperRectDomain,T,TCont> & object );




///////////////////////////////////////////////////////////////////////////////
// Inline methods.
#include "DGtal/kernel/images/Image.ih"


} // namespace DGtal
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Image_h

#undef Image_RECURSES
#endif // else defined(Image_RECURSES)
