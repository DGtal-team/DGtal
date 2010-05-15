#pragma once

/**
 * @file Point.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 *
 * Header file for module Point.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Point_RECURSES)
#error Recursive header files inclusion detected in Point.h
#else // defined(Point_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Point_RECURSES

#if !defined Point_h
/** Prevents repeated inclusion of headers. */
#define Point_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Point
/**
 * Description of class 'Point' <p>
 *
 * Aim: Implement the notion of Point in a Digital Space.
 *
 */

template<typename T, std::size_t N>
class Point : public PointVector<T,N>
{

    // ----------------------- Standard services ------------------------------
public:

    /**
    * Constructor.
    */
    Point();

    /**
    * Destructor.
    */
    ~Point();


    // ----------------------- Interface --------------------------------------
public:

    /**
    * Addition operator with assignement.
    *
    * \param v is the Point that gets added to \a *this.
    */
    Point<T,N>& operator+= ( const Point<T,N>& v );

    /**
    * Addition operator.
    *
    * \param v is the Point that gets added to \a *this.
    */
    Point<T,N> operator+ ( const Point<T,N>& v ) const;


    /**
    * Substraction operator with assignement.
    *
    * \param v is the Point that gets substracted to \a *this.
    */
    Point<T,N>& operator-= ( const Point<T,N>& v );

    /**
    * Substraction operator.
    *
    * \param v is the Point that gets added to \a *this.
    */
    Point<T,N> operator- ( const Point<T,N>& v ) const;



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

    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    // ------------------------- Internals ------------------------------------
private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Point ( const Point & other );


}; // end of class Point


/**
 * Overloads 'operator<<' for displaying objects of class 'Point'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Point' to write.
 * @return the output stream after the writing.
 */
//std::ostream&
//operator<<( std::ostream & out, const Point & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/Point.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Point_h

#undef Point_RECURSES
#endif // else defined(Point_RECURSES)
