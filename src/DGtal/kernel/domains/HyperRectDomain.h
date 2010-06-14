#pragma once

/**
 * @file HyperRectDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/25
 *
 * Header file for module HyperRectDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain.h
#else // defined(HyperRectDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_RECURSES

#if !defined HyperRectDomain_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain_Iterator.h"
#include "DGtal/kernel/domains/HyperRectDomain_SpanIterator.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class HyperRectDomain
/**
 * Description of class 'HyperRectDomain' <p>
 * Aim:
 *
 * The following code snippet demonstrates how to use \p HyperRectDomain
 *
 *  \code
 *  #include <DGtal/kernel/Space.h>
 *  #include <DGtal/kernel/domains/HyperRectDomain.h>
 * ...
 *
 * //We create a digital Space based on 'int' integers and in dimension 4
 * typedef DGtal::Space<int,4> Space4DType;
 * typedef Space4DType::TPoint Point4DType;
 *
 * const int rawA[ ] = { 1, 2, 3 ,4};
 * const int rawB[ ] = { 5, 5, 3 ,4};
 * Point4DType A ( rawA );
 * Point4DType B ( rawB );
 *
 * //Domain construction from two points
 * DGtal::HyperRectDomain<Space4DType> myDomain ( A, B );
 *
 * //We just iterate on the Domain points and print out the point coordinates.
 * std::copy ( myDomain.begin(),
 *             myDomain.end(),
 *             std::ostream_iterator<Point4DType> ( std::cout, " " ) );
 *  \endcode
 *
 *
 * \see test_HyperRectDomain.cpp
 * \see test_HyperRectDomain-snippet.cpp
 */
template<class TSpace>
class HyperRectDomain
{
    // ----------------------- Standard services ------------------------------
public:

    typedef typename TSpace::TPoint TPoint;
    typedef typename TPoint::TValue TValue;

    ///Type def of domain iterators
    typedef HyperRectDomain_Iterator<TPoint> ConstIterator;
    typedef HyperRectDomain_SpanIterator<TPoint> ConstSpanIterator;


      
    /**
    * Default Constructor.
    */
    HyperRectDomain();

    /**
    * Constructor from  two points \param aPointA and \param aPoint B
    * defining the space diagonal.
    *
    */
    HyperRectDomain ( const TPoint &aPointA, const TPoint &aPointB );


    /**
     * Destructor.
     */
    ~HyperRectDomain();

    /**
    * Copy constructor.
    * @param other the object to clone.
    * Forbidden by default.
    */
    HyperRectDomain ( const HyperRectDomain & other );
    
    
    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    * Forbidden by default.
    */
    HyperRectDomain & operator= ( const HyperRectDomain & other );


    
    //------------- Global Iterator
    /**
    * begin() iterator.
    *
    **/
    ConstIterator begin() const;

    /**
    * begin(aPoint) iterator. Returns an iterator starting at \param aPoint
    *
    **/
    ConstIterator begin ( const TPoint &aPoint ) const;

    /**
    * end() iterator.
    *
    **/
    ConstIterator end() const;


    //------------- Span Iterator
    /**
    * Returns a Span iterator starting at \param aPoint and moving toward the dimension \param aDimension.
    *
    **/
    ConstSpanIterator span_begin ( const TPoint &aPoint, const std::size_t aDimension) const;


    /**
    * Creates a end() Span iterator along the dimension \param aDimension.
    *
    **/
    ConstSpanIterator span_end (const std::size_t aDimension) const;



// ----------------------- Interface --------------------------------------
public:

    /**
    * Returns the extent of the HyperRectDomain
    *
    **/
    TValue extent() const;


    /**
    * Returns the lowest point of the space diagonal.
    *
    **/
    const TPoint &lowerBound() const;

    /**
    * Returns the highest point of the space diagonal.
    *
    **/
    const TPoint &upperBound() const ;


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


///The lowest point of the space diagonal
    TPoint myLowerBound;
///The highest point of the space diagonal
    TPoint myUpperBound;

private:




// ------------------------- Internals ------------------------------------
private:

}; // end of class HyperRectDomain


/**
 * Overloads 'operator<<' for displaying objects of class 'HyperRectDomain'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'HyperRectDomain' to write.
 * @return the output stream after the writing.
 */
template<class TSpace>
std::ostream&
operator<< ( std::ostream & out, const HyperRectDomain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/domains/HyperRectDomain.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_h

#undef HyperRectDomain_RECURSES
#endif // else defined(HyperRectDomain_RECURSES)
