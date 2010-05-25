#pragma once

/**
 * @file Domain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/21
 *
 * Header file for module Domain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Domain_RECURSES)
#error Recursive header files inclusion detected in Domain.h
#else // defined(Domain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Domain_RECURSES

#if !defined Domain_h
/** Prevents repeated inclusion of headers. */
#define Domain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/Space.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Domain
/**
 * Description of class 'Domain' <p>
 * Aim: \a Domain class implements basic functions and iterators on digital domains upon a
 * digital space (\ref Space).
 *
 *
 *
 */

template <class TSpace>
class Domain
{

    // ----------------------- Standard services ------------------------------
public:

    typedef typename TSpace::PointType PointType;
    typedef TSpace SpaceType;
    
    /**
    * Constructor.
    */
    Domain();


    /**
     * Destructor.
     */
    ~Domain();

    // ----------------------- Interface --------------------------------------
public:

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


private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Domain ( const Domain & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Domain & operator= ( const Domain & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class Domain


/**
 * Overloads 'operator<<' for displaying objects of class 'Domain'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Domain' to write.
 * @return the output stream after the writing.
 */
template <class TSpace>
std::ostream&
operator<< ( std::ostream & out, const Domain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/Domain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Domain_h

#undef Domain_RECURSES
#endif // else defined(Domain_RECURSES)
