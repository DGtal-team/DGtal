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
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Point
  /** 
   * Description of class 'Point' <p>
   * Aim: 
   */
  class Point
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~Point();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

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

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Point();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Point( const Point & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Point & operator=( const Point & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Point


  /**
   * Overloads 'operator<<' for displaying objects of class 'Point'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Point' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<<( std::ostream & out, const Point & object );

  
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
