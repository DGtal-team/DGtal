#pragma once

/** 
 * @file Matrix.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 * 
 * Header file for module Matrix.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Matrix_RECURSES)
#error Recursive header files inclusion detected in Matrix.h
#else // defined(Matrix_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Matrix_RECURSES

#if !defined Matrix_h
/** Prevents repeated inclusion of headers. */
#define Matrix_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Matrix
  /** 
   * Description of class 'Matrix' <p>
   * Aim: 
   */
  class Matrix
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~Matrix();

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
    Matrix();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Matrix( const Matrix & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Matrix & operator=( const Matrix & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Matrix


  /**
   * Overloads 'operator<<' for displaying objects of class 'Matrix'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Matrix' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<<( std::ostream & out, const Matrix & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/Matrix.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Matrix_h

#undef Matrix_RECURSES
#endif // else defined(Matrix_RECURSES)
