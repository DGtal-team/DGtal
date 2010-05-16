#pragma once

/** 
 * @file Integer.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/13
 * 
 * Header file for module Integer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Integer_RECURSES)
#error Recursive header files inclusion detected in Integer.h
#else // defined(Integer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Integer_RECURSES

#if !defined Integer_h
/** Prevents repeated inclusion of headers. */
#define Integer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  /////////////////////////////////////////////////////////////////////////////
  // class Integer
  /** 
   * Description of class 'Integer' <p>
   * Aim: 
   */
  template <typename T>
  class Integer
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~Integer();

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
    Integer();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Integer( const Integer & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Integer & operator=( const Integer & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Integer


  /**
   * Overloads 'operator<<' for displaying objects of class 'Integer'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Integer' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<<( std::ostream & out, const Integer & object );

  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Integer_h

#undef Integer_RECURSES
#endif // else defined(Integer_RECURSES)
