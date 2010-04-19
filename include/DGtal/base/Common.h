/** 
 * @file Common.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2009/12/10
 * 
 * Header file for module Common.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Common_RECURSES)
#error Recursive header files inclusion detected in Common.h
#else // defined(Common_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Common_RECURSES

#if !defined Common_h
#define Common_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>

//////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////// 
#ifndef INLINE
/** Forces inline if nothing is provided by the compiler. */
#define INLINE inline
#endif

namespace DGtal 
{
  
  
  /////////////////////////////////////////////////////////////////////////////
  // class Common
  /** 
   * Description of class 'Common' <p>
   * Aim: Holds the current configuration.
   */
  class Common
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~Common();

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
    INLINE Common();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    INLINE Common( const Common & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    INLINE Common & operator=( const Common & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Common


  /**
   * Overloads 'operator<<' for displaying objects of class 'Common'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Common' to write.
   * @return the output stream after the writing.
   */
  INLINE std::ostream&
  operator<<( std::ostream & out, const Common & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/base/Common.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Common_h

#undef Common_RECURSES
#endif // else defined(Common_RECURSES)
