#pragma once

/** 
 * @file Space.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 * 
 * Header file for module Space.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Space_RECURSES)
#error Recursive header files inclusion detected in Space.h
#else // defined(Space_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Space_RECURSES

#if !defined Space_h
/** Prevents repeated inclusion of headers. */
#define Space_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/Point.h"
#include "DGtal/kernel/Vector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Space
  /** 
   * Description of class 'Space' <p>
   *
   * Aim: Space defines the fundamental structure of a Digital Space
   *
   */
  
  template <typename IntT, std::size_t Dimension>
  class Space
  {
    
    typedef IntT IntergerType;
    typedef std::size_t  DimensionType;
    
    typedef Point<IntT, Dimension> PointType;
    typedef Vector<IntT,Dimension> VectorType;
    //typedef Matrix<DimensionT,DimensionT,IntT> Matrix;
    
    
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
    * Constructor
    *
    */
    Space();
    
    /**
     * Destructor. 
     */
    ~Space();

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


  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Space( const Space & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Space & operator=( const Space & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Space


  /**
   * Overloads 'operator<<' for displaying objects of class 'Space'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Space' to write.
   * @return the output stream after the writing.
   */
 // std::ostream&
//  operator<<( std::ostream & out, const Space & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/Space.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Space_h

#undef Space_RECURSES
#endif // else defined(Space_RECURSES)
