#pragma once

/** 
 * @file Vector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 * 
 * Header file for module Vector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Vector_RECURSES)
#error Recursive header files inclusion detected in Vector.h
#else // defined(Vector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Vector_RECURSES

#if !defined Vector_h
/** Prevents repeated inclusion of headers. */
#define Vector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Vector
  /** 
   * Description of class 'Vector' <p>
   *
   * Aim: Implement the notion of Point in a Digital Space.
   */
  template<typename T, std::size_t N> 
  class Vector : public PointVector<T,N>
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
    * Constructor.
    */
    Vector();
    
    /**
     * Destructor. 
     */
    ~Vector();

    // ----------------------- Interface --------------------------------------
  public:

    /**
    * Addition operator.
    *
    * \param v is the Point that gets added to \a *this.
    */
    Vector<T,N>& operator+= (const Vector<T,N>& v);
    
    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    * Forbidden by default.
    */
    Vector<T,N> & operator=( const Vector<T,N> & other );
    
    
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
    Vector( const Vector & other );

   
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Vector


  /**
   * Overloads 'operator<<' for displaying objects of class 'Vector'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Vector' to write.
   * @return the output stream after the writing.
   */
  //std::ostream&
  //operator<<( std::ostream & out, const Vector & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/Vector.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Vector_h

#undef Vector_RECURSES
#endif // else defined(Vector_RECURSES)
