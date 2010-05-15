#pragma once

/** 
 * @file Vector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
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
   * Aim: Implement the notion of Vector in a Digital Space. They are
   * not truely vectors (as in vector spaces) but rather
   * Z-modules. The main difference is that the inverse may not exist.
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

    /**
     * Constructor from array of values.
     *
     * @param ptrValues the array of values (should be at least as long as
     * the size of the vector)
     */
    Vector( const T * ptrValues );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Vector( const Vector & other );

    // ----------------------- Interface --------------------------------------
  public:

    /**
    * Addition operator.
    *
    * \param v is the Vector that gets added to \a *this.
    */
    Vector<T,N>& operator+= (const Vector<T,N>& v);
    
    /**
    * Addition operator.
    *
    * \param v is the Vector that gets added to \a *this.
    */
    Vector<T,N> operator+ (const Vector<T,N>& v) const;
    
    
    /**
    * Substraction operator with assignement.
    *
    * \param v is the Vector that gets substracted to \a *this.
    */
    Vector<T,N>& operator-= (const Vector<T,N>& v);
    
    /**
    * Substraction operator.
    *
    * \param v is the Vector that gets added to \a *this.
    */
    Vector<T,N> operator- (const Vector<T,N>& v) const;
    
    
    
    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
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


   
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Vector


  /**
   * Overloads 'operator<<' for displaying objects of class 'Vector'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Vector' to write.
   * @return the output stream after the writing.
   */
  template<typename T, std::size_t N> 
  std::ostream&
  operator<<( std::ostream & out, const Vector<T,N> & object );

  
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
