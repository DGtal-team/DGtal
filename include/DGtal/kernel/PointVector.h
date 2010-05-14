#pragma once

/** 
 * @file PointVector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/14
 * 
 * Header file for module PointVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PointVector_RECURSES)
#error Recursive header files inclusion detected in PointVector.h
#else // defined(PointVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointVector_RECURSES

#if !defined PointVector_h
/** Prevents repeated inclusion of headers. */
#define PointVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"


#include <boost/array.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class PointVector
  /** 
   * Description of class 'PointVector' <p>
   *
   * Aim: Implements basic operations that will be used in  \ref Point  and \ref Vector classes.
   *
   */
  
  template<typename T, std::size_t N> 
  class PointVector
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
    * Constructor.
    *
    */
    PointVector();
    
    
    /**
     * Destructor. 
     */
    ~PointVector();

    // ----------------------- Interface --------------------------------------
  public:

    
    /**
    * Returns the size of the vector (i.e. the number of its
    * coefficients).
    */
    std::size_t getDimension() const;
    
    
    /**
    * Returns the  \a i-th coefficient of the vector.
    *
    * \pre The \a i index must lie between \a 0 and \a size() .
    *
    * \param i is the index of the retrieved coefficient.
    */
    const T& ro(std::size_t i) const;
    
    /**
    * Returns a non-const reference to the \a i-th element of the
    * vector.
    *
    * \pre The \a i index must lie between \a 0 and \a size() .
    *
    * \param i is the index of the retrieved coefficient.
    */
    T& rw(uint i);
    
    
    /**
    * Multiplies \a *this by the \a coeff scalar number.
    *
    * \param coeff is the factor \a *this get multiplied by.
    */
    PointVector<T,N>& operator*= (T coeff);
    
    
    /**
    * Resets all the values to zero.
    */
    void zero();
    
    
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

    // ------------------------- Private Datas --------------------------------
  private:

    
    // ------------------------- Hidden services ------------------------------
  public:

    ///Internal data-structure: boost/array with constant size.
    boost::array<T,N> myArray;
    
    
  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    PointVector( const PointVector & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    PointVector & operator=( const PointVector & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class PointVector


  /**
   * Overloads 'operator<<' for displaying objects of class 'PointVector'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PointVector' to write.
   * @return the output stream after the writing.
   */
  //std::ostream&
  //operator<<( std::ostream & out, const PointVector<T,N> & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/kernel/PointVector.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PointVector_h

#undef PointVector_RECURSES
#endif // else defined(PointVector_RECURSES)
