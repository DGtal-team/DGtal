#pragma once

/**
 * @file ArithDSS4.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for module ArithDSS4.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithDSS4_RECURSES)
#error Recursive header files inclusion detected in ArithDSS4.h
#else // defined(ArithDSS4_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithDSS4_RECURSES

#if !defined ArithDSS4_h
/** Prevents repeated inclusion of headers. */
#define ArithDSS4_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/ArithDSS.h"
#include <cmath>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithDSS4
  /**
   * Description of template class 'ArithDSS4' <p>
   * \brief Aim:
   */
  template <typename Domain2D>
  class ArithDSS4: public ArithDSS<Domain2D>
  {
    // ----------------------- Standard services ------------------------------
  public:

		typedef typename Domain2D::Coordinate Integer;
		//2D point of a domain
		typedef typename Domain2D::Point Point;
		//2D vector of a domain
		typedef typename Domain2D::Vector Vector;

    /**
     * Constructor.
     */
    ArithDSS4(const Point& aFirstPoint, const Point& aSecondPoint);

    /**
     * Destructor.
     */
    ~ArithDSS4();

    // ----------------------- Interface --------------------------------------
  public:


    // ------------------------- Protected Datas ------------------------------
  protected:
    /**
		 * Computes the norm of the two components
     * of a 2D vector as the L1 norm
		 * @param x and y, two values. 
     * @return the L1 norm of a 2D vector.
     */
    Integer norm(const Integer  & x, const Integer & y) const;

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
    ArithDSS4 ( const ArithDSS4 & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ArithDSS4 & operator= ( const ArithDSS4 & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ArithDSS4


  /**
   * Overloads 'operator<<' for displaying objects of class 'ArithDSS4'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ArithDSS4' to write.
   * @return the output stream after the writing.
   */
 /*  template <typename T> */
/*   std::ostream&  operator<< ( std::ostream & out, const ArithDSS4<T> & object ) */
/*   { */
/*       object.selfDisplay( out); */
/*       return out; */
/*     } */
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/ArithDSS4.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithDSS4_h

#undef ArithDSS4_RECURSES
#endif // else defined(ArithDSS4_RECURSES)
