#pragma once

/**
 * @file Point.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
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
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/Vector.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class Point
  /**
   * Description of class 'Point' <p>
   *
   * Aim: Implement the notion of Point in a Digital Space. Inherits
   * from PointVector which is the template container for Point and
   * Vector.
   *
   */

  template<typename T, std::size_t N>
    class Point : public PointVector<T,N>
    {

      // ----------------------- Standard services ------------------------------
    public:

      typedef typename PointVector<T,N>::Iterator Iterator;
      typedef typename PointVector<T,N>::ConstIterator ConstIterator;
      typedef T TValue;

      static const std::size_t Dimension = N;
      
      /**
       * Constructor.
       */
      Point();

      /**
       * Destructor.
       */
      ~Point();

      /**
       * Constructor from array of values.
       *
       * @param ptrValues the array of values (should be at least as long as
       * the size of the vector)
       */
      Point( const T * ptrValues );
    
    
      /**
       * Copy constructor.
       * @param other the object to clone.
       */
      Point( const Point & other );

      // ----------------------- Interface --------------------------------------
    public:

      /**
       * Addition operator with assignement.
       *
       * \param v is the Point that gets added to \a *this.
       */
      Point<T,N>& operator+= ( const Vector<T,N>& v );

      /**
       * Addition operator.
       *
       * \param v is the Point that gets added to \a *this.
       */
      Point<T,N> operator+ ( const Vector<T,N>& v ) const;


      /**
       * Substraction operator with assignement.
       *
       * \param v is the Point that gets substracted to \a *this.
       */
      Point<T,N>& operator-= ( const Vector<T,N>& v );

      /**
       * Substraction operator.
       * Point - Vector => Point
       *
       * \param v is the Point that gets added to \a *this.
       */
      Point<T,N> operator- ( const Vector<T,N>& v ) const;

      /**
       * Substraction operator.
       * Point - Point => Vector
       *
       * \param p is the Point that gets substracted to \a *this.
       */
      Vector<T,N> operator- ( const Point<T,N>& p ) const;



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



    }; // end of class Point


  /**
   * Overloads 'operator<<' for displaying objects of class 'Point'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Point' to write.
   * @return the output stream after the writing.
   */
  template<typename T, std::size_t N>
    inline
    std::ostream&
    operator<<( std::ostream & out, const Point<T,N> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/Point.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Point_h

#undef Point_RECURSES
#endif // else defined(Point_RECURSES)
