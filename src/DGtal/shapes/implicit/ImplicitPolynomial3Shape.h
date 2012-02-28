/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file ImplicitPolynomial3Shape.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2012/02/14
 *
 * Header file for module ImplicitPolynomial3Shape.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImplicitPolynomial3Shape_RECURSES)
#error Recursive header files inclusion detected in ImplicitPolynomial3Shape.h
#else // defined(ImplicitPolynomial3Shape_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImplicitPolynomial3Shape_RECURSES

#if !defined ImplicitPolynomial3Shape_h
/** Prevents repeated inclusion of headers. */
#define ImplicitPolynomial3Shape_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/CPredicate.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/shapes/implicit/CImplicitFunction.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of template class 'ImplicitPolynomial3Shape' <p>
   * \brief Aim: model of CEuclideanOrientedShape concepts to create a
   * shape from a polynomial.
   *
   * Model of CImplicitFunction
   *
   * @tparam TSpace the Digital space definition.
   */
  
  template < typename TSpace >
  class ImplicitPolynomial3Shape
  {

  public:
    typedef ImplicitPolynomial3Shape<TSpace> Self;
    typedef TSpace Space;
    typedef typename Space::RealPoint RealPoint;
    typedef typename Space::RealVector RealVector;
    typedef typename RealPoint::Coordinate Ring;
    typedef typename Space::Integer Integer;
    typedef MPolynomial< 3, Ring > Polynomial3;
    typedef Ring Value;
    
    BOOST_STATIC_ASSERT(( Space::dimension == 3 ));
    
    /** 
        Constructor from an arbitrary polynomial.
        
        @param poly any multivariate polynomial (the number of
        variables is the dimension of the space)
    */
    ImplicitPolynomial3Shape( const Polynomial3 & poly );
    
    /**
       Assignment.
       @param other the object to copy.
       @return a reference on 'this'.
     */
    ImplicitPolynomial3Shape & operator= ( const ImplicitPolynomial3Shape & other );
    
    /** 
        Destructor.
    */    
    ~ImplicitPolynomial3Shape();

    /** 
        Initialize from an arbitrary polynomial.
        
        @param poly any multivariate polynomial (the number of
        variables is the dimension of the space)
    */
    void init( const Polynomial3 & poly );

    // ----------------------- Interface --------------------------------------
  public:
    
    /**
       @param aPoint any point in the Euclidean space.
       @return the value of the polynomial at \a aPoint.
    */
    double operator()(const RealPoint &aPoint) const;
    
    /**
       @param aPoint any point in the Euclidean space.
       @return 'true' if the polynomial value is > 0.
    */
    bool isInside(const RealPoint &aPoint) const;

    /**
       @param aPoint any point in the Euclidean space.

       @return INSIDE if the polynomial value is > 0, OUTSIDE if < 0,
       ON otherwise.
    */
    Orientation orientation(const RealPoint &aPoint) const;

    /**
       @param aPoint any point in the Euclidean space.
       @return the gradient vector of the polynomial at \a aPoint.
    */
    inline
    RealVector gradient( const RealPoint &aPoint ) const;

    // ----------------------- Interface --------------------------------------
  public:
    
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

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /// The 3-polynomial defining the implicit shape.
    Polynomial3 myPolynomial;
    /// The gradient 3-polynomials (computed).
    Polynomial3 myDerivatives[ 3 ];
   
    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ImplicitPolynomial3Shape();

  private:

    
  }; // end of class ImplicitPolynomial3Shape


  /**
   * Overloads 'operator<<' for displaying objects of class 'ImplicitPolynomial3Shape'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImplicitPolynomial3Shape' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const ImplicitPolynomial3Shape<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImplicitPolynomial3Shape_h

#undef ImplicitPolynomial3Shape_RECURSES
#endif // else defined(ImplicitPolynomial3Shape_RECURSES)
