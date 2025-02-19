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
 * @file
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/08/24
 *
 * Header file for module LagrangeInterpolation
 *
 * This file is part of the DGtal library.
 */

#if defined(LagrangeInterpolation_RECURSES)
#error Recursive header files inclusion detected in LagrangeInterpolation.h
#else // defined(LagrangeInterpolation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LagrangeInterpolation_RECURSES

#if !defined LagrangeInterpolation_h
/** Prevents repeated inclusion of headers. */
#define LagrangeInterpolation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CEuclideanRing.h"
#include <vector>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /**
    Description of class 'LagrangeInterpolation' 
    <p> \brief Aim: This class implements Lagrange basis functions and Lagrange interpolation.
    
    @see testLagrangeInterpolation.cpp

    @tparam TEuclideanRing any model of CEuclideanRing like int, double, etc.
   */
  template <concepts::CEuclideanRing TEuclideanRing>
  class LagrangeInterpolation
  {

    // ----------------------- public types -----------------------------------
  public:
    typedef LagrangeInterpolation< TEuclideanRing > Self;
    typedef TEuclideanRing Ring;

    /// The monovariate polynomial type
    typedef DGtal::MPolynomial< 1, Ring > Polynomial;
    typedef std::size_t                   Size;
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~LagrangeInterpolation() = default;

    /**
     * Constructor. The object is invalid.
     */
    LagrangeInterpolation()
      : myDeterminant( NumberTraits< Ring >::ZERO )
    {}

    /// Constructs the Lagrange interpolation object with its x-values
    ///
    /// @param xvalues the vector of X-values, corresponding to the
    /// abscissa where the interpolation will take place.
    LagrangeInterpolation( const std::vector< Ring >& xvalues )
    {
      init( xvalues );
    }
                          
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    LagrangeInterpolation( const LagrangeInterpolation & other ) = default;

    /**
     * Move constructor.
     * @param other the object to clone.
     */
    LagrangeInterpolation( LagrangeInterpolation&& other ) = default;

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    LagrangeInterpolation & operator=( const LagrangeInterpolation & other ) = default;

    /**
     * Move assignment.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    LagrangeInterpolation & operator=( LagrangeInterpolation&& other ) = default;

    /// @return the number of interpolation points (or number of x-values)
    inline Size size() const
    {
      return myX.size();
    }

    /// @return the degree of Lagrange polynomials (i.e. `size()-1`)
    inline Size degree() const
    {
      return size() - 1;
    }

    /// @return the common denominator to Lagrange bases and Lagrange polynomial.
    inline Ring denominator() const
    {
      return myDeterminant;
    }
    
    /// Initializes the Lagrange interpolation object with its
    /// x-values, and computes the corresponding Lagrange Basis.
    ///
    /// @param xvalues the vector of X-values, corresponding to the
    /// abscissa where the interpolation will take place.
    void init( const std::vector< Ring >& xvalues )
    {
      myX           = xvalues;
      // Compute Vandermonde determinant
      myDeterminant = NumberTraits< Ring >::ONE;
      for ( Dimension i = 0; (i+1) < size(); ++i )
        for ( Dimension j = i + 1; j < size(); ++j )
          myDeterminant *= myX[ j ] - myX[ i ];
      // compute Lagrange polynomial basis.
      myLagrangeBasis.clear();
      myLagrangeBasis.reserve( size() );
      for ( Dimension j = 0; j < size(); ++j )
        {
          Ring c = myDeterminant;
          for ( Dimension m = 0; m < size(); ++m )
            if ( m != j ) 
              c /= myX[ j ] - myX[ m ];
          Polynomial P = c; // constant
          for ( Dimension m = 0; m < size(); ++m )
            if ( m != j ) 
              P *= mmonomial<Ring>( 1 ) - myX[ m ] * mmonomial<Ring>( 0 );
          myLagrangeBasis.push_back( P );
        }
    }

    /// @param[in] yvalues the y-values corresponding to the current
    /// x-values, in the same order.
    ///
    /// @return the numerator of the Lagrange polynomial that
    /// interpolates all given values (divide by denominator to get
    /// the exact Lagrange polynomial).
    ///
    /// @note if \a yvalues has not the correct size, then the zero
    /// polynomial is returned.
    Polynomial polynomial( const std::vector< Ring >& yvalues )
    {
      Polynomial P;
      if ( yvalues.size() != size() ) return P;
      for ( Dimension j = 0; j < size(); ++j )
        P += yvalues[ j ] * myLagrangeBasis[ j ];
      return P;
    }
    
    // ----------------------- Accessors ------------------------------
  public:

    /// @param i any value between 0 included and `size()` excluded.
    ///
    /// @return the numerator of \a i-th Lagrange basis polynomial
    /// (divide by denominator to get the exact Lagrange basis
    /// polynomial).
    Polynomial basis( Size i ) const
    {
      ASSERT( i < size() );
      return myLagrangeBasis[ i ];
    }
      
    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param that_stream the output stream where the object is written.
     */
    void selfDisplay( std::ostream & that_stream ) const
    {
      that_stream << "[LagrangeInterpolation det=" << myDeterminant << std::endl;
      for ( Size i = 0; i < size(); i++ )
        that_stream << "l_" << i << "=" << basis( i ) << std::endl;
      that_stream << "]";
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool OK() const
    {
      return myDeterminant != NumberTraits< Ring >::ZERO;
    }
  
    // ------------------------- Datas ----------------------------------------
  protected:

    /// The vector of X-values (abscissa)
    std::vector< Ring > myX;
    /// The determinant of the Vandermonde matrix corresponding to X-values.
    Ring                myDeterminant;
    /// The Lagrange polynomial basis corresponding to X-values.
    std::vector<Polynomial> myLagrangeBasis;
  
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'LagrangeInterpolation'.
   * @param that_stream the output stream where the object is written.
   * @param that_object_to_display the object of class 'LagrangeInterpolation' to write.
   * @return the output stream after the writing.
   */
  template <typename TEuclideanRing>
  std::ostream&
  operator<<( std::ostream & that_stream, 
        const LagrangeInterpolation< TEuclideanRing > & that_object_to_display )
  {
    that_object_to_display.selfDisplay( that_stream );
    return that_stream;
  }
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LagrangeInterpolation_h

#undef LagrangeInterpolation_RECURSES
#endif // else defined(LagrangeInterpolation_RECURSES)
