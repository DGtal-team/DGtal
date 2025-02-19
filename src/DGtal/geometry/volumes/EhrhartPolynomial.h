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
 * Header file for module EhrhartPolynomial
 *
 * This file is part of the DGtal library.
 */

#if defined(EhrhartPolynomial_RECURSES)
#error Recursive header files inclusion detected in EhrhartPolynomial.h
#else // defined(EhrhartPolynomial_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EhrhartPolynomial_RECURSES

#if !defined EhrhartPolynomial_h
/** Prevents repeated inclusion of headers. */
#define EhrhartPolynomial_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/math/LagrangeInterpolation.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /**
    Description of class 'EhrhartPolynomial' <p> \brief Aim: This
    class implements the class Ehrhart Polynomial which is related to
    lattice point enumeration in bounded lattice polytopes.
    
    @see testEhrhartPolynomial.cpp

    @tparam TSpace an arbitrary model of CSpace, which tells in which space live bounded lattice polytopes.
    @tparam TInteger an arbitrary model of CInteger, which must be precise enough to compute the Ehrhart polynomial (either int64_t or BigInteger).
   */
  template <typename TSpace, concepts::CInteger TInteger>
  class EhrhartPolynomial
  {
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));

    // ----------------------- public types -----------------------------------
  public:
    typedef EhrhartPolynomial< TSpace, TInteger > Self;
    typedef TSpace                               Space;
    typedef TInteger                             Integer;
    typedef BoundedLatticePolytope<Space>        LatticePolytope;
    typedef LagrangeInterpolation< Integer >     Lagrange;
    typedef typename Lagrange::Polynomial        Polynomial;
    typedef std::size_t                          Size;
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor. 
     */
    ~EhrhartPolynomial() = default;

    /**
     * Constructor. The object is invalid.
     */
    EhrhartPolynomial()
      : myE(), myD( NumberTraits< Integer >::ZERO )
    {}

    /// Constructs the Ehrhart polynomial from a lattice polytope
    ///
    /// @param polytope the lattice polytope
    EhrhartPolynomial( const LatticePolytope& polytope )
    {
      init( polytope );
    }
                          
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    EhrhartPolynomial( const EhrhartPolynomial & other ) = default;

    /**
     * Move constructor.
     * @param other the object to clone.
     */
    EhrhartPolynomial( EhrhartPolynomial&& other ) = default;

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    EhrhartPolynomial & operator=( const EhrhartPolynomial & other ) = default;

    /**
     * Move assignment.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    EhrhartPolynomial & operator=( EhrhartPolynomial&& other ) = default;

    /// @return the Ehrhart polynomial (integral numerator part)
    inline Polynomial numerator() const
    {
      return myE;
    }

    /// The (integral) denominator of the Ehrhart polynomial
    inline Integer denominator() const
    {
      return myD;
    }

    /// Initializes the Ehrhart polynomial with a lattice polytope.
    ///
    /// @param polytope the lattice polytope
    void init( const LatticePolytope& polytope )
    {
      std::vector< Integer > X( Space::dimension + 1 );
      std::vector< Integer > Y( Space::dimension + 1 );
      X[ 0 ] = NumberTraits< Integer >::ZERO;
      Y[ 0 ] = NumberTraits< Integer >::ONE;
      for ( Integer i = 1; i <= Space::dimension; ++i )
        {
          LatticePolytope Q = i * polytope;
          Integer nb = Q.count();
          X[ i ] = i;
          Y[ i ] = nb;
        }
      // Compute polynomial (degree d)
      Lagrange L( X );
      myE = L.polynomial( Y );
      myD = L.denominator();
    }

    // @param t the dilation factor for this polytope P
    // @return the number of lattice points of t * P
    Integer count( Integer t ) const
    {
      return myE( t ) / myD;
    }

    // @param t the dilation factor for this polytope P
    // @return 0 if everything is correct.
    Integer remainder( Integer t ) const
    {
      return myE( t ) % myD;
    }
    
    // @param t the dilation factor for this polytope P
    // @return the number of lattice points interior to t * P
    // @note Use reciprocity formula (-1)^d E(-t)
    Integer countInterior( Integer t ) const { 
      return myE.degree() % 2 == 0 
        ?  ( myE( -t ) / myD ) 
        :  - ( myE( -t ) / myD ); 
    }

    // @param t the dilation factor for this polytope P
    // @return 0 if everything is correct.
    Integer remainderInterior( Integer t ) const { 
      return myE.degree() % 2 == 0 
        ?  ( myE( -t ) % myD ) 
        :  - ( myE( -t ) % myD ); 
    }
      
    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param that_stream the output stream where the object is written.
     */
    void selfDisplay( std::ostream & that_stream ) const
    {
      that_stream << "[EhrhartPolynomial num=" << numerator()
                  << " den=" << denominator() << " ]" << std::endl;
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool OK() const
    {
      return myD != NumberTraits< Integer >::ZERO;
    }
  
    // ------------------------- Datas ----------------------------------------
  protected:

    /// The Ehrhart polynomial (integral numerator part)
    Polynomial myE;
    /// The (integral) denominator of the Ehrhart polynomial
    Integer    myD;
  
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'EhrhartPolynomial'.
   * @tparam TSpace an arbitrary model of CSpace, which tells in which space live bounded lattice polytopes.
   * @tparam TInteger an arbitrary model of CInteger, which must be precise enough to compute the Ehrhart polynomial (either int64_t or BigInteger).
   * @param that_stream the output stream where the object is written.
   * @param that_object_to_display the object of class 'EhrhartPolynomial' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace, typename TInteger>
  std::ostream&
  operator<<( std::ostream & that_stream, 
              const EhrhartPolynomial< TSpace, TInteger > & that_object_to_display )
  {
    that_object_to_display.selfDisplay( that_stream );
    return that_stream;
  }
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined EhrhartPolynomial_h

#undef EhrhartPolynomial_RECURSES
#endif // else defined(EhrhartPolynomial_RECURSES)
