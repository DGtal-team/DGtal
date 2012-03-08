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
 * @file StandardDSLQ0.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/08
 *
 * Header file for module StandardDSLQ0.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(StandardDSLQ0_RECURSES)
#error Recursive header files inclusion detected in StandardDSLQ0.h
#else // defined(StandardDSLQ0_RECURSES)
/** Prevents recursive inclusion of headers. */
#define StandardDSLQ0_RECURSES

#if !defined StandardDSLQ0_h
/** Prevents repeated inclusion of headers. */
#define StandardDSLQ0_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
#include "DGtal/math/arithmetic/Pattern.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class StandardDSLQ0
  /**
     Description of template class 'StandardDSLQ0' <p> \brief Aim:
     Represents a digital straight line in first quadrant (Q0: x >= 0,
     y >= 0 ).

     It is the 4-connected set of points (x,y) of Z2 satisfyting:

     \f$ mu <= ax - by < mu + a + b \f$

     @param TFraction the type chosen to represent fractions, a model
     of CFraction. You may use SternBrocot<>::Fraction for instance.

     Model of boost::CopyConstructible, boost::Assignable, CPointPredicate
   */
  template <typename TFraction>
  class StandardDSLQ0
  {
    // ----------------------- Standard types ------------------------------
  public:
    typedef TFraction Fraction;
    typedef StandardDSLQ0<TFraction> Self;
    typedef typename Fraction::Integer Integer;
    typedef typename Fraction::Size Size;

    // BOOST_CONCEPT_ASSERT(( CFraction<Fraction> ));

    typedef IntegerComputer<Integer> IC;
    typedef typename IC::IntegerParamType IntegerParamType;
    typedef typename IC::Vector2I Vector2I;
    typedef typename IC::Point2I Point2I;

    // Model of CPointPredicate
    typedef typename IC::Point2I Point;

    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor.
     */
    ~StandardDSLQ0();

    /**
       Constructor. Null DSL.
     */
    StandardDSLQ0();

    /**
       Copy constructor.
       @param other the object to clone.
     */
    StandardDSLQ0 ( const StandardDSLQ0 & other );

    /**
       Assignment.
       @param other the object to copy.
       @return a reference on 'this'.
    */
    StandardDSLQ0 & operator= ( const StandardDSLQ0 & other );

    /**
       Creates the DSL(a,b,mu).
       @param slope the slope a/b, where gcd(a,b)=1
       @param mu the shift to origin.
    */
    StandardDSLQ0( Fraction slope, IntegerParamType mu );

    /**
       Creates the DSL(a/g,b/g,mu), where g = gcd( a, b).
       @param a any integer
       @param b any integer 
       @param mu the shift to origin.
    */
    StandardDSLQ0( IntegerParamType a, IntegerParamType b, IntegerParamType mu );

    /// @param p any point in Z2.
    /// @return 'true' iff the point \a p belongs to this.
    bool operator()( const Point & p ) const;

    /// @return the slope of this DSL, an irreducible fraction
    Fraction slope() const;

    /// @return the shift to origin, which is also the lower
    /// diopantine constraint.
    const Integer & mu() const;

    /// @return the upper diopantine constraint, ie. mu + a + b - 1
    Integer mup() const;

    /// @return a the numerator of the slope.
    Integer a() const;
    /// @return b the denominator of the slope.
    Integer b() const;

    /// @return the pattern of this DSL
    const Pattern<Fraction> & pattern() const;

    /// Remainder a p.x - b p.y of point \a p in this DSL.
    /// @param p any point in Z2
    /// @return a p.x - b p.y
    Integer r( const Point & p ) const;

    /// @return the first upper leaning point of the DSL in the first
    /// quadrant (x >= 0).
    Point U() const;
    /// @return the first lower leaning point of the DSL with greater
    /// x than U().
    Point L() const;

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
  protected:
    /// the characteristic pattern of this DSL.
    Pattern<Fraction> myPattern;
    /// the shift to origin.
    Integer myMu;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class StandardDSLQ0


  /**
   * Overloads 'operator<<' for displaying objects of class 'StandardDSLQ0'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'StandardDSLQ0' to write.
   * @return the output stream after the writing.
   */
  template <typename TFraction>
  std::ostream&
  operator<< ( std::ostream & out, const StandardDSLQ0<TFraction> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/arithmetic/StandardDSLQ0.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined StandardDSLQ0_h

#undef StandardDSLQ0_RECURSES
#endif // else defined(StandardDSLQ0_RECURSES)
