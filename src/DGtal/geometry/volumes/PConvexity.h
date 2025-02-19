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
 * @file PConvexity.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2024/06/20
 *
 * Header file for module PConvexity.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PConvexity_RECURSES)
#error Recursive header files inclusion detected in PConvexity.h
#else // defined(PConvexity_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PConvexity_RECURSES

#if !defined PConvexity_h
/** Prevents repeated inclusion of headers. */
#define PConvexity_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/geometry/volumes/ConvexityHelper.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace detail
  {
    /// Hidden class to represent the P-convexity in a recursive way.
    /// Only used to compute P-convexity, but not exposed to users.
    ///
    /// @note This d-dimensional object builds d (d-1)-dimensional
    /// similar objects, but the k-th remembers to build only k lower
    /// dimensional ones.
    ///
    /// @tparam dim the dimension of the digital space
    /// @tparam TInteger any model of integer (used to represent digital point coordinates).
    template < Dimension dim,
	       concepts::CInteger TInteger = DGtal::int32_t >
    struct RecursivePConvexity {
      /// Integer must be a model of the concept CInteger.
      using Integer        = TInteger;
      using Point          = DGtal::PointVector< dim, Integer >;
      using ProjPoint      = DGtal::PointVector< dim-1, Integer >;
      using ProjPConvexity = DGtal::detail::RecursivePConvexity< dim - 1, Integer >;

      /// Parameter bd is used to build exactly 2^d - 1 PConvexity objects
      /// when starting at dimension d.
      /// @param bd the maximum axis of projection.
      RecursivePConvexity( Dimension bd = dim )
      {
	init( bd );
      }
  
      /// @param bd the maximum axis of projection.
      void init( Dimension bd = dim )
      {
	for ( Dimension j = 0; j < bd; j++ )
	  projp.push_back( ProjPConvexity( j ) );
      }
  
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe when 'true' performs convex hull computations
      /// with arbitrary precision integer (if available), otherwise
      /// chooses a compromise between speed and precision (int64_t).
      ///
      /// @return 'true' if and only if X is a digitally convex set in the
      /// classic sense, i.e. \f$ Conv(X) \cap Z^d = X \f$.
      ///
      /// @pre X must not contain any duplicates.
      static
      bool is0Convex( const std::vector< Point >& X, bool safe ) 
      {
	if ( X.empty() ) return true;
	// Build polytope according to internal integer type.
	if ( safe )
	  {
	    typedef typename DGtal::detail::ConvexityHelperInternalInteger< Integer, true >::Type
	      InternalInteger;
	    const auto P = ConvexityHelper< dim, Integer, InternalInteger >::
	      computeLatticePolytope( X, false, false );
	    const std::size_t number_lattice_points_in_P = P.count();
	    return number_lattice_points_in_P == X.size();
	  }
	else
	  {
	    typedef typename DGtal::detail::ConvexityHelperInternalInteger< Integer, false >::Type
	      InternalInteger;
	    const auto P = ConvexityHelper< dim, Integer, InternalInteger >::
	      computeLatticePolytope( X, false, false );
	    const std::size_t number_lattice_points_in_P = P.count();
	    return number_lattice_points_in_P == X.size();
	  }
      }
  
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe when 'true' performs convex hull computations
      /// with arbitrary precision integer (if available), otherwise
      /// chooses a compromise between speed and precision (int64_t).
      ///
      /// @return 'true' if and only if X is a P-convex digital set.
      ///
      /// @pre X must not contain any duplicates.
      bool isPConvex( const std::vector< Point >& X, bool safe ) const
      {
	if ( ! is0Convex( X, safe ) ) return false;
	for ( std::size_t j = 0; j < projp.size(); j++ )
	  {
	    const auto pi_j_X = project( X, j );
	    if ( ! projp[ j ].isPConvex( pi_j_X, safe ) ) return false;
	  }
	return true;
      }

      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe when 'true' performs convex hull computations
      /// with arbitrary precision integer (if available), otherwise
      /// chooses a compromise between speed and precision (int64_t).
      ///
      /// @pre X must not contain any duplicates.
      ///
      /// @return a measure that has value 1.0 when X is digitally convex, and
      /// less otherwise.
      static
      double convexityMeasure( const std::vector< Point >& X, bool safe ) 
      {
	if ( X.empty() ) return 1.0;
	// Build polytope according to internal integer type.
	if ( safe )
	  {
	    typedef typename DGtal::detail::ConvexityHelperInternalInteger< Integer, true >::Type
	      InternalInteger;
	    const auto P = ConvexityHelper< dim, Integer, InternalInteger >::
	      computeLatticePolytope( X, false, false );
	    const std::size_t number_lattice_points_in_P = P.count();
	    return double( X.size() ) / double( number_lattice_points_in_P );
	  }
	else
	  {
	    typedef typename DGtal::detail::ConvexityHelperInternalInteger< Integer, false >::Type
	      InternalInteger;
	    const auto P = ConvexityHelper< dim, Integer, InternalInteger >::
	      computeLatticePolytope( X, false, false );
	    const std::size_t number_lattice_points_in_P = P.count();
	    return double( X.size() ) / double( number_lattice_points_in_P );
	  }
      }
      
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe when 'true' performs convex hull computations
      /// with arbitrary precision integer (if available), otherwise
      /// chooses a compromise between speed and precision (int64_t).
      ///
      /// @pre X must not contain any duplicates.
      ///
      /// @return a measure that has value 1.0 when X is P-convex (or
      /// equivalently fully convex), and less otherwise.  
      double fullConvexityMeasure( const std::vector< Point >& X, bool safe ) const
      {
	double m = convexityMeasure( X, safe );
	for ( std::size_t j = 0; j < projp.size(); j++ )
	  {
	    auto pX = project( X, j );
	    m      *= projp[ j ].fullConvexityMeasure( pX, safe );
	  }
	return m;
      }
  
      /// Projects a point \a p along dimension \a a.
      ///
      /// @param[in] p any digital point
      /// @param[in] a any dimension
      /// @return the digital point of dimension (d-1) with omitted a-th coordinate.
      static
      ProjPoint project( const Point& p, Dimension a ) 
      {
	ProjPoint pp;
	Dimension j = 0;
	for ( Dimension i = 0; i < Point::dimension; i++ )
	  if ( i != a ) pp[ j++ ] = p[ i ];
	return pp;
      }

      /// Projects the range of points \a p along dimension \a a.
      ///
      /// @param[in] p any range of digital points
      /// @param[in] a any dimension
      ///
      /// @return the range of digital points of dimension (d-1) with
      /// omitted a-th coordinate.
      ///
      /// @post the returned range has no duplicates.
      static
      std::vector< ProjPoint > project( const std::vector< Point >& p, Dimension a ) 
      {
	std::vector< ProjPoint > pp( p.size() );
	for ( std::size_t i = 0; i < p.size(); i++ )
	  pp[ i ] = project( p[ i ], a );
	std::sort( pp.begin(), pp.end() );
	auto last = std::unique( pp.begin(), pp.end() );
	pp.erase( last, pp.end() );
	return pp;
      }

      /// The array of lower dimensional P-convexities.
      std::vector< ProjPConvexity > projp;
    };

    /// Hidden class to represent the P-convexity in a recursive way.
    /// Only used to compute P-convexity, but not exposed to users.
    /// Specialization for dimension 1
    ///
    /// @note This d-dimensional object builds d (d-1)-dimensional
    /// similar objects, but the k-th remembers to build only k lower
    /// dimensional ones.
    ///
    /// @tparam dim the dimension of the digital space
    /// @tparam TInteger any model of integer (used to represent digital point coordinates).
    template < concepts::CInteger TInteger >
    struct RecursivePConvexity< 1, TInteger> {
      /// Integer must be a model of the concept CInteger.
      using Integer     = TInteger;
      using Point       = PointVector< 1, Integer >;

      /// Default constructor. Nothing to do.
      RecursivePConvexity( Dimension /* unused parameter in 1D specialization */ ) 
      {}
      
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe is a not used parameter for dimension 1, but is
      /// kept for the meta-programming recursive definition of
      /// P-convexity.
      ///
      /// @return 'true' if and only if X is a digital set in the
      /// classic sense, i.e. \f$ Conv(X) \cap Z^d = X \f$.
      ///
      /// @note Unused second parameter.
      static
      bool is0Convex( std::vector< Point > X, bool safe ) 
      {
	(void) safe;
	std::sort( X.begin(), X.end() );
	return X.empty()
	  || ( ( Integer(X.back()[ 0 ]) - Integer(X.front()[ 0 ]) + Integer(1) )
	       == Integer( X.size() ) );
      }

      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe is a not used parameter for dimension 1, but is
      /// kept for the meta-programming recursive definition of
      /// P-convexity.
      ///
      /// @return 'true' if and only if X is a P-convex digital set.
      ///
      /// @pre X must not contain any duplicates.
      static
      bool isPConvex( const std::vector< Point >& X, bool safe )
      {
	return is0Convex( X, safe );
      }
      
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe is a not used parameter for dimension 1, but is
      /// kept for the meta-programming recursive definition of
      /// P-convexity.
      ///
      /// @pre X must not contain any duplicates.
      ///
      /// @return a measure that has value 1.0 when X is digitally convex, and
      /// less otherwise.
      static
      double convexityMeasure( std::vector< Point > X, bool safe )
      {
	(void) safe; //< not used in dimension 1.
	if ( X.empty() ) return 1.0;
	std::sort( X.begin(), X.end() );
	Integer nb =  Integer(X.back()[ 0 ]) - Integer(X.front()[ 0 ]) + Integer(1);
	return double( X.size() ) / double( nb );
      }
      
      /// @param X any range of lattice points (without duplicates)
      ///
      /// @param safe is a not used parameter for dimension 1, but is
      /// kept for the meta-programming recursive definition of
      /// P-convexity.
      ///
      /// @pre X must not contain any duplicates.
      ///
      /// @return a measure that has value 1.0 when X is P-convex (or
      /// equivalently fully convex), and less otherwise.
      static
      double fullConvexityMeasure( const std::vector< Point >& X, bool safe )
      {
	return convexityMeasure( X, safe );
      }
      
    };

  } // namespace detail 
  
    
  
  /////////////////////////////////////////////////////////////////////////////
  // template class PConvexity
  /**
     Description of template class 'PConvexity' <p> \brief Aim: A
     class to check if digital sets are P-convex. The P-convexity is
     defined as follows:
     A digital set X subset of \f$ \mathbb{Z}^d \f$ is P-convex iff
     - if d=1, then X must be an interval of integers, possibly empty
     - otherwise X must be 0-convex (digitally convex) and the projection of X along any dimension must be P-convex too.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TSpace an arbitrary model of digital space, i.e. see CSpace.
  */
  template < typename TSpace >
  class PConvexity
  {
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));

  public:
    typedef PConvexity<TSpace>       Self;
    typedef TSpace                   Space;
    typedef typename Space::Integer  Integer;
    typedef typename Space::Point    Point;
    
    static const Dimension dimension = Space::dimension;
    using RPConvexity = DGtal::detail::RecursivePConvexity< dimension, Integer >;

    // ----------------------- Standard services --------------------------------------
  public:
    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /// Destructor.
    ~PConvexity() = default;

    /// Main constructor.
    ///
    /// @param safe when 'true' performs convex hull computations with arbitrary
    /// precision integer (if available), otherwise chooses a
    /// compromise between speed and precision (int64_t).
    PConvexity( bool safe = false )
      : myRPC(), mySafe( safe )
    {}

    /// @}

    // ----------------------- Convexity services --------------------------------------
  public:
    /// @name Convexity services
    /// @{

    /// @param X any range of lattice points (without duplicates)
    ///
    /// @return 'true' if and only if X is a digitally convex set in the
    /// classic sense, i.e. \f$ Conv(X) \cap Z^d = X \f$.
    ///
    /// @pre X must not contain any duplicates.
    bool is0Convex( const std::vector< Point >& X ) const
    {
      return myRPC.is0Convex( X, mySafe );
    }

    /// @param X any range of lattice points (without duplicates)
    ///
    /// @return 'true' if and only if X is a P-convex digital set.
    ///
    /// @pre X must not contain any duplicates.
    bool isPConvex( const std::vector< Point >& X ) const
    {
      return myRPC.isPConvex( X, mySafe );
    }
    
    /// @}

    // ----------------------- Measure services --------------------------------------
  public:
    /// @name Measure services
    /// @{

    /// @param X any range of lattice points (without duplicates)
    ///
    /// @pre X must not contain any duplicates.
    ///
    /// @return a measure that has value 1.0 when X is digitally convex, and
    /// less otherwise.
    double convexityMeasure( const std::vector< Point >& X ) const
    {
      return myRPC.convexityMeasure( X, mySafe );
    }

    /// @param X any range of lattice points (without duplicates)
    ///
    /// @pre X must not contain any duplicates.
    ///
    /// @return a measure that has value 1.0 when X is P-convex (or
    /// equivalently fully convex), and less otherwise.
    double fullConvexityMeasure( const std::vector< Point >& X ) const
    {
      return myRPC.fullConvexityMeasure( X, mySafe );
    }

    /// @}

    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface services
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[PConvexity dim=" << dimension
	  << " safe=" << ( mySafe ? "True" : "False" )
	  << " #bits<int>=" << ( sizeof( Integer ) * 8 ) << "]";
    }

    /**
     * Checks the validity/consistency of the object. Only invalid if dimension < 1.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return dimension >= 1;
    }

    /// @}

    // ------------------------- Protected Datas ------------------------------
  protected:

    /// The recursive PConvexity object used to determine P-convexity.
    RPConvexity myRPC;
    
    /// when 'true' performs convex hull computations with arbitrary
    /// precision integer (if available), otherwise chooses a
    /// compromise between speed and precision (int64_t).
    bool mySafe;
    
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class PConvexity

  /// @name Functions related to PConvexity (output)
  /// @{

  /**
   * Overloads 'operator<<' for displaying objects of class 'PConvexity'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PConvexity' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out,
               const PConvexity<TKSpace> & object )
  {
    object.selfDisplay( out );
    return out;
  }

  /// @}

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PConvexity_h

#undef PConvexity_RECURSES
#endif // else defined(PConvexity_RECURSES)
