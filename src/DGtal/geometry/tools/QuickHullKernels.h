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
 * @file QuickHullKernels.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for module QuickHullKernels.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(QuickHullKernels_RECURSES)
#error Recursive header files inclusion detected in QuickHullKernels.h
#else // defined(QuickHullKernels_RECURSES)
/** Prevents recursive inclusion of headers. */
#define QuickHullKernels_RECURSES

#if !defined QuickHullKernels_h
/** Prevents repeated inclusion of headers. */
#define QuickHullKernels_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/math/linalg/SimpleMatrix.h"

namespace DGtal
{
  namespace detail {
    
    /// Compute the center of the bounding box containing the given points.
    /// @tparam Point any type of points
    /// @param[in] points a range of points
    /// @return the (approximate center of the given range.
    /// @note Complexity is O(n), if n is the size of \a points.
    template < typename Point >
    Point center( const std::vector< Point >& points )
    {
      if ( points.empty() ) return Point::zero;
      Point l = points[ 0 ];
      Point u = l;
      for ( const auto& p : points ) {
        l = l.inf( p );
        u = u.sup( p );
      }
      return Point( ( l + u ) / 2 );
    }

    /// Transform an input range of points into an output range with a
    /// conversion function, and possibly removes duplicates in the
    /// output range. Used as preprocessing of QuickHull algorithm.
    ///
    /// @tparam OutputValue any value that is LessThanComparable,
    /// Assignable, CopyConstructible, DefaultConstructible.
    ///
    /// @tparam ConversionFct is a functor from input values to OutputValue.
    ///
    /// @tparam ForwardIterator is a forward iterator on some input values.
    ///
    /// @param[out] output_values a range of points converted from the
    /// input range, with duplicates possibly removed according to
    /// \a remove_duplicates
    ///
    /// @param[out] input2output the surjective mapping between the
    /// input range and the output range used for computation.
    ///
    /// @param[out] output2input the injective mapping between the
    /// output range and the input range.
    ///
    /// @param[in] itb,ite a range of values.
    ///
    /// @param[in] F a function that may transform input values to output values
    ///
    /// @param[in] remove_duplicates when 'true' remove duplicate
    /// values, otherwise, `output_values.size()` is equal to the size
    /// of the range itb,ite.
    template < typename OutputValue,
               typename ForwardIterator,
               typename ConversionFct >
    void transform( std::vector< OutputValue >& output_values,
                    std::vector< std::size_t >& input2output,
                    std::vector< std::size_t >& output2input,
                    ForwardIterator itb, ForwardIterator ite,
                    const ConversionFct& F,
                    bool remove_duplicates )
    {
      typedef std::size_t Size;
      std::vector< OutputValue > input;
      while ( itb != ite ) {
        const auto ip = *itb++;
        input.push_back( F( ip ) );
      }
      if ( ! remove_duplicates ) {
        output_values.swap( input );
        input2output.resize( input.size() );
        output2input.resize( input.size() );
        for ( Size i = 0; i < input.size(); ++i )
          input2output[ i ] = output2input[ i ] = i;
      }
      else {
        output_values.clear();
        std::vector< std::size_t > i2c_sort( input.size() );
        input2output.resize( input.size() );
        for ( Size i = 0; i < input.size(); i++ ) i2c_sort[ i ] = i;
        // indirect sort
        std::sort( i2c_sort.begin(), i2c_sort.end(),
                   [&input] ( Size i, Size j ) { return input[ i ] < input[ j ]; } );
        output_values.resize( input.size() );
        output_values[ 0 ] = input[ i2c_sort[ 0 ] ];
        input2output[ i2c_sort[ 0 ] ] = 0;
        Size j = 0;
        for ( Size i = 1; i < input.size(); i++ ) {
          if ( input[ i2c_sort[ i-1 ] ] != input[ i2c_sort[ i ] ] )
            output_values[ ++j ] = input[ i2c_sort[ i ] ];
          input2output[ i2c_sort[ i ] ] = j;
        }
        output_values.resize( j+1 );
        output2input.resize( output_values.size() );
        for ( Size i = 0; i < input2output.size(); i++ )
          output2input[ input2output[ i ] ] = i;
      }      
    }

  } // namespace detail {


  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullCommonKernel
  /**
     Description of template class 'ConvexHullCommonKernel' <p> \brief
     Aim: the common part of all geometric kernels for computing the
     convex hull or Delaunay triangulation of a range of points.

     @tparam dim the dimension of the space that is used for computing
     the convex hull (either the same as input points for convex hull,
     or one more than input points for Delaunay triangulation)
  */
  template < Dimension dim >
  struct ConvexHullCommonKernel {
    typedef DGtal::PointVector< dim, DGtal::int64_t > Point;
    typedef DGtal::PointVector< dim, DGtal::int64_t > Vector;
    typedef int64_t                    Scalar;
    typedef std::size_t                Size;
    typedef Size                       Index;
    typedef std::vector< Index >       IndexRange;
    typedef std::array< Index, dim >   CombinatorialPlaneSimplex;
    static const Dimension dimension = dim;

    class HalfSpace {
      friend class ConvexHullCommonKernel< dim >;
      Vector N; ///< the normal vector
      Scalar c; ///< the intercept
      HalfSpace( const Vector& aN, const Scalar aC )
        : N( aN ), c( aC ) {}
    public:
      HalfSpace() = default;
      const Vector& internalNormal() const    { return N; }
      Scalar internalIntercept() const { return c; }
    };
    
    /// Default constructor.
    ConvexHullCommonKernel() = default;

    /// Computes an halfspace from dimension points specified by \a
    /// simplex with vertices in a range \a vpoints of Point. It is
    /// oriented such that the point of index \a idx_below is included
    /// in the half-space (i.e. below).
    ///
    /// @param[in] vpoints a range of points over which the simplex is defined.
    /// @param[in] simplex a range of dimension indices of points defining an hyperplane.
    /// @param[in] idx_below the index of a p-oint that is below the hyperplane.
    ///
    /// @return the corresponding halfspace (has null normal vector if
    /// simplex was not full dimensional)
    HalfSpace
    compute( const std::vector< Point >& vpoints,
             const CombinatorialPlaneSimplex& simplex,
             Index idx_below )
    {
      HalfSpace hs = compute( vpoints, simplex );
      if ( hs.N != Vector::zero ) 
        {
          const Scalar nu = hs.N.dot( vpoints[ idx_below ] );
          if ( nu > hs.c ) { hs.N = -hs.N; hs.c = -hs.c; }
        }
      return hs;
    }

    /// Computes an halfspace from dimension points specified by \a
    /// simplex with vertices in a range \a vpoints of
    /// Point. Orientation is induced by the order of the points. If
    /// the simplex is degenrated, the half-space is invalid and has
    /// null normal.
    ///
    /// @param[in] vpoints a range of points over which the simplex is defined.
    /// @param[in] simplex a range of dimension indices of points defining an hyperplane.
    ///
    /// @return the corresponding halfspace (has null normal vector if
    /// simplex was not full dimensional)
    HalfSpace
    compute( const std::vector< Point >& vpoints,
             const CombinatorialPlaneSimplex& simplex )
    {
      typedef DGtal::SimpleMatrix< Scalar, dimension, dimension > Matrix;
      Matrix A;
      Vector N;
      Scalar c;
      for ( Dimension i = 1; i < dimension; i++ )
        for ( Dimension j = 0; j < dimension; j++ )
          A.setComponent( i-1, j,
                          vpoints[ simplex[ i ] ][ j ]
                          - vpoints[ simplex[ 0 ] ][ j ] );
      for ( Dimension j = 0; j < dimension; j++ )
        N[ j ] = A.cofactor( dimension-1, j );
      c = N.dot( vpoints[ simplex[ 0 ] ] );
      return HalfSpace { N, c };
    }
    
    /// @param H the half-space
    /// @return the normal to this facet.
    Vector normal( const HalfSpace& H ) const
    {
      return H.N;
    }

    /// @param H the half-space
    /// @return the intercept of this facet.
    Scalar intercept( const HalfSpace& H ) const
    {
      return H.c;
    }
    
    /// Equivalent of the dot product of the normals of the half-spaces.
    ///
    /// @param H1 an half-space
    /// @param H2 an half-space
    ///
    /// @return a positive scalar if both half-spaces points to to the
    /// same hemisphere, a negative scalar if they point to opposite
    /// hemispheres, and zero if they are orthogonal.
    Scalar dot( const HalfSpace& H1, const HalfSpace& H2 ) const
    {
      return H1.N.dot( H2.N );
    }

    /// @param H1 an half-space
    /// @param H2 an half-space
    ///
    /// @return 'true' if the half-spaces have the smae members.
    ///
    /// @note two half-spaces may be not equal but may represent the
    /// same set of points. For instance `H1={{1,0},3}` and
    /// `H1={{2,0},6}`.
    bool equal( const HalfSpace& H1, const HalfSpace& H2 ) const
    {
      return H1.c == H2.c && H1.N == H2.N;
    }
    
    /// @param H the half-space
    /// @param p any point
    /// @return the (signed) height of \a p wrt this plane.
    Scalar height( const HalfSpace& H, const Point& p ) const
    { return H.N.dot( p ) - H.c; }

    /// @param H the half-space
    /// @param p any point
    /// @return the volume of the vectors spanned by the simplex and this point.
    Scalar volume( const HalfSpace& H, const Point& p ) const
    { Scalar v = height( H, p ); return v < 0 ? -v : v; }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p is strictly above this plane (so in direction N ).
    bool above( const HalfSpace& H, const Point& p ) const
    { return height( H, p ) > 0; }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p is above or lies on this plane (so in direction N ).
    bool aboveOrOn( const HalfSpace& H, const Point& p ) const
    { return height( H, p ) >= 0; }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p lies on this plane.
    bool on( const HalfSpace& H, const Point& p ) const
    { return height( H, p ) == 0; } 
    
    
  }; //   template < Dimension dim >  struct ConvexHullIntegralKernel {


  
  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullIntegralKernel
  /**
     Description of template class 'ConvexHullIntegralKernel' <p>
     \brief Aim: a geometric kernel to compute the convex hull of
     digital points with integer-only arithmetic.

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct ConvexHullIntegralKernel : public ConvexHullCommonKernel< dim >
  {
    typedef ConvexHullCommonKernel< dim > Base;
    // inheriting types
    using typename Base::Point;
    using typename Base::Vector;
    using typename Base::Scalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;
    
    /// Default constructor.
    ConvexHullIntegralKernel() = default;

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return false; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// corresponds to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      (void) hs; // unused parameter
      return false;
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< Point >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> Point
        {
          Point p;
          for ( Dimension i = 0; i < dimension; i++ )
            p[ i ] = static_cast< Scalar >( input[ i ] );
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const Point& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension; k++ )
        out_p[ k ] = p[ k ];
    }

  }; //   template < Dimension dim >  struct ConvexHullIntegralKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class DelaunayIntegralKernel
  /**
     Description of template class 'DelaunayIntegralKernel' <p> \brief
     Aim: a geometric kernel to compute the Delaunay triangulation of
     digital points with integer-only arithmetic. It casts lattice
     point into a higher dimensional space and computes its convex
     hull. Facets pointing toward the bottom form the simplices of the
     Delaunay triangulation.

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct DelaunayIntegralKernel : public ConvexHullCommonKernel< dim+1 >
  {
    typedef ConvexHullCommonKernel< dim+1 > Base;
    // inheriting types
    using typename Base::Point;
    using typename Base::Vector;
    using typename Base::Scalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// Default constructor.
    DelaunayIntegralKernel() = default;

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return true; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// correspond to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      return hs.internalNormal()[ dimension - 1 ] >= 0;
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points. This kernel transforms
    /// `dim`-dimensional into `dim+1`-dimensional points, where the
    /// last coordinate lies on a paraboloid. This is the classical
    /// way for computing a Delaunay triangulation as the convex hull
    /// of the points onto a paraboloid of higher dimension.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< Point >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> Point
        {
          Point p;
          Scalar z = 0;
          for ( Dimension i = 0; i < dimension-1; i++ ) {
            const Scalar x = static_cast< Scalar >( input[ i ] );
            p[ i ]   = x;
            z       += x*x;
          }
          p[ dimension-1 ] = z;
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const Point& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension-1; k++ )
        out_p[ k ] = p[ k ];
    }
        
  }; //   template < Dimension dim >  struct DelaunayIntegralKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullRationalKernel
  /**
     Description of template class 'ConvexHullRationalKernel' <p>
     \brief Aim: a geometric kernel to compute the convex hull of
     floating points with integer-only arithmetic. Floating points are
     approximated with rational points with fixed precision (a given
     number of bits). All remaining computations are exact, as long as
     there is no overflow.

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct ConvexHullRationalKernel : public ConvexHullCommonKernel< dim >
  {
    typedef ConvexHullCommonKernel< dim > Base;
    // inheriting types
    using typename Base::Point;
    using typename Base::Vector;
    using typename Base::Scalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// The precision as the common denominator for all rational points.
    double precision;
    
    /// Constructor with specified precision
    ///
    /// @param[in] aPrecision the chosen precision as the common
    /// denominator of all rationals (by defaut, 1024).
    ConvexHullRationalKernel( double aPrecision = 1024. )
      : precision( aPrecision ) {}

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return false; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// corresponds to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      (void) hs; // unused parameter      
      return false;
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< Point >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> Point
        {
          Point p;
          for ( Dimension i = 0; i < dimension; i++ )
            p[ i ] = static_cast< Scalar >( round( input[ i ] * precision ) );
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const Point& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension; k++ )
        out_p[ k ] = ( (double) p[ k ] ) / precision;
    }
    
    
  }; //   template < Dimension dim >  struct ConvexHullRationalKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class DelaunayRationalKernel
  /**
     Description of template class 'DelaunayRationalKernel' <p> \brief
     Aim: a geometric kernel to compute the Delaunay triangulation of
     a range of floating points with integer-only arithmetic. Floating
     points are approximated with rational points with fixed precision
     (a given number of bits), which are cast in a higher dimensional
     space. All remaining computations are exact, as long as there is
     no overflow.

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct DelaunayRationalKernel : public ConvexHullCommonKernel< dim+1 >
  {
    typedef ConvexHullCommonKernel< dim+1 > Base;
    // inheriting types
    using typename Base::Point;
    using typename Base::Vector;
    using typename Base::Scalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// The precision as the common denominator for all rational points.
    double precision;
    
    /// Constructor with specified precision
    ///
    /// @param[in] aPrecision the chosen precision as the common
    /// denominator of all rationals (by defaut, 1024).
    DelaunayRationalKernel( double aPrecision = 1024. )
      : precision( aPrecision ) {}

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return true; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// correspond to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      return hs.internalNormal()[ dimension - 1 ] >= 0;
    }

    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points. This kernel transforms
    /// `dim`-dimensional into `dim+1`-dimensional points, where the
    /// last coordinate lies on a paraboloid. This is the classical
    /// way for computing a Delaunay triangulation as the convex hull
    /// of the points onto a paraboloid of higher dimension.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< Point >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> Point
        {
          Point p;
          Scalar z = 0;
          for ( Dimension i = 0; i < dimension - 1; i++ ) {
            const Scalar x = static_cast< Scalar >( round( input[ i ] * precision ) );
            p[ i ] = x;
            z       += x*x;
          }
          p[ dimension-1 ] = z;
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const Point& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension - 1; k++ )
        out_p[ k ] = ( (double) p[ k ] ) / precision;
    }
    
  }; //   template < Dimension dim >  struct DelaunayRationalKernel {

  
  
} // namespace DGtal {

#endif // !defined QuickHullKernels_h

#undef QuickHullKernels_RECURSES
#endif // else defined(QuickHullKernels_RECURSES)
