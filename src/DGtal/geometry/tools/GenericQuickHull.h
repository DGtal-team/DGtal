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
 * @file GenericQuickHull.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/04
 *
 * Header file for module GenericQuickHull.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericQuickHull_RECURSES)
#error Recursive header files inclusion detected in GenericQuickHull.h
#else // defined(GenericQuickHull_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericQuickHull_RECURSES

#if !defined GenericQuickHull_h
/** Prevents repeated inclusion of headers. */
#define GenericQuickHull_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
#include "DGtal/geometry/tools/QuickHull.h"
#include "DGtal/geometry/tools/QuickHullKernels.h"

namespace DGtal
{
  namespace detail {
    template <typename TKernel, Dimension K>
    struct GenericQuickHullKernels
    {
      typedef TKernel                    Kernel;
      typedef Kernel                     Type;
      typedef std::size_t                Size;
      typedef typename Kernel::LowerSelf LowerKernel;
      typedef typename Kernel::CoordinatePoint Point;
      typedef typename Point::Coordinate Scalar;
      typedef QuickHull< Kernel >        QHull;
      typedef typename QHull::Index      Index;
      typedef typename QHull::IndexRange IndexRange;
      static const Dimension             dimension = K;

      GenericQuickHullKernels( const Kernel& aKernel = Kernel() )
        : kernel( aKernel )
      {
        std::cout << "[GenericQuickHullKernels<K," << K << ">::GenericQuickHullKernels]\n";
      }
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool remove_duplicates )
      {
        std::cout << "[GenericQuickHullKernels<K," << K << ">::compute]"
                  << " #I=" << I.size() << " #X=" << X.size() << "\n";
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        typedef AffineGeometry< Point >      ProjAffine;
        typedef AffineBasis< Point >         ProjBasis;
        if ( (I.size()-1) != dimension )
          { // This kernel is not adapted => go to lower dimension
            return lower_kernels.compute( I, X, remove_duplicates );
          }
        // Build points of affine basis
        std::vector< InputPoint > Z( I.size() );
        for ( auto i = 0; i < I.size(); i++ )
          Z[ i ] = X[ I[ i ] ];
        // Build the affine basis spanning the convex hull affine space.
        Basis basis( Z, Basis::Type::SCALED_REDUCED );
        // Build projected points on affine basis
        proj_dilation  = basis.projectPoints( proj_points, X );
        // Compute convex hull using quickhull.
        QHull hull;
        bool ok_input = hull.setInput( proj_points, remove_duplicates );
        bool ok_hull  = hull.computeConvexHull( QHull::Status::VerticesCompleted );
        std::vector< Point > positions;
        hull.getVertexPositions( positions );
        std::vector< IndexRange > faces;
        hull.getFacetVertices( faces );
        std::cout << "Convex hull #V=" << positions.size()
                  << " #F=" << faces.size() << "\n";
        return ok_input && ok_hull;
      }
      
      Kernel kernel; ///< the quickhull computation kernel
      std::vector< Point > proj_points;
      Scalar               proj_dilation;
      GenericQuickHullKernels< LowerKernel, K-1> lower_kernels;
    };

    template <typename TKernel>
    struct GenericQuickHullKernels<TKernel, 1>
    {
      typedef TKernel                    Kernel;
      typedef Kernel                     Type;
      typedef std::size_t                Size;
      typedef typename Kernel::CoordinatePoint Point;
      typedef typename Point::Coordinate Scalar;
      typedef std::size_t                Index;
      typedef std::vector< Index >       IndexRange;
      static const Dimension             dimension = 1;

      GenericQuickHullKernels( const Kernel& aKernel = Kernel() )
        : kernel( aKernel )
      {
        std::cout << "[GenericQuickHullKernels<K,1>::GenericQuickHullKernels]\n";
      }
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool remove_duplicates )
      {
        std::cout << "[GenericQuickHullKernels<K,1>::GenericQuickHullKernels]\n";
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        typedef AffineGeometry< Point >      ProjAffine;
        typedef AffineBasis< Point >         ProjBasis;
        if ( (I.size()-1) != dimension )
          { // This kernel is not adapted => lower dimension is either
            // 0, ie. 1 point, or -1, ie. 0 points.
            if ( ! X.empty() )
              std::cout << "Convex hull dim=0 #V=" << 1
                        << " #F=" << 0 << "\n";
            else
              std::cout << "Convex hull dim=-1 #V=" << 0
                        << " #F=" << 0 << "\n";
            return true;
          }
        // Build points of affine basis
        std::vector< InputPoint > Z( I.size() );
        for ( auto i = 0; i < I.size(); i++ )
          Z[ i ] = X[ I[ i ] ];
        // Build the affine basis spanning the convex hull affine space.
        Basis basis( Z, Basis::Type::SCALED_REDUCED );
        // Build projected points on affine basis
        proj_dilation  = basis.projectPoints( proj_points, X );
        // Compute convex hull by looking at extremal points
        Index left  = 0;
        Index right = 0;
        for ( Index i = 1; i < proj_points.size(); i++ )
          {
            if ( proj_points[ i ][ 0 ] < proj_points[ left ][ 0 ] )
              left = i;
            else if ( proj_points[ i ][ 0 ] > proj_points[ right ][ 0 ] )
              right = i;
          }
        std::cout << "Convex hull #V=2=(" << left << "," << right << ")"
                  << " #F=" << 0 << "\n";
        return true;
      }

      Kernel               kernel;
      std::vector< Point > proj_points;
      Scalar               proj_dilation;
    };
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // template class GenericQuickHull

  /// Description of template class 'GenericQuickHull' <p> \brief Aim:
  /// Implements the quickhull algorithm by Barber et al. \cite barber1996,
  /// a famous arbitrary dimensional convex hull
  /// computation algorithm. It relies on dedicated geometric kernels
  /// for computing and comparing facet geometries.
  ///
  /// @tparam TKernel any type of GenericQuickHull kernel, like ConvexHullIntegralKernel.
  template < typename TKernel >
  struct GenericQuickHull
  {
    typedef TKernel                    Kernel;
    typedef typename Kernel::CoordinatePoint     Point;
    typedef typename Kernel::CoordinateVector    Vector;
    typedef typename Kernel::CoordinateScalar    Scalar;
    typedef typename Kernel::InternalScalar      InternalScalar;
    typedef std::size_t                Index;
    typedef std::size_t                Size;
    BOOST_STATIC_ASSERT(( Point::dimension == Vector::dimension ));
    typedef std::vector< Index >       IndexRange;
    typedef typename Kernel::HalfSpace HalfSpace;
    typedef typename Kernel::CombinatorialPlaneSimplex CombinatorialPlaneSimplex;
    static const Size  dimension  = Point::dimension;

    /// Label for points that are not assigned to any facet.
    enum { UNASSIGNED = (Index) -1 };

    
    // ----------------------- standard services --------------------------
  public:
    /// @name Standard services (construction, initialization, accessors)
    /// @{
  
    /// Default constructor
    /// @param[in] K_ a kernel for computing facet geometries.
    /// @param[in] dbg the trace level, from 0 (no) to 3 (very verbose).
    GenericQuickHull( const Kernel& K_ = Kernel(), int dbg = 0 )
      : generic_kernels( K_ ), debug_level( dbg )
    {}

    /// @}
    
    // -------------------------- Convex hull services ----------------------------
  public:

    /// @name convex hull services
    /// @{
    
    /// Sets the input data for the QuickHull convex hull algorithm,
    /// which is a range of points.
    ///
    /// @tparam InputPoint a model of point that is convertible to
    /// Point datatype.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the object is successfully initialized,
    /// status must be Status::InputInitialized, 'false' otherwise.
    template < typename InputPoint >
    bool compute( const std::vector< InputPoint >& input_points,
                  bool remove_duplicates = true )
    {
      // Determine affine dimension of set of input points.
      typedef AffineGeometry< Point > Affine;
      std::vector< Size > indices = Affine::affineSubset( input_points );
      return generic_kernels.compute( indices, input_points, remove_duplicates );
    }
    

    /// @}
    
    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[GenericGenericQuickHull]";
    }
  
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    }
    /// @}

    // ------------------------ public datas --------------------------
  public:
    /// @name public datas
    /// @{
  
  public:
    /// debug_level from 0:no to 2
    int debug_level; 
    /// the set of points, indexed as in the array.
    std::vector< Point > points;

    detail::GenericQuickHullKernels<TKernel, dimension> generic_kernels;
    
    /// @}
    
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'GenericQuickHull'.
   * @tparam TKernel any type of GenericQuickHull kernel, like ConvexHullIntegralKernel.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GenericQuickHull' to write.
   * @return the output stream after the writing.
   */
  template < typename TKernel >
  std::ostream&
  operator<< ( std::ostream & out,
               const GenericQuickHull< TKernel > & object )
  {
    object.selfDisplay( out );
    return out;
  }
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericQuickHull_h

#undef GenericQuickHull_RECURSES
#endif // else defined(GenericQuickHull_RECURSES)
