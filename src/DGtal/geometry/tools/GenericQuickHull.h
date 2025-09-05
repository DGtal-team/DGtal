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
  // Forward declaration.
  template < typename TKernel > struct GenericQuickHull;

  namespace detail {
    template <typename TParentKernel, typename TKernel, Dimension K>
    struct GenericQuickHullKernels
    {
      typedef TParentKernel              ParentKernel;
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

      GenericQuickHullKernels( GenericQuickHull< ParentKernel >* ptrGenQHull = nullptr )
        : ptr_gen_qhull( ptrGenQHull ), lower_kernels( ptrGenQHull )
      {
        // std::cout << "[GenericQuickHullKernels<K," << K << ">::GenericQuickHullKernels]"
        //           << " dim=" << ptr_gen_qhull->dimension << "\n";
      }
      
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool remove_duplicates )
      {
        // std::cout << "[GenericQuickHullKernels<K," << K << ">::compute]"
        //           << " #I=" << I.size() << " #X=" << X.size() << "\n";
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        typedef AffineGeometry< Point >      ProjAffine;
        typedef AffineBasis< Point >         ProjBasis;
        if ( (I.size()-1) != dimension )
          { // This kernel is not adapted => go to lower dimension
            return lower_kernels.compute( I, X, remove_duplicates );
          }
        ptr_gen_qhull->affine_dimension = dimension;
        auto& points    = ptr_gen_qhull->points;
        auto& ppoints   = ptr_gen_qhull->projected_points;
        auto& positions = ptr_gen_qhull->positions;
        auto& v2p       = ptr_gen_qhull->vertex2point;
        auto& facets    = ptr_gen_qhull->facets;
        Basis basis;
        if ( dimension != ptr_gen_qhull->dimension )
          {
            // Build points of affine basis
            std::vector< InputPoint > Z( I.size() );
            for ( auto i = 0; i < I.size(); i++ )
              Z[ i ] = X[ I[ i ] ];
            // Build the affine basis spanning the convex hull affine space.
            basis = Basis( Z, Basis::Type::SCALED_REDUCED );
          }
        // Build projected points on affine basis
        proj_dilation  = basis.projectPoints( proj_points, X );
          
        // Compute convex hull using quickhull.
        QHull hull( Kernel(), ptr_gen_qhull->debug_level );
        bool ok_input = hull.setInput( proj_points, remove_duplicates );
        bool ok_hull  = hull.computeConvexHull( QHull::Status::VerticesCompleted );
        if ( ! ok_hull || ! ok_input )
          {
            trace.error() << "[GenericQuickHullKernels::compute]"
                          << " Error in quick hull computation.\n"
                          << "qhull=" << hull << "\n";
            return false;
          }
        /// Copy back convex hull in initial space.
        points.resize( X.size() );
        for ( Size i = 0; i < points.size(); i++ )
          points[ i ] = Affine::transform( X[ i ] );
        hull.getVertex2Point( v2p );
        hull.getFacetVertices( facets );
        positions.resize( v2p.size() );
        for ( Size i = 0; i < positions.size(); i++ )
          positions[ i ] = X[ v2p[ i ] ];
        ppoints.resize( proj_points.size() );
        for ( Size i = 0; i < ppoints.size(); i++ )
          {
            ppoints[ i ] = ParentKernel::CoordinatePoint::zero;
            for ( Dimension j = 0; j < Point::dimension; j++ )
              ppoints[ i ][ j ] = proj_points[ i ][ j ];
          }
        
        return ok_input && ok_hull;
      }

      GenericQuickHull< ParentKernel >* ptr_gen_qhull;
      GenericQuickHullKernels< ParentKernel, LowerKernel, K-1> lower_kernels;
      std::vector< Point > proj_points;
      Scalar               proj_dilation;
    };

    
    template <typename TParentKernel, typename TKernel>
    struct GenericQuickHullKernels<TParentKernel, TKernel, 1>
    {
      typedef TParentKernel              ParentKernel;
      typedef TKernel                    Kernel;
      typedef Kernel                     Type;
      typedef std::size_t                Size;
      typedef typename Kernel::CoordinatePoint Point;
      typedef typename Point::Coordinate Scalar;
      typedef std::size_t                Index;
      typedef std::vector< Index >       IndexRange;
      static const Dimension             dimension = 1;

      GenericQuickHullKernels( GenericQuickHull< ParentKernel >* ptrGenQHull = nullptr )
        : ptr_gen_qhull( ptrGenQHull )
      {
        // std::cout << "[GenericQuickHullKernels<K,1>::GenericQuickHullKernels]"
        //           << " dim=" << ptr_gen_qhull->dimension << "\n";
      }
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool remove_duplicates )
      {
        // std::cout << "[GenericQuickHullKernels<K,1>::GenericQuickHullKernels]\n";
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        typedef AffineGeometry< Point >      ProjAffine;
        typedef AffineBasis< Point >         ProjBasis;

        auto& aff_dim   = ptr_gen_qhull->affine_dimension;
        auto& points    = ptr_gen_qhull->points;
        auto& ppoints   = ptr_gen_qhull->projected_points;
        auto& positions = ptr_gen_qhull->positions;
        auto& v2p       = ptr_gen_qhull->vertex2point;
        auto& facets    = ptr_gen_qhull->facets;
        facets.clear(); // no facets
        
        if ( (I.size()-1) != dimension )
          { // This kernel is not adapted => lower dimension is either
            // 0, ie. 1 point, or -1, ie. 0 points.
            if ( ! X.empty() )
              {
                aff_dim = 0;
                points.resize( X.size() );
                for ( Size i = 0; i < points.size(); i++ )
                  points[ i ] = Affine::transform( X[ i ] );
                ppoints = points;
                positions.resize( 1 );
                positions[ 0 ] = X[ 0 ];
                v2p.resize( 1 );
                v2p[ 0 ] = 0;
              }
            else
              {
                aff_dim = -1;
                points.clear();
                ppoints.clear();
                positions.clear();
                v2p.clear();
              }
            return true;
          }
        // Generic 1D case.
        aff_dim = dimension;
        Basis basis;
        if ( dimension != ptr_gen_qhull->dimension )
          {
            // Build points of affine basis
            std::vector< InputPoint > Z( I.size() );
            for ( auto i = 0; i < I.size(); i++ )
              Z[ i ] = X[ I[ i ] ];
            // Build the affine basis spanning the convex hull affine space.
            basis = Basis( Z, Basis::Type::SCALED_REDUCED );
          }
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
        points.resize( X.size() );
        for ( Size i = 0; i < points.size(); i++ )
          points[ i ] = Affine::transform( X[ i ] );
        ppoints.resize( proj_points.size() );
        for ( Size i = 0; i < ppoints.size(); i++ )
          {
            ppoints[ i ] = ParentKernel::CoordinatePoint::zero;
            ppoints[ i ][ 0 ] = proj_points[ i ][ 0 ];
          }
        v2p.resize( 2 );
        v2p[ 0 ] = left;
        v2p[ 1 ] = right;
        positions.resize( 2 );
        positions[ 0 ] = X[ v2p[ 0 ] ];
        positions[ 1 ] = X[ v2p[ 1 ] ];
        return true;
      }

      GenericQuickHull< ParentKernel >* ptr_gen_qhull;
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
      : kernel( K_ ), generic_kernels( this ), debug_level( dbg )
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
      bool ok = generic_kernels.compute( indices, input_points, remove_duplicates );
      if ( ( ! ok ) || ( debug_level >= 1 ) )
        {
          std::cout << "Generic Convex hull #V=" << positions.size()
                    << " #F=" << facets.size() << "\n";
          for ( Size i = 0; i < facets.size(); i++ )
            {
              std::cout << "F_" << i << " = (";
              for ( auto v : facets[ i ] ) std::cout << " " << v;
              std::cout << " )\n";
            }
          for ( Size i = 0; i < positions.size(); i++ )
            std::cout << "V_" << i
                      << " pi(x)=" << projected_points[ vertex2point[ i ] ]
                      << " -> x=" << positions[ i ] << "\n";
        }
      return ok;
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
      out << "[GenericQuickHull"
          << " dim=" << dimension
          << " #in=" << points.size()
          << " aff_dim=" << affine_dimension
          << " #V=" << positions.size()
          << " #F=" << facets.size()
          << "]";
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
    /// The main quickhull kernel that is used for convex hull computations.
    Kernel kernel;
    /// debug_level from 0:no to 2:verbose
    int debug_level; 
    /// The delegate computation kernel that can take care of all kind
    /// of convex hulls, full dimensional or degenerated.
    detail::GenericQuickHullKernels<TKernel, TKernel, dimension> generic_kernels;

    /// the set of input points, indexed as in the input
    std::vector< Point >      points;
    /// the set of projected input points, indexed as in the input
    std::vector< Point >      projected_points;
    /// The affine dimension of the input set.
    int64_t                   affine_dimension;
    /// The positions of the vertices (a subset of the input points).
    std::vector< Point >      positions;
    /// The range giving for each facet the indices of its vertices.
    std::vector< IndexRange > facets;
    /// The indices of the vertices of the convex hull in the original set.
    IndexRange                vertex2point;
    
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
