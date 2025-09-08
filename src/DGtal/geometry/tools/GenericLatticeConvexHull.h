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
 * @file GenericLatticeConvexHull.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/04
 *
 * Header file for module GenericLatticeConvexHull.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericLatticeConvexHull_RECURSES)
#error Recursive header files inclusion detected in GenericLatticeConvexHull.h
#else // defined(GenericLatticeConvexHull_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericLatticeConvexHull_RECURSES

#if !defined GenericLatticeConvexHull_h
/** Prevents repeated inclusion of headers. */
#define GenericLatticeConvexHull_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
#include "DGtal/geometry/tools/QuickHull.h"
#include "DGtal/geometry/tools/QuickHullKernels.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"

namespace DGtal
{
  // Forward declaration.
  template < Dimension dim,
             typename TCoordinateInteger,
             typename TInternalInteger >
  struct GenericLatticeConvexHull;

  namespace detail {
    template < Dimension dim,
               typename TCoordinateInteger,
               typename TInternalInteger,
               Dimension K >
    struct GenericLatticeConvexHullComputers
    {
      typedef ConvexHullIntegralKernel
      < K,TCoordinateInteger,TInternalInteger > Kernel;
      typedef detail::GenericLatticeConvexHullComputers
      < dim, TCoordinateInteger, TInternalInteger, K-1> LowerKernels;
      typedef std::size_t                Size;
      typedef typename Kernel::CoordinatePoint Point;
      typedef typename Point::Coordinate Integer;
      typedef QuickHull< Kernel >        QHull;
      typedef SpaceND< K, Integer >      Space;
      typedef BoundedLatticePolytope< Space > LatticePolytope;
      typedef GenericLatticeConvexHull< dim,
                                        TCoordinateInteger,
                                        TInternalInteger > Computer;
      static const Dimension dimension = K;

      GenericLatticeConvexHullComputers( Computer* ptrGenQHull )
        : ptr_gen_qhull( ptrGenQHull ), lower_kernels( ptrGenQHull ),
          hull( Kernel(), ptrGenQHull->debug_level )
      {
        clear();
      }

      /// Clears the object as if no computations have been made.
      void clear()
      {
        hull.clear(); 
        proj_points.clear();
        proj_dilation = 1;
        polytope.clear();
        lower_kernels.clear();
      }
      
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool remove_duplicates )
      {
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        hull.clear();
        polytope.clear();
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
            basis = Basis( Z, Basis::Type::ECHELON_REDUCED );
          }
        // Build projected points on affine basis
        proj_dilation  = basis.projectPoints( proj_points, X );
          
        // Compute convex hull using quickhull.
        bool ok_input = hull.setInput( proj_points, remove_duplicates );
        bool ok_hull  = hull.computeConvexHull( QHull::Status::VerticesCompleted );
        if ( ! ok_hull || ! ok_input )
          {
            trace.error() << "[GenericLatticeConvexHullComputers::compute]"
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
            ppoints[ i ] = Computer::OutputPoint::zero;
            for ( Dimension j = 0; j < Point::dimension; j++ )
              ppoints[ i ][ j ] = proj_points[ i ][ j ];
          }
        
        return ok_input && ok_hull;
      }

      bool makePolytope()
      {
        typedef typename LatticePolytope::Domain     Domain;
        typedef typename LatticePolytope::HalfSpace  PolytopeHalfSpace;
        typedef typename QHull::HalfSpace            ConvexHullHalfSpace;
        if ( ptr_gen_qhull->affine_dimension != dimension )
          { // This kernel is not adapted => go to lower dimension
            return lower_kernels.makePolytope();
          }
        // If polytope is already initialized returns.
        if ( polytope.nbHalfSpaces() > 0 ) return true;
        // Compute domain
        Point l = proj_points[ 0 ];
        Point u = proj_points[ 0 ];
        for ( std::size_t i = 1; i < proj_points.size(); i++ )
          {
            const auto& p = proj_points[ i ];
            l = l.inf( p );
            u = u.sup( p );
          }
        Domain domain( l, u );
        
        // Initialize polytope
        std::vector< ConvexHullHalfSpace > HS;
        std::vector< PolytopeHalfSpace >   PHS;
        hull.getFacetHalfSpaces( HS );
        PHS.reserve( HS.size() );
        for ( auto& H : HS ) {
          Point  N;
          Integer nu;
          for ( Dimension i = 0; i < dimension; ++i )
            N[ i ] = IntegerConverter< dimension, Integer >
              ::cast( H.internalNormal()[ i ] );
          nu = IntegerConverter< dimension, Integer >::cast( H.internalIntercept() );
          PHS.emplace_back( N, nu );
        }
        polytope = LatticePolytope( domain, PHS.cbegin(), PHS.cend(), false, true );
        return true;
      }

      /// Computes the number of integer points lying within the polytope.
      ///
      /// @return the number of integer points lying within the polytope,
      /// or -1 if their was a problem when computing the polytope.
      ///
      /// @note Quite fast: obtained by line intersection, see
      /// BoundedLatticePolytopeCounter
      Integer count()
      {
        if ( ptr_gen_qhull->affine_dimension != dimension )
          { // This kernel is not adapted => go to lower dimension
            return lower_kernels.count();
          }
        // If polytope is not initialized returns error.
        if ( polytope.nbHalfSpaces() == 0 ) return -1;
        return polytope.count();
      }
      
      Computer*            ptr_gen_qhull;
      LowerKernels         lower_kernels;
      QHull                hull; ///< the quick hull object that computes the convex hull
      std::vector< Point > proj_points;
      Integer              proj_dilation;
      LatticePolytope      polytope;
    };

    
    template < Dimension dim,
               typename TCoordinateInteger,
               typename TInternalInteger >
    struct GenericLatticeConvexHullComputers< dim, TCoordinateInteger, TInternalInteger, 1>
    {
      typedef ConvexHullIntegralKernel
      < 1,TCoordinateInteger,TInternalInteger > Kernel;
      typedef Kernel                     Type;
      typedef std::size_t                Size;
      typedef typename Kernel::CoordinatePoint Point;
      typedef typename Point::Coordinate Integer;
      typedef SpaceND<1, Integer>        Space;
      typedef GenericLatticeConvexHull< dim,
                                        TCoordinateInteger,
                                        TInternalInteger > Computer;
      static const Dimension             dimension = 1;

      GenericLatticeConvexHullComputers( Computer* ptrGenQHull )
        : ptr_gen_qhull( ptrGenQHull )
      {
        clear();
      }

      /// Clears the object as if no computations have been made.
      void clear()
      {
        proj_points.clear();
        proj_dilation = 1;
      }
      
      template <typename TInputPoint>
      bool compute( const std::vector< Size >& I,
                    const std::vector< TInputPoint >& X,
                    bool  )
      {
        // std::cout << "[GenericLatticeConvexHullComputers<K,1>::GenericLatticeConvexHullComputers]\n";
        typedef TInputPoint InputPoint;
        typedef AffineGeometry< InputPoint > Affine;
        typedef AffineBasis< InputPoint >    Basis;
        typedef std::size_t                  Index;

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
                v2p[ 0 ]   = 0;
                nb_in_hull = 1;
              }
            else
              {
                aff_dim = -1;
                points.clear();
                ppoints.clear();
                positions.clear();
                v2p.clear();
                nb_in_hull = 0;
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
            basis = Basis( Z, Basis::Type::ECHELON_REDUCED );
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
            ppoints[ i ] = Computer::OutputPoint::zero;
            ppoints[ i ][ 0 ] = proj_points[ i ][ 0 ];
          }
        v2p.resize( 2 );
        v2p[ 0 ] = left;
        v2p[ 1 ] = right;
        positions.resize( 2 );
        positions[ 0 ] = X[ v2p[ 0 ] ];
        positions[ 1 ] = X[ v2p[ 1 ] ];
        auto    dx  = Affine::transform( points[ v2p[ 1 ] ] - points[ v2p[ 0 ] ] );
        auto    sdx = Affine::simplifiedVector( dx );
        Integer n   = dx.normInfinity() / sdx.normInfinity();
        nb_in_hull  = n+1;
        return true;
      }

      bool makePolytope()
      {
        return true;
      }

      /// Computes the number of integer points lying within the polytope.
      ///
      /// @return the number of integer points lying within the polytope,
      /// or -1 if their was a problem when computing the polytope.
      Integer count() const
      {
        return nb_in_hull;
      }
      
      Computer*            ptr_gen_qhull;
      std::vector< Point > proj_points;
      Integer              proj_dilation;
      Integer              nb_in_hull;                   
    };
  }
  
  /////////////////////////////////////////////////////////////////////////////
  // template class GenericLatticeConvexHull

  /// Description of template class 'GenericLatticeConvexHull' <p> \brief Aim:
  /// Implements the quickhull algorithm by Barber et al. \cite barber1996,
  /// a famous arbitrary dimensional convex hull
  /// computation algorithm. It relies on dedicated geometric kernels
  /// for computing and comparing facet geometries.
  ///
  /// @tparam dim the dimension of the space of processed points.
  ///
  /// @tparam TCoordinateInteger the integer type that represents
  /// coordinates of lattice points, a model of concepts::CInteger.
  ///
  /// @tparam TInternalInteger the integer type that is used for
  /// internal computations of above/below plane tests, a model of
  /// concepts::CInteger. Must be at least as precise as
  ///TCoordinateInteger.
  template < Dimension dim,
             typename TCoordinateInteger  = DGtal::int64_t,
             typename TInternalInteger = DGtal::int64_t >
  struct GenericLatticeConvexHull
  {
    typedef ConvexHullIntegralKernel< dim,
                                      TCoordinateInteger,
                                      TInternalInteger > Kernel;
    typedef typename Kernel::CoordinatePoint     Point;
    typedef typename Kernel::CoordinateScalar    Integer;
    typedef typename Kernel::InternalScalar      InternalInteger;
    typedef Point                                OutputPoint;
    typedef std::size_t                Index;
    typedef std::size_t                Size;
    BOOST_STATIC_ASSERT(( Point::dimension == Point::dimension ));
    typedef std::vector< Index >       IndexRange;
    typedef detail::GenericLatticeConvexHullComputers
    < dim, TCoordinateInteger, TInternalInteger, dim > GenericComputers;

    static const Size  dimension  = Point::dimension;

    
    // ----------------------- standard services --------------------------
  public:
    /// @name Standard services (construction, initialization, accessors)
    /// @{
  
    /// Default constructor
    /// @param[in] K_ a kernel for computing facet geometries.
    /// @param[in] dbg the trace level, from 0 (no) to 3 (very verbose).
    GenericLatticeConvexHull( const Kernel& K_ = Kernel(), int dbg = 0 )
      : kernel( K_ ), debug_level( dbg ), generic_computers( this )
    {
      clear();
    }

    /// Clears the object as if no computations have been made.
    void clear()
    {
      affine_dimension  = -1;
      polytope_computed = false;
      generic_computers.clear();
    }
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
      typedef AffineGeometry< InputPoint > Affine;
      std::vector< Size > indices = Affine::affineSubset( input_points );
      bool ok = generic_computers.compute( indices, input_points, remove_duplicates );
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

    /// Computes the number of integer points lying within the polytope.
    ///
    /// @return the number of integer points lying within the polytope,
    /// or -1 if their was a problem when computing the polytope.
    ///
    /// @note Quite fast: obtained by line intersection, see
    /// BoundedLatticePolytopeCounter
    Integer count()
    {
      if ( ! polytope_computed )
        polytope_computed = generic_computers.makePolytope();
      if ( ! polytope_computed ) return -1; 
      return generic_computers.count();
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
      out << "[GenericLatticeConvexHull"
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
    GenericComputers generic_computers;

    /// the set of input points, indexed as in the input
    std::vector< OutputPoint > points;
    /// the set of projected input points, indexed as in the input
    std::vector< OutputPoint > projected_points;
    /// The affine dimension of the input set.
    int64_t                    affine_dimension;
    /// The positions of the vertices (a subset of the input points).
    std::vector< OutputPoint > positions;
    /// The range giving for each facet the indices of its vertices.
    std::vector< IndexRange >  facets;
    /// The indices of the vertices of the convex hull in the original set.
    IndexRange                 vertex2point;
    /// When 'true', the polytope has been computed.
    bool                       polytope_computed { false };
    /// @}
    
  };

  /// Overloads 'operator<<' for displaying objects of class 'GenericLatticeConvexHull'.
  ///
  /// @tparam dim the dimension of the space of processed points.
  ///
  /// @tparam TCoordinateInteger the integer type that represents
  /// coordinates of lattice points, a model of concepts::CInteger.
  ///
  /// @tparam TInternalInteger the integer type that is used for
  /// internal computations of above/below plane tests, a model of
  /// concepts::CInteger. Must be at least as precise as
  /// TCoordinateInteger.
  ///
  /// @param out the output stream where the object is written.
  /// @param object the object of class 'GenericLatticeConvexHull' to write.
  /// @return the output stream after the writing.
  template < Dimension dim,
             typename TCoordinateInteger,
             typename TInternalInteger >
  std::ostream&
  operator<< ( std::ostream & out,
               const GenericLatticeConvexHull< dim, TCoordinateInteger, TInternalInteger > & object )
  {
    object.selfDisplay( out );
    return out;
  }
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericLatticeConvexHull_h

#undef GenericLatticeConvexHull_RECURSES
#endif // else defined(GenericLatticeConvexHull_RECURSES)
