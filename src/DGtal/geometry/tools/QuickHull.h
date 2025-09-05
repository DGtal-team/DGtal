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
 * @file QuickHull.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for module QuickHull.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(QuickHull_RECURSES)
#error Recursive header files inclusion detected in QuickHull.h
#else // defined(QuickHull_RECURSES)
/** Prevents recursive inclusion of headers. */
#define QuickHull_RECURSES

#if !defined QuickHull_h
/** Prevents repeated inclusion of headers. */
#define QuickHull_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clock.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/QuickHullKernels.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class QuickHull

  /// Description of template class 'QuickHull' <p> \brief Aim:
  /// Implements the quickhull algorithm by Barber et al. \cite barber1996,
  /// a famous arbitrary dimensional convex hull
  /// computation algorithm. It relies on dedicated geometric kernels
  /// for computing and comparing facet geometries.
  ///
  /// You can use it to build convex hulls of points with integral
  /// coordinate (using kernel ConvexHullIntegralKernel), convex hulls
  /// with rational coordinates (using kernel
  /// ConvexHullRationalKernel) or to build Delaunay triangulations
  /// (using kernels DelaunayIntegralKernel and
  /// DelaunayRationalKernel).
  ///
  /// @see \ref moduleQuickHull
  ///
  /// Below is a complete example that computes the convex hull of
  /// points randomly defined in a ball, builds a 3D mesh out of it and
  /// output it as an OBJ file.
  ///
  /// @code
  /// #include "DGtal/base/Common.h"
  /// #include "DGtal/kernel/PointVector.h"
  /// #include "DGtal/shapes/SurfaceMesh.h"
  /// #include "DGtal/io/writers/SurfaceMeshWriter.h"
  /// #include "QuickHull.h"
  /// 
  /// using namespace DGtal::Z3i;
  /// int main( int argc, char* argv[] )
  /// {
  ///   int nb = argc > 1 ? atoi( argv[ 1 ] ) : 100; // nb points
  ///   int R  = argc > 2 ? atoi( argv[ 2 ] ) : 10;  // x-radius of ellipsoid
  ///   // (0) typedefs
  ///   typedef DGtal::ConvexHullIntegralKernel< 3 > Kernel3D;
  ///   typedef DGtal::QuickHull< Kernel3D >         QuickHull3D;
  ///   // (1) create range of random points in ball
  ///   std::vector< Point > V;
  ///   const auto R2 = R/// R;
  ///   for ( int i = 0; i < nb; ) {
  ///     Point p( rand() % (2*R+1) - R, rand() % (2*R+1) - R, rand() % (2*R+1) - R );
  ///     if ( p.squaredNorm() < R2 ) { V.push_back( p ); i++; }
  ///   }
  ///   // (2) compute convex hull
  ///   QuickHull3D hull;
  ///   hull.setInput( V );
  ///   hull.computeConvexHull();
  ///   std::cout << "#points=" << hull.nbPoints()
  ///             << " #vertices=" << hull.nbVertices()
  ///             << " #facets=" << hull.nbFacets() << std::endl;
  ///   // (3) build mesh
  ///   std::vector< RealPoint > positions;
  ///   hull.getVertexPositions( positions );
  ///   std::vector< std::vector< std::size_t > > facets;
  ///   hull.getFacetVertices( facets );
  ///   typedef DGtal::SurfaceMesh< RealPoint, RealVector> SMesh;
  ///   SMesh mesh( positions.cbegin(), positions.cend(), facets.cbegin(), facets.cend() );
  ///   // (4) output result as OBJ file
  ///   std::ofstream out( "qhull.obj" );
  ///   DGtal::SurfaceMeshWriter< Space::RealPoint, Space::RealVector >
  ///     ::writeOBJ( out, mesh );
  ///   out.close();
  ///   return 0;
  /// } 
  /// @endcode
  ///
  /// @note In opposition with the usual QuickHull implementation, this
  /// class uses a kernel that can be chosen in order to provide exact
  /// computations. This is the case for lattice points.
  ///
  /// @note In opposition with CGAL `3D convex hull` package, or with
  /// the arbitrary dimensional `dD Triangulation` package, this
  /// algorithm does not build a simplicial convex hull. Facets may not
  /// be trangles or simplices in higher dimensions.
  ///
  /// @note This version is generally more than twice faster than CGAL
  /// convex_hull_3 for the usual CGAL kernels Cartesian and
  /// Exact_predicates_inexact_constructions_kernel.
  ///
  /// @note However this implementation is not tailored for incremental
  /// dynamic convex hull computations.
  ///
  /// @tparam TKernel any type of QuickHull kernel, like ConvexHullIntegralKernel.
  template < typename TKernel >
  struct QuickHull
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

    /// A facet is d-1 dimensional convex cell lying on the boundary
    /// of a full dimensional convex set. Its supporting hyperplane
    /// defines an half-space touching and enclosing the convex set.
    struct Facet {
      HalfSpace  H; ///< the facet geometry
      IndexRange neighbors;   ///< neighbor facets
      IndexRange outside_set; ///< outside set, i.e. points above this facet
      IndexRange on_set;      ///< on set, i.e. points on this facet, *sorted*
      Index below; ///< index of point that is below this facet

      Facet() = default;
      Facet( const Facet& ) = default;
      Facet( Facet&& ) = default;
      Facet& operator=( Facet&& ) = default;
      Facet& operator=( const Facet& ) = default;
      Facet( const HalfSpace& aH, Index b )
        : H( aH ), below( b ) {}

      void clear()
      {
        H = HalfSpace();
        neighbors.clear();
        outside_set.clear();
        on_set.clear();
        below = UNASSIGNED;
      }
      void addPointOn( Index p )
      {
        const auto it = std::find( on_set.cbegin(), on_set.cend(), p );
        if ( it == on_set.cend() ) on_set.push_back( p );
      }
      void display( std::ostream& out ) const
      {
        const auto N = H.internalNormal(); 
        out << "[Facet iN=(" << N[0];
        for ( Dimension i = 1; i < N.dimension; i++ ) out << "," << N[ i ];
        out << ") c=" << H.internalIntercept() << " b=" << below << " n={";
        for ( auto&& n : neighbors ) out << " " << n;
        out << " } #out=" << outside_set.size();
        out << " on={";
        for ( auto&& n : on_set ) out << " " << n;
        out << " }]" << std::endl;
      }

      void addNeighbor( Index n )
      {
        const auto it = std::find( neighbors.cbegin(), neighbors.cend(), n );
        if ( it == neighbors.cend() ) neighbors.push_back( n );
      }
      void subNeighbor( Index n )
      {
        auto it = std::find( neighbors.begin(), neighbors.end(), n );
        if ( it != neighbors.end() ) {
          std::swap( *it, neighbors.back() );
          neighbors.pop_back();
        }
      }
      void swap( Facet& other )
      {
        if ( this != &other ) {
          std::swap( H, other.H );
          neighbors.swap  ( other.neighbors );
          outside_set.swap( other.outside_set );
          on_set.swap     ( other.on_set );
          std::swap( below, other.below );
        }
      }
      Size variableMemory() const
      {
        Size M;
        M += neighbors.capacity()   * sizeof( Index );
        M += outside_set.capacity() * sizeof( Index );
        M += on_set.capacity()      * sizeof( Index );
        return M;
      }
    };

    /// A ridge for point p is a pair of facets, such that p is visible
    /// from first facet, but not from second facet.
    typedef std::pair< Index, Index > Ridge;

    /// Represents the status of a QuickHull object.
    enum class Status {
      Uninitialized     =  0, ///< QuickHull is empty and has just been instantiated.
      InputInitialized  =  1, ///< A range of input points has been given to QuickHull.
      SimplexCompleted  =  2, ///< An initial full-dimensional simplex has been found. QuickHull core algorithm can start.
      FacetsCompleted   =  3, ///< All facets of the convex hull are identified.
      VerticesCompleted =  4, ///< All vertices of the convex hull are determined.
      AllCompleted      =  5, ///< Same as VerticesCompleted.
      NotFullDimensional= 10, ///< Error: the initial simplex is not full dimensional.
      InvalidRidge      = 11, ///< Error: some ridge is not consistent (probably integer overflow).
      InvalidConvexHull = 12  ///< Error: the convex hull does not contain all its points (probably integer overflow).
    };

    // ----------------------- standard services --------------------------
  public:
    /// @name Standard services (construction, initialization, accessors)
    /// @{
  
    /// Default constructor
    /// @param[in] K_ a kernel for computing facet geometries.
    /// @param[in] dbg the trace level, from 0 (no) to 3 (very verbose).
    QuickHull( const Kernel& K_ = Kernel(), int dbg = 0 )
      : kernel( K_ ), debug_level( dbg ), myStatus( Status::Uninitialized )
    {}

    /// @return the current status of this object, in Uninitialized,
    /// NotFullDimensional, InputInitialized, Completed, InvalidRidge,
    /// InvalidConvexHull
    Status status() const
    { return myStatus; }
  
    /// Clears the object.
    void clear()
    {
      myStatus = Status::Uninitialized;
      points.clear();
      processed_points.clear();
      input2comp.clear();
      comp2input.clear();
      assignment.clear();
      facets.clear();
      deleted_facets.clear();
      p2v.clear();
      v2p.clear();
      timings.clear();
    }

    
    /// @return an estimation of the current memory occupation of this object.
    Size memory() const
    {
      // int debug_level;
      Size M = sizeof( kernel ) + sizeof( int );
      // std::vector< Point > points;    
      M += sizeof( std::vector< Point > )
        + points.capacity() * sizeof( Point );
      M += sizeof( std::vector< Index > )
        + processed_points.capacity() * sizeof( Index );
      M += sizeof( std::vector< Index > )
        + input2comp.capacity() * sizeof( Index );
      M += sizeof( std::vector< Index > )
        + comp2input.capacity() * sizeof( Index );
      // std::vector< Index > assignment;
      M += sizeof( std::vector< Index > )
        + assignment.capacity() * sizeof( Index );
      // std::vector< Facet > facets;
      M += sizeof( std::vector< Facet > )
        + facets.capacity() * sizeof( Facet );
      for ( const auto& f : facets ) M += f.variableMemory();
      // std::set< Index > deleted_facets;
      M += sizeof( std::set< Index > )
        + deleted_facets.size() * ( sizeof( Index ) + 2*sizeof(Index*) );
      // IndexRange p2v;
      M += sizeof( std::vector< Index > )
        + p2v.capacity() * sizeof( Index );
      // IndexRange v2p;
      M += sizeof( std::vector< Index > )
        + v2p.capacity() * sizeof( Index );
      // std::vector< double > timings;
      M += sizeof( std::vector< double > )
        + timings.capacity() * sizeof( double );
      return M;
    }

    /// @return the number of points used as input.
    Size nbPoints() const 
    { return points.size(); }

    /// @return the number of facets of the convex hull.
    /// @pre status() >= Status::FacetsCompleted
    Size nbFacets() const 
    {
      ASSERT( status() >= Status::FacetsCompleted
              && status() <= Status::AllCompleted );
      return facets.size();
    }

    /// @return the number of vertices of the convex hull.
    /// @pre status() >= Status::VerticesCompleted
    Size nbVertices() const 
    {
      ASSERT( status() >= Status::VerticesCompleted
              && status() <= Status::AllCompleted );
      return v2p.size();
    }

    /// @return the number of finite facets of the convex hull.
    /// @pre status() >= Status::VerticesCompleted
    Size nbFiniteFacets() const 
    {
      ASSERT( status() >= Status::VerticesCompleted
              && status() <= Status::AllCompleted );
      return nb_finite_facets;
    }

    /// @return the number of infinite facets of the convex hull.
    /// @pre status() >= Status::VerticesCompleted
    Size nbInfiniteFacets() const 
    {
      ASSERT( status() >= Status::VerticesCompleted
              && status() <= Status::AllCompleted );
      return nb_infinite_facets;
    }
    
    /// @}
    // -------------------------- Convex hull services ----------------------------
  public:
    /// @name Initialization services
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
    bool setInput( const std::vector< InputPoint >& input_points,
                   bool remove_duplicates = true )
    {
      Clock tic;
      tic.startClock();
      clear();
      timings.clear();
      kernel.makeInput( points, input2comp,  comp2input,
                        input_points, remove_duplicates );
      timings.push_back( tic.stopClock() );
      if ( points.size() <= dimension ) {
        myStatus = Status::NotFullDimensional;
        return false;
      }
      myStatus = Status::InputInitialized;
      return true;
    }

    /// Sets the initial full dimensional simplex
    ///
    /// @pre status() must be Status::InputInitialized
    ///
    /// @param full_splx a `dimension+1`-simplex specified as indices in the
    /// vector of input point.
    ///
    /// @return 'true' iff this initial simplex is full dimensional,
    /// the object has then the status SimplexCompleted, otherwise
    /// returns 'false' and the status is NotFullDimensional.
    bool setInitialSimplex( const IndexRange& full_splx )
    {
      if ( status() != Status::InputInitialized ) return false;
      if ( full_splx.size() != dimension + 1 )
        {
          trace.error() << "[QuickHull::setInitialSimplex]"
                        << " not a full dimensional simplex" << std::endl;
          myStatus = Status::NotFullDimensional;
          return false;
        }          
      CombinatorialPlaneSimplex splx;
      for ( Index j = 0; j < dimension; ++j )
        splx[ j ] = full_splx[ j ];
      const auto      H = kernel.compute( points, splx, full_splx.back() );
      const auto volume = kernel.volume( H, points[ full_splx.back() ] );
      if ( volume > 0 )
        return computeSimplexConfiguration( full_splx );
      myStatus = Status::NotFullDimensional;
      return false;
    }

    /// @}
    // -------------------------- Convex hull services ----------------------------
  public:
    /// @name Convex hull services
    /// @{
    
    /// Computes the convex hull of the given range of points until a
    /// specified \a target:
    ///
    /// - Status::SimplexCompleted: an initial full dimensional
    ///   simplex is determined.
    ///
    /// - Status::FacetsCompleted: all the facets of the convex hull
    ///   are extracted (core of quickhull). You can stop here if you
    ///   only need an H-representation of the output polytope.
    ///
    /// - Status::VerticesCompleted: all the vertices of the hull are
    ///   determined and are consistently ordered on each facet. You
    ///   need to stop here if you need a V-representation of the
    ///   output polytope.
    ///
    /// @pre status() must be at least Status::InputInitialized
    ///
    /// @param[in] target the computation target in Status::SimplexCompleted,
    /// Status::FacetsCompleted, Status::VerticesCompleted.
    ///
    /// @return 'true' if the computation target has been successfully
    /// achieved, 'false' if the achieved status is not the one
    /// specified.
    bool computeConvexHull( Status target = Status::VerticesCompleted )
    {
      if ( target < Status::InputInitialized || target > Status::AllCompleted )
        return false;
      Clock tic;
      if ( status() == Status::InputInitialized )
        { // Initialization
          tic.startClock();
          bool ok1 = computeInitialSimplex();
          timings.push_back( tic.stopClock() );
          if ( ! ok1 )              return false;
          if ( status() == target ) return true;
        }
      if ( status() == Status::SimplexCompleted )
        { // Computes facets
          tic.startClock();
          bool ok2 = computeFacets();
          timings.push_back( tic.stopClock() );
          if ( ! ok2 )              return false;
          if ( status() == target ) return true;
        }
      if ( status() == Status::FacetsCompleted )
        { // Computes vertices
          tic.startClock();
          bool ok3 = computeVertices();
          timings.push_back( tic.stopClock() );
          if ( ! ok3 )              return false;
          if ( status() == target ) return true;
        }
      if ( target == Status::AllCompleted
           && status() == Status::VerticesCompleted )
        { // for now, Status::VerticesCompleted and
          // Status::AllCompleted are the same.
          myStatus = Status::AllCompleted;
          return true;
        }
      return false;
    }
  
    /// Computes the initial full dimensional simplex from the input data.
    ///
    /// @return 'true' iff the input data contains d+1 points in general
    /// position, the object has then the status SimplexCompleted, otherwise
    /// returns 'false' and the status is NotFullDimensional.
    bool computeInitialSimplex()
    {
      const auto full_simplex = pickInitialSimplex();
      if ( full_simplex.empty() ) {
        myStatus = Status::NotFullDimensional;
        return false;
      }
      return computeSimplexConfiguration( full_simplex );
    }

    /// Computes the facets of the convex hull using Quickhull
    /// algorithm. If everything went well, the status is
    /// Status::FacetsCompleted afterwards.
    ///
    /// @pre the status shoud be Status::SimplexCompleted
    /// (computeInitialSimplex should have been called).
    ///
    /// @return 'true' except if the status is not Initialized when
    /// called.
    bool computeFacets()
    {
      if ( status() != Status::SimplexCompleted ) return false;
      std::queue< Index > Q;
      for ( Index fi = 0; fi < facets.size(); ++fi )
        Q.push( fi );
      Index n = 0;
      while ( processFacet( Q ) ) {
        if ( debug_level >= 1 )
          trace.info() << "---- Iteration " << n++ << " #Q=" << Q.size() << std::endl;
      }
      cleanFacets();
      if ( debug_level >= 2 ) {
        trace.info() << ".... #facets=" << facets.size()
                  << " #deleted=" << deleted_facets.size() << std::endl;
      }
      myStatus = Status::FacetsCompleted;
      return true;
    }

    /// Computes the vertices of the convex hull once the facets have
    /// been computed. It computes for each facet its vertices and
    /// reorder them so that, taken in order, their orientation
    /// matches the orientation of the facet. If everything went well, the
    /// status is Status::VerticesCompleted afterwards.
    ///
    /// @pre the status shoud be Status::FacetsCompleted (computeFacets should have
    /// been called).
    ///
    /// @return 'true' except if the status is not FacetsCompleted when called.
    bool computeVertices()
    {
      static const int MAX_NB_VPF = 10 * dimension;
      if ( status() != Status::FacetsCompleted ) return false;

      // Renumber infinite facets in case of Delaunay triangulation computation.
      renumberInfiniteFacets();
      
      // Builds the maps v2p: vertex -> point, and p2v : point -> vertex.
      facet_counter = IndexRange( MAX_NB_VPF, 0 );
      v2p.clear();
      p2v.resize( points.size() );
      std::vector< IndexRange > p2f( points.size() );
      for ( Index f = 0; f < facets.size(); ++f ) {
        for ( auto&& p : facets[ f ].on_set ) p2f[ p ].push_back( f ); 
      }

      // vertices belong to at least d facets
      Index v = 0;
      for ( Index p = 0; p < points.size(); ++p ) {
        const auto nbf = p2f[ p ].size();
        facet_counter[ std::min( (int) nbf, MAX_NB_VPF-1 ) ] += 1;
        if ( nbf >= dimension ) {
          v2p.push_back( p );
          p2v[ p ] = v++;
        }
        else p2v[ p ] = UNASSIGNED;
      }

      // Display debug informations
      if ( debug_level >= 1 ) {
        trace.info() << "#vertices=" << v2p.size() << " #facets=" << facets.size()
                  << std::endl;
        trace.info() << "#inc_facets/point= ";
        for ( auto n : facet_counter ) trace.info() << n << " ";
        trace.info() << std::endl;
      }
      myStatus = Status::VerticesCompleted;
      return true;
    }

    
    /// @}
    // -------------------------- Output services ----------------------------
  public:
    /// @name Output services
    /// @{

    /// Gets the positions of the convex hull vertices.
    ///
    /// @tparam OutputPoint a model of point such that the Point
    /// datatype is convertible to it.
    ///
    /// @param[out] vertex_positions the range of vertex positions.
    ///
    /// @return 'true' if the convex hull was computed before and
    /// status() was Status::VerticesCompleted or
    /// Status::AllCompleted, 'false' otherwise.
    template < typename OutputPoint >
    bool getVertexPositions( std::vector< OutputPoint >& vertex_positions )
    {
      vertex_positions.clear();
      if ( ! ( status() >= Status::VerticesCompleted
               && status() <= Status::AllCompleted ) ) return false;
      vertex_positions.resize( v2p.size() );
      for ( Index i = 0; i < v2p.size(); i++ ) {
        kernel.convertPointTo( points[ v2p[ i ] ], vertex_positions[ i ] );
      }
      return true;
    }

    /// Gets for each vertex its index in the input range of points.
    ///
    /// @param[out] vertex_to_point the range giving for each vertex
    /// its index in the input range of points.
    ///
    /// @return 'true' if the convex hull was computed before and
    /// status() was Status::VerticesCompleted or
    /// Status::AllCompleted, 'false' otherwise.
    bool getVertex2Point( IndexRange& vertex_to_point )
    {
      vertex_to_point.clear();
      if ( ! ( status() >= Status::VerticesCompleted
               && status() <= Status::AllCompleted ) ) return false;
      vertex_to_point = v2p;
      return true;
    }

    /// Gets for each point its index in the output range of vertices
    /// (or UNASSIGNED if the point is not part of the convex hull)
    ///
    /// @param[out] point_to_vertex the range giving for each point
    /// its index in the output range of vertices (or UNASSIGNED if
    /// the point is not part of the convex hull)
    ///
    /// @return 'true' if the convex hull was computed before and
    /// status() was Status::VerticesCompleted or
    /// Status::AllCompleted, 'false' otherwise.
    bool getPoint2Vertex( IndexRange& point_to_vertex )
    {
      point_to_vertex.clear();
      if ( ! ( status() >= Status::VerticesCompleted
               && status() <= Status::AllCompleted ) ) return false;
      point_to_vertex = p2v;
      return true;
    }
    
    /// @param[out] facet_vertices the range giving for each facet the
    /// indices of its vertices.
    ///
    /// @return 'true' if the convex hull was computed before and
    /// status() was Status::VerticesCompleted or
    /// Status::AllCompleted, 'false' otherwise.
    bool getFacetVertices( std::vector< IndexRange >& facet_vertices ) const
    {
      facet_vertices.clear();
      if ( ! ( status() >= Status::VerticesCompleted
               && status() <= Status::AllCompleted ) ) return false;
      facet_vertices.reserve( nbFacets() );
      for ( Index f = 0; f < nbFacets(); ++f )  {
        IndexRange ofacet = orientedFacetPoints( f );
        for ( auto& v : ofacet ) v = p2v[ v ];
        facet_vertices.push_back( ofacet );
      }
      return true;
    }

    /// This methods return the halfspaces corresponding to each
    /// facet. It is useful to build a polytope.
    ///
    /// @param[out] facet_halfspaces the range giving for each facet
    /// its halfspace as a pair (normal, intercept).
    ///
    /// @return 'true' if the convex hull was computed before and
    /// status() was Status::FacetsCompleted,
    /// Status::VerticesCompleted or Status::AllCompleted, 'false'
    /// otherwise.
    bool getFacetHalfSpaces( std::vector< HalfSpace >& facet_halfspaces )
    {
      facet_halfspaces.clear();
      if ( ! ( status() >= Status::FacetsCompleted
               && status() <= Status::AllCompleted ) ) return false;
      facet_halfspaces.reserve( nbFacets() );
      for ( Index f = 0; f < nbFacets(); ++f )  {
        facet_halfspaces.push_back( facets[ f ].H );
      }
      return true;
    }
    
    
    /// @}
    // -------------------------- Check hull services ----------------------------
  public:
    /// @name Check hull services
    /// @{
    
    /// Global validity check of the convex hull after processing.
    ///
    /// @note Be careful, this function is much slower than computing
    /// the convex hull. It can take a long time since its complexity
    /// is \f$ O(n f ) \f$, where \a n is the number of input points
    /// and \a f the number of facets.
    bool check()
    {
      bool ok = true;
      if ( status() < Status::FacetsCompleted || status() > Status::AllCompleted ) {
        trace.warning() << "[Quickhull::check] invalid status="
                        << (int)status() << std::endl;
        ok = false;
      }
      if ( processed_points.size() != points.size() ) {
        trace.warning() << "[Quickhull::check] not all points processed: "
                        << processed_points.size() << " / " <<  points.size()
                        << std::endl;
        ok = false;
      }
      if ( ! checkHull() ) {
        trace.warning() << "[Quickhull::check] Check hull is invalid. "
                        << std::endl;
        ok = false;
      }
      if ( ! checkFacets() ) {
        trace.warning() << "[Quickhull::check] Check facets is invalid. "
                        << std::endl;
        ok = false;
      }
      return ok;
    }

    /// @return 'true' if all facets are consistent with their neighors
    bool checkFacets()
    {
      Size   nb = 0;
      Size nbok = 0;
      for ( Index f = 0; f < facets.size(); ++f )
        if ( deleted_facets.count( f ) == 0 ) {
          bool ok = checkFacet( f );
          nbok   += ok ? 1 : 0;
          nb     += 1;
        }
      return nb == nbok;
    }

    /// @return 'true' iff the current set of facets encloses all the points
    ///
    /// @note Be careful, this function is much slower than computing
    /// the convex hull. It can take a long time since its complexity
    /// is \f$ O(n f ) \f$, where \a n is the number of input points
    /// and \a f the number of facets.
    bool checkHull()
    {
      Index nbok = 0;
      // for ( Index v = 0; v < points.size(); ++v ) {
      for ( auto v : processed_points ) {
        bool ok = true;
        for ( Index f = 0; f < facets.size(); ++f )
          if ( deleted_facets.count( f ) == 0 ) {
            if ( above( facets[ f ], points[ v ] ) ) {
              ok = false;
              trace.error() << "- bad vertex " << v << " " << points[ v ]
                            << " dist="
                            << ( kernel.height( facets[ f ].H, points[ v ] ) )
                            << std::endl;
              break;
            }
          }
        nbok += ok ? 1 : 0;
      }
      if ( debug_level >= 2 ) {
        trace.info() << nbok << "/"
                  << processed_points.size() << " vertices inside convex hull."
                  << std::endl;
      }
      if ( nbok != processed_points.size() ) myStatus = Status::InvalidConvexHull;
      return nbok == processed_points.size();
    }
  
    /// @}
    // ------------------------ public datas --------------------------
  public:
    /// @name public datas
    /// @{
  
  public:
    /// Kernel that is duplicated for computing facet geometries.
    mutable Kernel kernel;
    /// debug_level from 0:no to 2
    int debug_level; 
    /// the set of points, indexed as in the array.
    std::vector< Point > points;
    /// the surjective mapping between the input range and the output
    /// range used for convex hull computation.
    IndexRange input2comp;
    /// the injective mapping between the convex hull point range and
    /// the input range.
    IndexRange comp2input;
    /// Points already processed (and within the convex hull).
    IndexRange processed_points;
    /// assignment of points to facets
    std::vector< Index > assignment;
    /// the current set of facets.
    std::vector< Facet > facets;
    /// set of deleted facets
    std::set< Index > deleted_facets;
    /// point index -> vertex index (or UNASSIGNED)
    IndexRange p2v;
    /// vertex index -> point index
    IndexRange v2p;
    /// Number of finite facets
    Size nb_finite_facets;
    /// Number of infinite facets (!= 0 only for specific kernels)
    Size nb_infinite_facets;
    /// Timings of the different phases: 0: init, 1: facets, 2: vertices.
    std::vector< double > timings;
    /// Counts the number of facets with a given number of vertices.
    std::vector< Size > facet_counter;
    
    /// @}
    // ------------------------ protected datas --------------------------
  protected:
    /// @name protected datas
    /// @{
  
    /// The status of the object: Uninitialized, InputInitialized,
    /// SimplexCompleted, FacetsCompleted, VerticesCompleted,
    /// InvalidRidge, InvalidConvexHull, NotFullDimensional
    Status myStatus;
  
    /// @}
    // --------------------- protected services --------------------------
  protected:
    /// @name protected services
    /// @{

    /// @param F any valid facet
    /// @param p any point
    /// @return the height of p wrt F (0: on, >0: above ).
    InternalScalar height( const Facet& F, const Point& p ) const
    { return kernel.height( F.H, p ); }

    /// @param F any valid facet
    /// @param p any point
    /// @return 'true' iff p is above F.
    bool above( const Facet& F, const Point& p ) const
    { return kernel.above( F.H, p ); }

    /// @param F any valid facet
    /// @param p any point
    /// @return 'true' iff p is above F or on F.
    bool aboveOrOn( const Facet& F, const Point& p ) const
    { return kernel.aboveOrOn( F.H, p ); }
    
    /// @param F any valid facet
    /// @param p any point
    /// @return 'true' iff p lies on F.
    bool on( const Facet& F, const Point& p ) const
    { return kernel.on( F.H, p ); }
    
    /// Cleans and renumber the facets so that no one belongs to
    /// deleted_facets.
    void cleanFacets()
    {
      if ( deleted_facets.empty() ) return;
      IndexRange renumbering( facets.size() );
      Index i = 0;
      Index j = 0;
      for ( auto& l : renumbering ) {
        if ( ! deleted_facets.count( j ) ) l = i++;
        else l = UNASSIGNED;
        j++;
      }
      const Index nf = facets.size() - deleted_facets.size();
      deleted_facets.clear();
      for ( Index f = 0; f < facets.size(); f++ )
        if ( ( renumbering[ f ] != UNASSIGNED ) && ( f != renumbering[ f ] ) )
          facets[ renumbering[ f ] ] = facets[ f ];
      facets.resize( nf );
      for ( auto& F : facets ) {
        for ( auto& N : F.neighbors ) {
          if ( renumbering[ N ] == UNASSIGNED )
            trace.error() << "Invalid deleted neighboring facet." << std::endl;
          else N = renumbering[ N ];
        }
      }
    }

    /// Determine infinite facets and renumber them so that finite
    /// facets come first and infinite facets come after.
    void renumberInfiniteFacets()
    {
      nb_finite_facets   = facets.size();
      nb_infinite_facets = 0;
      if ( ! kernel.hasInfiniteFacets()  ) return;
      IndexRange renumbering( facets.size() );
      Index i = 0;
      Index k = facets.size();
      Index j = 0;
      for ( auto& l : renumbering ) {
        if ( ! kernel.isHalfSpaceFacetInfinite( facets[ j ].H ) ) l = i++;
        else l = --k;
        j++;
      }
      if ( i != k )
        trace.error() << "[Quickhull::renumberInfiniteFacets]"
                      << " Error renumbering infinite facets "
                      << " up finite=" << i << " low infinite=" << k << std::endl;
      std::vector< Facet > new_facets( facets.size() );
      for ( Index f = 0; f < facets.size(); f++ )
        new_facets[ renumbering[ f ] ].swap( facets[ f ] );
      facets.swap( new_facets );
      for ( auto& F : facets ) {
        for ( auto& N : F.neighbors ) {
          N = renumbering[ N ];
        }
      }
      // Assign correct number of facets.
      nb_finite_facets   = i;
      nb_infinite_facets = facets.size() - nb_finite_facets;
    }
    
  
    /// Process top facet in queue Q as in Quickhull algorithm, and
    /// updates the queue accordingly (top facet poped, new facets
    /// pushed).
    ///
    /// @param[inout] Q a queue of facet index to process. which is
    /// updated by the method.
    ///
    /// @return 'true' if there is still work to do, 'false' when finished
    ///
    /// @note Core of Quickhull algorithm.
    bool processFacet( std::queue< Index >& Q )
    {
      // If Q empty, we are done
      if ( Q.empty() ) return false;
      Index F = Q.front();
      Q.pop();
      // If F is already deleted, proceed to next in queue.
      if ( deleted_facets.count( F ) ) return true;
      // Take car of current facet.
      const Facet& facet = facets[ F ];
      if ( debug_level >= 3 ) {
        trace.info() << "---------------------------------------------" << std::endl;
        trace.info() << "---- ACTIVE FACETS---------------------------" << std::endl;
        bool ok = true;
        for ( Index i = 0; i < facets.size(); i++ )
          if ( ! deleted_facets.count( i ) ) {
            trace.info() << "- facet " << i << " ";
            facets[ i ].display( trace.info() );
            ok = ok && checkFacet( i );
          }
        if ( ! ok ) { // should not happen.
          myStatus = Status::InvalidConvexHull;
          return false;
        }
      }
      if ( debug_level >= 2 ) {
        trace.info() << "---------------------------------------------" << std::endl;
        trace.info() << "Processing facet " << F << " ";
        facet.display( trace.info() );
      }
      if ( facet.outside_set.empty() ) return true;
      // Selects furthest vertex
      Index  furthest_v = facet.outside_set[ 0 ];
      auto   furthest_h = height( facet, points[ furthest_v ] );
      for ( Index v = 1; v < facet.outside_set.size(); v++ ) {
        auto h = height( facet, points[ facet.outside_set[ v ] ] );
        if ( h > furthest_h ) {
          furthest_h = h;
          furthest_v = facet.outside_set[ v ];
        }
      }
      const Point& p = points[ furthest_v ];
      // Extracts Visible facets V and Horizon Ridges H
      std::vector< Index > V;   // visible facets
      std::set< Index >    M;   // marked facets (are in E or were in E)
      std::queue< Index >  E;   // queue to extract visible facets
      std::vector< Ridge > H;   // visible facets
      E.push  ( F );
      M.insert( F );
      while ( ! E.empty() ) {
        Index G = E.front(); E.pop();
        V.push_back( G );
        for ( auto& N : facets[ G ].neighbors ) {
          if ( aboveOrOn( facets[ N ], p ) ) {
            if ( M.count( N ) ) continue;
            E.push( N );
          } else {
            H.push_back( { G, N } );
          }
          M.insert( N );
        }
      } // while ( ! E.empty() ) 
      if ( debug_level >= 1 ) {
        trace.info() << "#Visible=" << V.size() << " #Horizon=" << H.size()
                  << " furthest_v=" << furthest_v << std::endl;
      }
      // Create new facets
      IndexRange new_facets;
      // For each ridge R in H
      for ( Index i = 0; i < H.size(); i++ )
        {
          // Create a new facet from R and p
          IndexRange ridge = pointsOnRidge( H[ i ] );
          if ( debug_level >= 3 ) {
            trace.info() << "Ridge (" << H[i].first << "," << H[i].second << ") = {";
            for ( auto&& r : ridge ) trace.info() << " " << r;
            trace.info() << " } furthest_v=" << furthest_v << std::endl;
          }
          IndexRange base( 1 + ridge.size() );
          Index j     = 0;
          base[ j++ ] = furthest_v;
          for ( auto&& v : ridge ) base[ j++ ] = v;
          if ( j < dimension ) {
            trace.error() << "Bad ridge between " << std::endl
                      << "- facet " << H[i].first << " ";
            facets[ H[i].first ].display( trace.error() );
            trace.error() << "- facet " << H[i].second << " ";
            facets[ H[i].second ].display( trace.error() );
          }                    
          Index nf = newFacet();
          new_facets.push_back( nf );
          facets[ nf ] = makeFacet( base, facets[ H[i].first ].below );
          facets[ nf ].on_set = IndexRange { base.cbegin(), base.cend() };
          std::sort( facets[ nf ].on_set.begin(), facets[ nf ].on_set.end() );
          makeNeighbors( nf, H[ i ].second );
          if ( debug_level >= 3 ) {
            trace.info() << "* New facet " << nf << " ";
            facets[ nf ].display( trace.info() );
          }
          // Checks that the facet is not parallel to another in the Horizon
          for ( Index k = 0; k < new_facets.size() - 1; k++ )
            if ( areFacetsParallel( new_facets[ k ], nf ) ) {
              if ( debug_level >= 1 ) {
                trace.info() << "Facets " << new_facets[ k ] << " and " << nf
                          << " are parallel => merge." << std::endl;
              }
              mergeParallelFacets( new_facets[ k ], nf );
              new_facets.pop_back();
              deleteFacet( nf );
              if ( debug_level >= 3 ) {
                facets[ new_facets[ k ] ].display( trace.info() );
              }
            }
        }
      // For each new facet 
      for ( Index i = 0; i < new_facets.size(); i++ )
        { // link the new facet to its neighbors
          for ( Index j = i + 1; j < new_facets.size(); j++ )
            {
              const Index nfi = new_facets[ i ];
              const Index nfj = new_facets[ j ];
              if ( areFacetsNeighbor( nfi, nfj ) )
                makeNeighbors( nfi, nfj );
            }
        }
      // Extracts all outside points from visible facets V
      IndexRange outside_pts;
      for ( auto&& vf : V ) {
        for ( auto&& v : facets[ vf ].outside_set ) {
          if ( v != furthest_v ) {
            outside_pts.push_back( v );
            assignment[ v ] = UNASSIGNED;
          }
        }
      }
      // For each new facet F'
      for ( Index i = 0; i < new_facets.size(); i++ ) {
        Facet& Fp = facets[ new_facets[ i ] ];
        Index max_j = outside_pts.size();
        for ( Index j = 0; j < max_j; ) {
          const Index v = outside_pts[ j ];
          if ( above( Fp, points[ v ] ) ) {
            Fp.outside_set.push_back( v );
            assignment[ v ]  = new_facets[ i ];
            outside_pts[ j ] = outside_pts.back();
            outside_pts.pop_back();
            max_j--;
          } else j++;
        }
        if ( debug_level >= 3 ) {
          trace.info() << "- New facet " << new_facets[ i ] << " ";
          Fp.display( trace.info() );
        }
      }
      // Update processed points
      processed_points.push_back( furthest_v );
      for ( auto v : outside_pts ) processed_points.push_back( v );
      
      // Delete the facets in V
      for ( auto&& v : V ) {
        if ( debug_level >= 2 ) {
          trace.info() << "Delete facet " << v << " ";
          facets[ v ].display( trace.info() );
        }
        deleteFacet( v );
      }

      // Add new facets to queue
      for ( Index i = 0; i < new_facets.size(); i++ )
        Q.push( new_facets[ i ] );
      if ( debug_level >= 1 ) {
        trace.info() << "#facets=" << facets.size()
                  << " #deleted=" << deleted_facets.size() << std::endl;
      }

      // Checks that everything is ok.
      if ( debug_level >= 1 ) {
        trace.info() << "[CHECK INVARIANT] " << processed_points.size()
                     << " / " << points.size() << " points processed." << std::endl;
        bool okh = checkHull(); 
        if ( ! okh )
          trace.error() << "[computeFacet] Invalid convex hull" << std::endl;
        bool okf = checkFacets();
        if ( ! okf )
          trace.error() << "[computeFacet] Invalid facets" << std::endl;
        if ( ! ( okh && okf ) ) myStatus = Status::InvalidConvexHull;
      }

      return status() == Status::SimplexCompleted;
    }
  
    /// @return true if the facet is valid
    bool checkFacet( Index f ) const
    {
      const Facet& F = facets[ f ];
      bool ok = F.on_set.size() >= dimension;
      for ( auto v : F.on_set )
        if ( ! on( F, points[ v ] ) ) {
          trace.error() << "[QuickHull::checkFacet( " << f
                    << ") Invalid 'on' vertex " << v << std::endl;
          ok = false;
        }
      if ( F.neighbors.size() < dimension ) {
        trace.error() << "[QuickHull::checkFacet( " << f
                  << ") Not enough neighbors " << F.neighbors.size() << std::endl;
        ok = false;
      }
      for ( auto nf : F.neighbors )
        if ( ! areFacetsNeighbor( f, nf ) ) {
          trace.error() << "[QuickHull::checkFacet( " << f
                    << ") Invalid neighbor " << nf << std::endl;
          ok = false;
        }
      if ( aboveOrOn( F, points[ F.below ] ) ) {
        trace.error() << "[QuickHull::checkFacet( " << f
                  << ") Bad below point " << F.below << std::endl;
        ok = false;
      }
      for ( auto ov : F.outside_set )
        if ( ! above( F, points[ ov ] ) ) {
          trace.error() << "[QuickHull::checkFacet( " << f
                    << ") Bad outside point " << ov << std::endl;
          ok = false;
        }
      for ( auto && v : F.on_set ) {
        Size      n = 0;
        for ( auto&& N : facets[ f ].neighbors )
          if ( on( facets[ N ], points[ v ] ) ) n += 1;
        if ( n < dimension-1 ) {
          trace.error() << "[QuickHull::checkFacet( " << f << ") 'on' point " << v
                    << " is a vertex of " << n << " facets "
                    << "(should be >= " << dimension-1 << ")" << std::endl;
          ok = false;
        }
      }
      return ok;
    }
  
    /// @return an unused facet index
    Index newFacet()
    {
      // SLightly faster to postpone deletion of intermediate facets.
      const Index f = facets.size();
      facets.push_back( Facet() );
      return f;
    }

    /// Deletes the given facet \a f.
    /// @param f a valid facet index
    void deleteFacet( Index f )
    {
      for ( auto n : facets[ f ].neighbors )
        facets[ n ].subNeighbor( f );
      deleted_facets.insert( f );
      facets[ f ].clear();
    }

    /// Makes two distinct facets \a if1 and \a if2 as neighbors
    /// @param if1 a valid facet index
    /// @param if2 a valid facet index
    void makeNeighbors( const Index if1, const Index if2 )
    {
      facets[ if1 ].addNeighbor( if2 );
      facets[ if2 ].addNeighbor( if1 );
    }
    
    /// Makes two distinct facets \a if1 and \a if2 no more neighbors
    /// @param if1 a valid facet index
    /// @param if2 a valid facet index
    void unmakeNeighbors( const Index if1, const Index if2 )
    {
      facets[ if1 ].subNeighbor( if2 );
      facets[ if2 ].subNeighbor( if1 );
    }

    /// Merge the two facets and return the index of the second one,
    /// which is the one deleted.
    ///
    /// @param if1 a valid facet index
    /// @param if2 a valid facet index
    /// @pre the two facets should be distinct and parallel
    Index mergeParallelFacets( const Index if1, const Index if2 )
    {
      Facet& f1 = facets[ if1 ];
      Facet& f2 = facets[ if2 ];
      std::copy( f2.outside_set.cbegin(), f2.outside_set.cend(),
                 std::back_inserter( f1.outside_set ) );
      IndexRange merge_idx;
      std::set_union( f1.on_set.cbegin(), f1.on_set.cend(),
                      f2.on_set.cbegin(), f2.on_set.cend(),
                      std::back_inserter( merge_idx ) );
      f1.on_set.swap( merge_idx );
      for ( auto && nf2 : f2.neighbors ) {
        if ( nf2 == if1 ) continue;
        facets[ nf2 ].subNeighbor( if2 );
        makeNeighbors( if1, nf2 );
      }
      return if2;
    }
  
    /// Checks if two distinct facets are parallel (i.e. should be merged).
    /// @param if1 a valid facet index
    /// @param if2 a valid facet index
    /// @return 'true' if the two facets are parallel.
    bool areFacetsParallel( const Index if1, const Index if2 ) const
    {
      ASSERT( if1 != if2 );
      const Facet& f1 = facets[ if1 ];
      const Facet& f2 = facets[ if2 ];
      if ( kernel.equal( f1.H, f2.H ) ) return true;
      // Need to check if one N is a multiple of the other.
      for ( auto&& v : f1.on_set )
        if ( ! on( f2, points[ v ] ) ) return false;
      return true;
    }
  
    /// Checks if two facets are neighbors by looking at the points on their boundary.
    ///
    /// @param if1 a valid facet index
    /// @param if2 a valid facet index
    ///
    /// @return 'true' if the two facets have enough common points to be
    /// direct neighbors.
    bool areFacetsNeighbor( const Index if1, const Index if2 ) const
    {
      const Facet& f1 = facets[ if1 ];
      const Facet& f2 = facets[ if2 ];
      Index nb = 0;
      for ( Index i1 = 0, i2 = 0; i1 < f1.on_set.size() && i2 < f2.on_set.size(); )
        {
          if ( f1.on_set[ i1 ] == f2.on_set[ i2 ] )     { nb++; i1++; i2++; }
          else if ( f1.on_set[ i1 ] < f2.on_set[ i2 ] ) i1++;
          else                                          i2++;
        }
      return nb >= ( dimension - 1 );
    }
  
    /// Builds a facet from a base convex set of at least d different
    /// points and a point below.
    ///
    /// @param[in] base a range containing at least d distinct points
    /// in general position.
    ///
    /// @param[in] below a point below the hyperplane containing \a simplex.
    ///
    /// @return a facet object of normal and c parameter such as the
    /// points of \a simplex lies on the facet hyperplane while point \a
    /// below lies under.
    Facet makeFacet( const IndexRange& base, Index below ) const
    {
      CombinatorialPlaneSimplex simplex;
      for ( Size i = 0; i < dimension; i++ ) simplex[ i ] = base[ i ];
      auto plane = kernel.compute( points, simplex, below );
      return Facet( plane, below );
    }

    /// @param[in] R a ridge between two facets (a pair of facets).
    /// @return the points lying on a ridge.
    IndexRange pointsOnRidge( const Ridge& R ) const
    {
      // Slightly faster to look at points instead at true geometry.
      IndexRange result;
      const Facet& f1 = facets[ R.first ];
      const Facet& f2 = facets[ R.second ];
      std::set_intersection( f1.on_set.cbegin(), f1.on_set.cend(),
                             f2.on_set.cbegin(), f2.on_set.cend(),
                             std::back_inserter( result ) );
      return result;
    }

    /// Given a facet index \a f, return its points oriented
    /// consistently with respect to the normal.
    ///
    /// @param f any valid facet index @return the range of point
    /// indices that support this facet, in a consistent ordering.
    ///
    /// @note the order of points is consistent if, picking any
    /// d-simplex in order in this range, their associated half-spaces
    /// have all the same orientation.
    IndexRange orientedFacetPoints( Index f ) const
    {
      const Facet& F = facets[ f ];
      IndexRange result = F.on_set;
      // Sort a facet such that its points, taken in order, have
      // always the same orientation of the facet.  More precisely,
      // facets span a `dimension-1` vector space. There are thus
      // dimension-2 fixed points, and the last ones (at least two)
      // may be reordered.
      CombinatorialPlaneSimplex splx;
      for ( Dimension k = 0; k < dimension-2; k++ )
        splx[ k ] = result[ k ];
      // std::cout << "Orienting face " << f << " (";
      // for ( auto i : result ) std::cout << " " << i;
      std::sort( result.begin()+dimension-2, result.end(),
                 [&] ( Index i, Index j )
                 {
                   splx[ dimension-2 ] = i;
                   splx[ dimension-1 ] = j;
                   const auto H = kernel.compute( points, splx );
                   const auto orient = kernel.dot( F.H, H );
                   return orient > 0;
                 } );
      for ( Dimension k = 0; k < dimension; k++ )
        splx[ k ] = result[ k ];
      // const auto H = kernel.compute( points, splx );
      // const auto orient = kernel.dot( F.H, H );
      // std::cout << " ) => (";
      // for ( auto i : result ) std::cout << " " << i;
      // std::cout << " ) N=" << H.internalNormal() << " dot=" << orient << "\n";
      return result;
    }

  
    /// Filters each vertex on the facet \a f to keep only the ones that
    /// are on or below the neighboring facets
    /// @note intended for debugging purposes.
    /// @param[in] f any valid facet index.
    void filterVerticesOnFacet( const Index f )
    {
      auto & on_set = facets[ f ].on_set;
      for ( Index i = 0; i < on_set.size(); )
        {
          Index     v = on_set[ i ];
          Size      n = 0;
          for ( auto&& N : facets[ f ].neighbors )
            if ( on( facets[ N ], points[ v ] ) ) n += 1;
          if ( n >= dimension-1 ) i++;
          else {
            on_set[ i ] = on_set.back();
            on_set.pop_back();
          }                        
        }
      std::sort( on_set.begin(), on_set.end() );
    }
  
    /// @return a full dimensional simplex as a vector of d + 1 distinct
    /// indices of input points, or an empty vector if none was found.
    IndexRange pickInitialSimplex() const
    {
      const Size          nb = points.size();
      if ( nb < dimension + 1 ) return IndexRange();
      IndexRange        best = pickIntegers( dimension + 1, nb );
      CombinatorialPlaneSimplex splx;
      for ( Index j = 0; j < dimension; ++j ) splx[ j ] = best[ j ];
      const auto     first_H = kernel.compute( points, splx, best.back() );
      auto       best_volume = kernel.volume ( first_H, points[ best.back() ] );
      // Randomized approach to find full dimensional simplex.
      //
      // Let a be the proportion of full dimensional simplices among
      // all simplices of the input points. Let p be the desired
      // probability to find a full dimensional simplex after t tries.
      // Then t >= log(1-p)/log(1-a)
      // For a=0.25, p=99% we found 16 tries.
      // For a=0.20, p=99% we found 20 tries.
      const Size     nbtries = std::min( (Size) 10, 1 + nb / 10 );
      // const Size max_nbtries = std::max( (Size) 10, 2 * nb );
      const Size max_nbtries = 20;
      for ( Size i = 0; i < max_nbtries; i++ )
        {
          IndexRange tmp = pickIntegers( dimension + 1, nb );
          for ( Index j = 0; j < dimension; ++j ) splx[ j ] = tmp[ j ];
          const auto        tmp_H = kernel.compute( points, splx, tmp.back() );
          const auto   tmp_volume = kernel.volume ( tmp_H, points[ tmp.back() ] );
          if ( best_volume < tmp_volume ) {
            if ( debug_level >= 1 ) {
              trace.info() << "(" << i << ")"
                        << " new_volume = " << tmp_volume
                        << " > " << best_volume << std::endl;
            }
            best = tmp;
            best_volume = tmp_volume;
          }
          if ( i >= nbtries && best_volume > 0 )
            return best;
        }
      // If not found, we adopt a deterministic algorithm based on Gauss reduction.
      best = AffineGeometry<Point>::affineSubset( points );
      if ( debug_level >= 1 )
        trace.info() << "[QuickHull::pickInitialSimplex] #affine subset = " << best.size() << std::endl;
      return ( best.size() == (dimension+1) ) ? best : IndexRange();
    }

    /// @return a vector of d distinct integers in `{0, 1, ..., n-1}` randomly chosen.
    /// @param[in] d the number of returned integers
    /// @param[in] n the range of possible integers `{0, 1, ..., n-1}`
    static IndexRange pickIntegers( const Size d, const Size n )
    {
      IndexRange result( d );
      bool distinct = false;
      while ( ! distinct )
        {
          distinct = true;
          for ( Index i = 0; i < d; i++ ) result[ i ] = rand() % n;
          std::sort( result.begin(), result.end() );
          for ( Index i = 1; distinct && i < d; i++ )
            distinct = result[ i-1 ] != result[ i ];
        }
      return result;
    }

    /// Computes the initial configuration induced by the given full
    /// dimensional simplex. Follows Status::InputInitialized and and
    /// terminate with Status::SimplexCompleted if everything went
    /// well.
    ///
    /// @return 'true' iff the input data contains d+1 points in general
    /// position, the object has then the status SimplexCompleted, otherwise
    /// returns 'false' and the status is NotFullDimensional.
    bool computeSimplexConfiguration( const IndexRange& full_simplex )
    {
      assignment = std::vector< Index >( points.size(), UNASSIGNED );
      facets.resize( dimension + 1 );
      deleted_facets.clear();
      for ( Index j = 0; j < full_simplex.size(); ++j )
        {
          IndexRange lsimplex( dimension );
          IndexRange isimplex( dimension );
          Index s = 0;
          for ( Index i = 0; i <= dimension; i++ )
            if ( i != j ) {
              lsimplex[ s ] = i;
              isimplex[ s ] = full_simplex[ i ];
              s++;
            }
          facets[ j ] = makeFacet( isimplex, full_simplex[ j ] );
          facets[ j ].neighbors = lsimplex;
          for ( auto&& v : isimplex ) facets[ j ].on_set.push_back( v );
          std::sort( facets[ j ].on_set.begin(), facets[ j ].on_set.end() );
        }
      // List of unassigned vertices
      for ( Index fi = 0; fi < facets.size(); ++fi ) {
        Facet& f = facets[ fi ];
        for ( Index v = 0; v < points.size(); v++ )
          {
            if ( assignment[ v ] == UNASSIGNED && above( f, points[ v ] ) ) {
              f.outside_set.push_back( v );
              assignment[ v ] = fi;
            } 
          }
      }
      for ( Index v = 0; v < points.size(); v++ )
        if ( assignment[ v ] == UNASSIGNED )
          processed_points.push_back( v );
      
      // Display some information
      if ( debug_level >= 2 ) {
        for ( auto&& f : facets ) f.display( trace.info() );
      }
      myStatus = Status::SimplexCompleted;
      if ( debug_level >= 1 ) {
        trace.info() << "[CHECK INVARIANT] " << processed_points.size()
                     << " / " << points.size() << " points processed." << std::endl;
        bool okh = checkHull(); 
        if ( ! okh )
          trace.error() << "[computeInitialSimplex] Invalid convex hull" << std::endl;
        bool okf = checkFacets();
        if ( ! okf )
          trace.error() << "[computeInitialSimplex] Invalid facets" << std::endl;
        if ( ! ( okh && okf ) ) myStatus = Status::InvalidConvexHull;
      }
      return status() == Status::SimplexCompleted;
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
      out << "[QuickHull " << dimension << "D"
        //        << " status=" << status()
          << " #P=" << nbPoints();
      if ( status() >= Status::FacetsCompleted && status() <= Status::AllCompleted )
        out << " #F=" << nbFacets();
      if ( status() >= Status::VerticesCompleted && status() <= Status::AllCompleted )
        out << " #V=" << nbVertices();
      out << "]";
      // if ( status() >= Status::FacetsCompleted && status() <= Status::AllCompleted
      //      && nbFacets() == 24 ) {
      //   for ( auto f : facets ) f.display( out );
      //   for ( auto v : v2p ) out << points[ v2p[ v ] ] << std::endl;
      // }
    }
  
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return status() >= Status::Uninitialized
        && status() <= Status::AllCompleted;
    }
    /// @}
  
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'QuickHull'.
   * @tparam TKernel any type of QuickHull kernel, like ConvexHullIntegralKernel.
   * @param out the output stream where the object is written.
   * @param object the object of class 'QuickHull' to write.
   * @return the output stream after the writing.
   */
  template < typename TKernel >
  std::ostream&
  operator<< ( std::ostream & out,
               const QuickHull< TKernel > & object )
  {
    object.selfDisplay( out );
    return out;
  }
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined QuickHull_h

#undef QuickHull_RECURSES
#endif // else defined(QuickHull_RECURSES)
  
