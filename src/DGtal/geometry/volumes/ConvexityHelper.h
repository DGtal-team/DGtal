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
 * @file ConvexityHelper.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/12/20
 *
 * Header file for module ConvexityHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConvexityHelper_RECURSES)
#error Recursive header files inclusion detected in ConvexityHelper.h
#else // defined(ConvexityHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConvexityHelper_RECURSES

#if !defined ConvexityHelper_h
/** Prevents repeated inclusion of headers. */
#define ConvexityHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/QuickHull.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"
#include "DGtal/geometry/volumes/BoundedRationalPolytope.h"
#include "DGtal/geometry/volumes/ConvexCellComplex.h"
#include "DGtal/shapes/PolygonalSurface.h"

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexityHelper
  /**
     Description of template class 'ConvexityHelper' <p> \brief Aim:
     Provides a set of functions to facilitate the computation of
     convex hulls and polytopes, as well as shortcuts to build cell
     complex representing a Delaunay complex.

     @tparam dim the dimension of the space where points and further objects live.
  */
  template < int dim, typename TInteger = DGtal::int32_t >
  struct ConvexityHelper {
    BOOST_STATIC_ASSERT( dim > 1 );
    static const Dimension dimension = dim;

    typedef TInteger                         Integer;
    typedef SpaceND< dim, Integer >          Space;
    typedef typename Space::Point            Point;
    typedef typename Space::Vector           Vector;
    typedef typename Space::RealPoint        RealPoint;
    typedef typename Space::RealVector       RealVector;
    typedef std::size_t                      Size;
    typedef std::size_t                      Index;
    typedef std::vector< Index >             IndexRange;
    typedef ConvexHullIntegralKernel< dim >  LatticeConvexHullKernel;
    typedef ConvexHullRationalKernel< dim >  RealConvexHullKernel;
    typedef DelaunayIntegralKernel< dim >    LatticeDelaunayKernel;
    typedef DelaunayRationalKernel< dim >    RealDelaunayKernel;
    typedef BoundedLatticePolytope< Space >  LatticePolytope;
    typedef BoundedRationalPolytope< Space > RationalPolytope;

    // ----------------- lattice convex hull services -------------------------
  public:
    /// @name Lattice convex hull services
    /// @{
    
    /// Computes and returns a halfspace representation of the tightiest lattice
    /// polytope enclosing all the given input lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @param[in] make_minkowski_summable Other constraints are added
    /// so that we can perform axis aligned Minkowski sums on this
    /// polytope. Useful for checking full convexity (see
    /// moduleDigitalConvexity).
    ///
    /// @return the tightiest bounded lattice polytope
    /// (i.e. H-representation) including the given range of points,
    /// or an empty polytope if the given range of points was not full
    /// dimensional.
    static
    LatticePolytope
    computeLatticePolytope( const std::vector< Point >& input_points,
                            bool remove_duplicates = true,
                            bool make_minkowski_summable = false );
    

    
    /// Computes a surface mesh representation of the boundary of the
    /// convex hull of the given lattice points.
    ///
    /// @note Since it builds a surface, this method is thus 3D.
    ///
    /// @tparam TSurfaceMesh any model of surface that can be
    /// initialized with a range of input positions (cast as real
    /// coordinates) and a range of index ranges giving for each face
    /// its range of incident vertices. For instance, you may use
    /// class SurfaceMesh.
    ///
    /// @param[out] mesh the output surface mesh that represents the
    /// boundary of the convex hull of the given range of points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output mesh is correct, otherwise return 'false'.
    template < typename TSurfaceMesh >
    static
    bool
    computeConvexHullBoundary( TSurfaceMesh&               mesh,
                               const std::vector< Point >& input_points,
                               bool remove_duplicates = true );

    /// Computes a polygonal surface representation of the boundary of the
    /// convex hull of the given lattice points.
    ///
    /// @note Since it builds a surface, this method is thus 3D.
    ///
    /// @param[out] polysurf the output polygonal surface that
    /// represents the boundary of the convex hull of the given range
    /// of points. Its euler characteristic should be 0 in even
    /// dimension, 2 in odd dimension.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output surface is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullBoundary( PolygonalSurface< Point >&  polysurf,
                               const std::vector< Point >& input_points,
                               bool remove_duplicates = true );

    /// Computes a cell complex representing the convex hull of the
    /// given lattice points, formed of one maximal dimension cell and
    /// as many cells of codimension 1 as the number of facets of the
    /// convex hull.
    ///
    /// @param[out] cell_complex the output cell complex that
    /// represents the convex hull of the given lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output complex is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullCellComplex( ConvexCellComplex< Point >& cell_complex,
                                  const std::vector< Point >& input_points,
                                  bool remove_duplicates = true );

    /// @}
    
    // ----------------- lattice Delaunay services -------------------------
  public:
    /// @name Lattice Delaunay services
    /// @{

    /// Computes the Delaunay cell complex associated to the given
    /// range of input points.
    ///
    /// @param[out] cell_complex the output cell complex that
    /// represents the Delaunay complex of the given lattice points.
    ///
    /// @param[in] input_points the range of input lattice points.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output complex is correct, otherwise return 'false'.
    ///
    /// @note The Delaunay cell complex may not be simplicial if some
    /// points are cospherical.
    static
    bool
    computeDelaunayCellComplex( ConvexCellComplex< Point >& cell_complex,
                                const std::vector< Point >& input_points,
                                bool remove_duplicates = true );
    
    /// @}

    // ----------------- rational convex hull services -------------------------
  public:
    /// @name Rational convex hull services
    /// @{
    
    /// Computes and returns a halfspace representation of the tightiest rational
    /// polytope enclosing all the given input real points.
    ///
    /// @param[in] input_points the range of input real points.
    ///
    /// @param[in] denominator the denominator used in all rational
    /// approximations of points with real value coordinates, the
    /// higher the more precise.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @param[in] make_minkowski_summable Other constraints are added
    /// so that we can perform axis aligned Minkowski sums on this
    /// polytope. Useful for checking full convexity (see
    /// moduleDigitalConvexity).
    ///
    /// @return the tightiest bounded lattice polytope
    /// (i.e. H-representation) including the given range of points,
    /// or an empty polytope if the given range of points was not full
    /// dimensional.
    static
    RationalPolytope
    computeRationalPolytope( const std::vector< RealPoint >& input_points,
                             Integer denominator, 
                             bool remove_duplicates = true,
                             bool make_minkowski_summable = false );

    /// Computes a surface mesh representation of the boundary of the
    /// rational polytope that approximates the convex hull of the given real points.
    ///
    /// @note Since it builds a surface, this method is thus 3D.
    ///
    /// @tparam TSurfaceMesh any model of surface that can be
    /// initialized with a range of input positions (cast as real
    /// coordinates) and a range of index ranges giving for each face
    /// its range of incident vertices. For instance, you may use
    /// class SurfaceMesh.
    ///
    /// @param[out] mesh the output surface mesh that represents the
    /// boundary of the convex hull of the given range of points.
    ///
    /// @param[in] input_points the range of input real points.
    ///
    /// @param[in] precision the scaling factor that is used to
    /// multiply each real coordinate before rounding it to an
    /// integer, a kind of common denominator if you think of the
    /// result as a rational number.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output mesh is correct, otherwise return 'false'.
    template < typename TSurfaceMesh >
    static
    bool
    computeConvexHullBoundary( TSurfaceMesh&               mesh,
                               const std::vector< RealPoint >& input_points,
                               double precision = 1024.0,
                               bool remove_duplicates = true );

    /// Computes a polygonal surface representation of the boundary of the
    /// rational polytope that approximates the convex hull of the given real points.
    ///
    /// @note Since it builds a surface, this method is thus 3D.
    ///
    /// @param[out] polysurf the output polygonal surface mesh that represents the
    /// boundary of the convex hull of the given range of points.
    ///
    /// @param[in] input_points the range of input real points.
    ///
    /// @param[in] precision the scaling factor that is used to
    /// multiply each real coordinate before rounding it to an
    /// integer, a kind of common denominator if you think of the
    /// result as a rational number.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output mesh is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullBoundary( PolygonalSurface< RealPoint > & polysurf,
                               const std::vector< RealPoint >& input_points,
                               double precision = 1024.0,
                               bool remove_duplicates = true );

    /// Computes a cell complex representing the convex hull of the
    /// given real points, formed of one maximal dimension cell and
    /// as many cells of codimension 1 as the number of facets of the
    /// convex hull.
    ///
    /// @param[out] cell_complex the output cell complex that
    /// represents the convex hull of the given real points.
    ///
    /// @param[in] input_points the range of input real points.
    ///
    /// @param[in] precision the scaling factor that is used to
    /// multiply each real coordinate before rounding it to an
    /// integer, a kind of common denominator if you think of the
    /// result as a rational number.
    ///
    /// @param[in] remove_duplicates should be set to 'true' if the
    /// input data has duplicates.
    ///
    /// @return 'true' if the input points were full dimensional and
    /// the output complex is correct, otherwise return 'false'.
    static
    bool
    computeConvexHullCellComplex( ConvexCellComplex< RealPoint >& cell_complex,
                                  const std::vector< RealPoint >& input_points,
                                  double precision = 1024.0,
                                  bool remove_duplicates = true );
    
    /// @}
    
    // ----------------- utility services -------------------------
  public:
    /// @name Utility services
    /// @{

    /// @tparam QHull any QuickHull concrete type.
    /// @param[in] hull a computed QuickHull object
    ///
    /// @param[out] cell_vertices the vector giving for each cell the
    /// indices of its vertices.
    ///
    /// @param[out] r2f the map giving for each ridge (i.e. the pair
    /// of cells defining each face) the index of its corresponding
    /// face.
    ///
    /// @param[out] face_vertices the vector giving for each face the
    /// indices of its vertices.
    /// 
    /// @pre `hull.status() >= Status::VerticesCompleted` and
    /// `hull.status() <= Status::AllCompleted`
    template < typename QHull >
    static
    void
    computeFacetAndRidgeVertices( const QHull& hull,
                                  std::vector< IndexRange >& cell_vertices,
                                  std::map< typename QHull::Ridge, Index >& r2f,
                                  std::vector< IndexRange >& face_vertices );

    /// @}
    
  }; // class ConvexityHelper

  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "ConvexityHelper.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConvexityHelper_h

#undef ConvexityHelper_RECURSES
#endif // else defined(ConvexityHelper_RECURSES)
