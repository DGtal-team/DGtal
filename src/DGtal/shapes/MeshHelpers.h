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
 * @file MeshHelpers.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/11
 *
 * Header file for module MeshHelpers.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MeshHelpers_RECURSES)
#error Recursive header files inclusion detected in MeshHelpers.h
#else // defined(MeshHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeshHelpers_RECURSES

#if !defined MeshHelpers_h
/** Prevents repeated inclusion of headers. */
#define MeshHelpers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CCellEmbedder.h"
#include "DGtal/topology/CDigitalSurfaceContainer.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/shapes/TriangulatedSurface.h"
#include "DGtal/shapes/PolygonalSurface.h"
#include "DGtal/shapes/Mesh.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class MeshHelpers
  /**
   * Description of template class 'MeshHelpers' <p>
   * \brief Aim: Static class that provides builder and converters between meshes.
   */
  class MeshHelpers
  {
    // ----------------------- Static services ------------------------------
  public:
    /// Builds a triangulated surface (class TriangulatedSurface) from
    /// a mesh (class Mesh). Note that a triangulated surface contains
    /// only triangles, so polygonal faces (0,1,2,3,4,...) of the
    /// input mesh are (naively) triangulated (triangles (0,1,2),
    /// (0,2,3), (0,3,4), etc). Furthermore, the output triangulated
    /// surface rebuilds a topology between faces.
    ///
    /// @tparam Point the type for points.
    /// @param[in]  mesh the input mesh.
    /// @param[out] trisurf the output triangulated surface mesh.
    ///
    /// @return 'true' on success, 'false' if the input \a mesh was
    /// not a combinatorial surface.
    template <typename Point>
    static
    bool mesh2TriangulatedSurface
    ( const Mesh<Point>& mesh,
      TriangulatedSurface<Point>& trisurf );

    /// Builds a polygon mesh (class PolygonalSurface) from
    /// a mesh (class Mesh). The output polygonal
    /// surface rebuilds a topology between faces.
    ///
    /// @tparam Point the type for points.
    /// @param[in]  mesh the input mesh.
    /// @param[out] polysurf the output polygonal surface mesh.
    ///
    /// @return 'true' on success, 'false' if the input \a mesh was
    /// not a combinatorial surface.
    template <typename Point>
    static
    bool mesh2PolygonalSurface
    ( const Mesh<Point>& mesh,
      PolygonalSurface<Point>& polysurf );

    /// Builds a triangulated surface (class TriangulatedSurface) from
    /// the dual graph of a 2-dimensional digital surface in K^3 (class
    /// DigitalSurface).
    ///
    /// @note that a triangulated surface contains only triangles, so
    /// faces of the input dual graph of the digital surface mesh are
    /// triangulated by adding a new vertex at the barycenter of the
    /// face vertices.
    ///
    /// @tparam DigitalSurfaceContainer the container chosen for the digital surface.
    /// @tparam CellEmbedder the embedder chosen for the digital surface.
    ///
    /// @param[in]  dsurf the input digital surface.
    /// @param[in]  cembedder the embedder for 2-cells of the digital surface, which are vertices in the output triangulated surface.
    /// @param[out] trisurf the output triangulated surface mesh.
    /// @param[out] vertexmap the output mapping between a Vertex of \a dsurf and an Index in \a trisurf.
    template < typename DigitalSurfaceContainer,
               typename CellEmbedder,
	       typename VertexMap >
    static
    void digitalSurface2DualTriangulatedSurface
    ( const DigitalSurface<DigitalSurfaceContainer>& dsurf,
      const CellEmbedder& cembedder,
      TriangulatedSurface<typename CellEmbedder::Value>& trisurf,
      VertexMap& vertexmap );

    /// Builds a polygonal surface (class PolygonalSurface) from
    /// the dual graph of a 2-dimensional digital surface in K^3 (class
    /// DigitalSurface).
    ///
    /// @tparam DigitalSurfaceContainer the container chosen for the digital surface.
    /// @tparam CellEmbedder the embedder chosen for the digital surface.
    ///
    /// @param[in]  dsurf the input digital surface.
    /// @param[in]  cembedder the embedder for 2-cells of the digital surface, which are vertices in the output polygonal surface.
    /// @param[out] polysurf the output polygonal surface mesh.
    /// @param[out] vertexmap the output mapping between a Vertex of \a dsurf and an Index in \a polysurf.
    template < typename DigitalSurfaceContainer,
               typename CellEmbedder,
	       typename VertexMap >
    static
    void digitalSurface2DualPolygonalSurface
    ( const DigitalSurface<DigitalSurfaceContainer>& dsurf,
      const CellEmbedder& cembedder,
      PolygonalSurface<typename CellEmbedder::Value>& polysurf,
      VertexMap& vertexmap );
    
    
    /// Builds a mesh (class Mesh) from a triangulated surface (class
    /// TriangulatedSurface). Note that the mesh looses the topology
    /// of the triangulated surface, since it is essentially a soup of
    /// triangles.
    ///
    /// @tparam Point the type for points.
    /// @param[in]  trisurf the input triangulated surface mesh.
    /// @param[in,out] mesh the output mesh (which should be empty).
    template <typename Point>
    static
    void triangulatedSurface2Mesh
    ( const TriangulatedSurface<Point>& trisurf,
      Mesh<Point>& mesh );

    /// Builds a mesh (class Mesh) from a polygon mesh (class
    /// PolygonalSurface). Note that the mesh looses the topology
    /// of the polygonal surface, since it is essentially a soup of
    /// triangles.
    ///
    /// @tparam Point the type for points.
    /// @param[in]     polysurf the input polygonal surface (ie a polygon mesh).
    /// @param[in,out] mesh the output mesh (which should be empty).
    template <typename Point>
    static
    void polygonalSurface2Mesh
    ( const PolygonalSurface<Point>& polysurf,
      Mesh<Point>& mesh );
    
  }; // end of class MeshHelpers

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/MeshHelpers.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshHelpers_h

#undef MeshHelpers_RECURSES
#endif // else defined(MeshHelpers_RECURSES)
