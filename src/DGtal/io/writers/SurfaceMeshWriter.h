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
 * @file SurfaceMeshWriter.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for class SurfaceMeshWriter
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceMeshWriter_RECURSES)
#error Recursive header files inclusion detected in SurfaceMeshWriter.h
#else // defined(SurfaceMeshWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceMeshWriter_RECURSES

#if !defined SurfaceMeshWriter_h
/** Prevents repeated inclusion of headers. */
#define SurfaceMeshWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/io/Color.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceMeshWriter
  /**
     Description of template class 'SurfaceMeshWriter' <p> \brief Aim:
     An helper class for writing mesh file formats (Waverfront OBJ at this point) and creating a SurfaceMesh.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
  */
  template < typename TRealPoint, typename TRealVector >
  struct SurfaceMeshWriter
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef SurfaceMeshWriter< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );

    typedef DGtal::SurfaceMesh< RealPoint, RealVector > SurfaceMesh;
    typedef typename SurfaceMesh::Size           Size;
    typedef typename SurfaceMesh::Index          Index;
    typedef typename SurfaceMesh::Vertex         Vertex;
    typedef typename SurfaceMesh::Vertices       Vertices;
    typedef typename SurfaceMesh::Face           Face;
    typedef typename SurfaceMesh::Faces          Faces;
    typedef typename SurfaceMesh::Scalar         Scalar;
    typedef typename SurfaceMesh::Scalars        Scalars;
    typedef std::vector< Color >                 Colors;

    /// Writes a surface mesh in an output file (in OBJ file format).
    /// @param[in,out] output the output stream where the OBJ file is written.
    /// @param[in] smesh the surface mesh.
    /// @return 'true' if writing in the output stream was ok.
    static
    bool writeOBJ( std::ostream & output, const SurfaceMesh & smesh );

    /// Writes a surface mesh in the given OBJ file (and an associated
    /// MTL file) and associate color information.
    ///
    /// @param[in] objfile the name of the OBJ file (like "bunny" or "bunny.obj").
    /// @param[in] smesh the surface mesh.
    ///
    /// @param[in] diffuse_colors either empty or a vector containing
    /// the diffuse color for each face.
    ///
    /// @param[in] ambient_color,diffuse_color,specular_color the
    /// default color information for each face.
    static
    bool writeOBJ( std::string            objfile,
                   const SurfaceMesh &    smesh,
                   const Colors&          diffuse_colors = Colors(),
                   const Color&           ambient_color  = Color( 32, 32, 32 ),
                   const Color&           diffuse_color  = Color( 200, 200, 255 ),
                   const Color&           specular_color = Color::White );

    /// Writes, in an OBJ file, the geometric lines on edges that
    /// satisfies the given edge predicate.
    ///
    /// @tparam EdgePredicate any type of functor SurfaceMesh::Edge -> bool.
    ///
    /// @param[in] objfile the name of the OBJ file (like "contour" or
    /// "contour.obj").
    /// @param[in] smesh the surface mesh.
    ///
    /// @param[in] edge_predicate the edge predicate, a functor
    /// SurfaceMesh::Edge -> bool.
    ///
    /// @param[in] relative_thickness the thickness as a ratio of the
    /// average edge length of the mesh.
    ///
    /// @param[in] ambient_color,diffuse_color,specular_color the
    /// default color information for each face.
    template <typename EdgePredicate>
    static
    bool writeEdgeLinesOBJ( std::string            objfile,
                            const SurfaceMesh &    smesh,
                            const EdgePredicate &  edge_predicate,
                            const double           relative_thickness = 0.05,
                            const Color&           ambient_color = Color::Black,
                            const Color&           diffuse_color = Color::Black,
                            const Color&           specular_color= Color::Black );

    /// Writes, in an OBJ file, the geometric isolines of a scalar field
    /// around value \a iso_value. Scalar values of the scalar field
    /// should be given at vertices and face centroids.
    ///
    /// @param[in] objfile the name of the OBJ file (like "isocontour" or
    /// "isocontour.obj").
    /// @param[in] smesh the surface mesh.
    ///
    /// @param[in] face_values, vertex_values the range of values of
    /// the scalard field at faces and at vertices.
    ///
    /// @param[in] iso_value the value of the isoline to output.
    ///
    /// @param[in] relative_thickness the thickness as a ratio of the
    /// average edge length of the mesh.
    ///
    /// @param[in] ambient_color,diffuse_color,specular_color the
    /// default color information for each face.
    static
    bool writeIsoLinesOBJ( std::string            objfile,
                           const SurfaceMesh &    smesh,
                           const Scalars&         face_values,
                           const Scalars&         vertex_values,
                           const Scalar           iso_value,
                           const double           relative_thickness = 0.05,
                           const Color&           ambient_color = Color::Black,
                           const Color&           diffuse_color = Color::Black,
                           const Color&           specular_color= Color::Black );

    /// Writes, in an OBJ file, the geometric isolines of a scalar field
    /// around values \a iso_values. Scalar values of the scalar field
    /// should be given at vertices and face centroids.
    ///
    /// @param[in] objfile the name of the OBJ file (like "isocontour" or
    /// "isocontour.obj").
    /// @param[in] smesh the surface mesh.
    ///
    /// @param[in] face_values, vertex_values the range of values of
    /// the scalard field at faces and at vertices.
    ///
    /// @param[in] iso_values the values of the isolines to output.
    ///
    /// @param[in] relative_thickness the thickness as a ratio of the
    /// average edge length of the mesh.
    ///
    /// @param[in] diffuse_colors the range of colors for each isoline
    /// (in same order) or empty range (then the color is always \a
    /// diffuse_color).
    ///
    /// @param[in] ambient_color,diffuse_color,specular_color the
    /// default color information for each face.
    static
    bool writeIsoLinesOBJ( std::string            objfile,
                           const SurfaceMesh &    smesh,
                           const Scalars&         face_values,
                           const Scalars&         vertex_values,
                           const Scalars&         iso_values,
                           const double           relative_thickness = 0.05,
                           const Colors&          diffuse_colors = Colors(),
                           const Color&           ambient_color = Color::Black,
                           const Color&           diffuse_color = Color::Black,
                           const Color&           specular_color= Color::Black );

  };


} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "SurfaceMeshWriter.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceMeshWriter_h

#undef SurfaceMeshWriter_RECURSES
#endif // else defined(SurfaceMeshWriter_RECURSES)
