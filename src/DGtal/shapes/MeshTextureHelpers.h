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
 * @file MeshTextureHelpers.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/03/24
 *
 * Header file for module MeshTextureHelpers.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MeshTextureHelpers_RECURSES)
#error Recursive header files inclusion detected in MeshTextureHelpers.h
#else // defined(MeshTextureHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeshTextureHelpers_RECURSES

#if !defined MeshTextureHelpers_h
/** Prevents repeated inclusion of headers. */
#define MeshTextureHelpers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <iterator>
#include <algorithm>
#include <array>
#include <tuple>

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
  // template class MeshTextureHelpers
  /**
   * Description of template class 'MeshTextureHelpers' <p>
   * \brief Aim: Static class that provides methods to manipulate and retreive
   *  texture information for meshes
   *
   *  UV map encoding:  per face, a triple of UV coordinates
   */
  template <typename TPoint>
  class MeshTextureHelpers
  {
    // ----------------------- Static services ------------------------------
  public:
    
    ///Point type
    using Point = TPoint;
    
    using TriangulatedSurf = TriangulatedSurface<TPoint>  ;
    
    ///UV coordinate (point in R^2)
    using UV = PointVector<2, double> ;
    
    ///UV map, vector of UV coordinates
    using UVMap = std::vector<UV>;
    
    ///UV information for face : triple per UV index
    using UVTriangle = std::array<std::size_t,3>;
    
    ///UVMap for a mesh
    using UVMesh = typename TriangulatedSurf::template IndexedPropertyMap<UVTriangle>;

    
    static
    std::tuple<TriangulatedSurf,  UVMesh, UVMap>
    loadOBJWithTextureCoord(const std::string &filename)
    {
      UVMap textureMap;
      UVMesh textMesh;
    
      std::vector<UVTriangle> originalVertIndices;
      std::vector<UVTriangle> originalUVIndices;
      TriangulatedSurf mesh;
      
      std::ifstream in(filename, std::ios::in);
      if (!in)
      {
        std::cerr << "Cannot open " << filename << std::endl;
        exit(1);
      }
      std::string line;
      double U,V;
      double x,y,z;

      while (std::getline(in, line))
      {
        //check v for vertices
        if (line.substr(0,2)=="v ")
        {
          std::istringstream v(line.substr(2));
          v>>x;v>>y;v>>z;
          mesh.addVertex({x,y,z});
        }
        //check for texture co-ordinate
        else if(line.substr(0,2)=="vt")
        {
          std::istringstream v(line.substr(3));
          UV tex;
          v>>U;v>>V;
          textureMap.push_back({U,V});
        }
        //check for faces
        else if(line.substr(0,2)=="f ")
        {
          int a,b,c; //to store mesh index
          int A,B,C; //to store texture index
          const char* chh=line.c_str();
          sscanf (chh, "f %i/%i %i/%i %i/%i",&a,&A,&b,&B,&c,&C); //here it read the line start with f and store the corresponding values in the variables
          a--;b--;c--;
          A--;B--;C--;
          mesh.addTriangle(a,b,c);
          
          UVTriangle vv={(size_t)a,(size_t)b,(size_t)c};
          UVTriangle vvv={(size_t)A,(size_t)B,(size_t)C};
          originalVertIndices.push_back(vv);
          originalUVIndices.push_back(vvv);
        }
      }
      
      //Building the mesh
      mesh.build();
      
      //Reordering the UV indices per face
      textMesh = mesh.template makeFaceMap<UVTriangle>();
      for(auto f=0; f < mesh.nbFaces(); ++f)
      {
        auto a=originalVertIndices[f][0], b=originalVertIndices[f][1];
        auto A=originalUVIndices[f][0],   B=originalUVIndices[f][1],   C=originalUVIndices[f][2];
        auto verts= mesh.verticesAroundFace(f);
        UVTriangle vv;
        if (verts[0]==a)
          vv[0]=A;
        else
          if (verts[0]==b)
            vv[0]=B;
          else
            vv[0]=C;
        if (verts[1]==a)
          vv[1]=A;
        else
          if (verts[1]==b)
            vv[1]=B;
          else
            vv[1]=C;
        if (verts[2]==a)
          vv[2]=A;
        else
          if (verts[2]==b)
            vv[2]=B;
          else
            vv[2]=C;
        textMesh[f] = vv;
      }
      return std::make_tuple(mesh,textMesh,textureMap);
    }
    
    
    
    /// Retreive the barycentric coordinate of a point in a triangular face.
    /// The point is first projected onto the triangle plane.
    ///
    /// The barycentric coefficients are all positive (and sum to 1)
    /// if the point (its orthogonal projection) lies inside the triangle.
    ///
    /// @param trisurf the input triangular surface.
    /// @param faceindex the index of the triangular face.
    /// @param aPoint a point
    /// @return the barycentric coordinates (same order as trisurf.verticesAroundFace(faceindex))
    static
    Point getBarycentricCoordinatesInFace( const TriangulatedSurf  & trisurf,
                                          const typename TriangulatedSurf::Face faceindex,
                                          const  Point &aPoint)
    {
      auto vertices = trisurf.verticesAroundFace(faceindex);
      Point A = trisurf.position(vertices[0]);
      Point B = trisurf.position(vertices[1]);
      Point C = trisurf.position(vertices[2]);
      
      //Compute the principal direction of the triangle
      Point normal = ((B-A).crossProduct(C-A)).getNormalized();
      
      //Project the point onto the triangle plane
      Point Pproj = aPoint - (aPoint-A).dot(normal) * normal;
      Point pA,pB,pC;
      Point lambda;
      pA = A-Pproj;
      pB = B-Pproj;
      pC = C-Pproj;
      
      Point va  = (A-B).crossProduct(A-C); // main triangle cross product
      Point va1 = pB.crossProduct(pC); // p1's triangle cross product
      Point va2 = pC.crossProduct(pA); // p2's triangle cross product
      Point va3 = pA.crossProduct(pB); // p3's triangle cross product
      
      double  area = va.norm();
      lambda[0] = (pB.crossProduct(pC)).norm() / area * sgn(va.dot(va1));
      lambda[1] = (pC.crossProduct(pA)).norm() / area * sgn(va.dot(va2));;
      lambda[2] = (pA.crossProduct(pB)).norm() / area * sgn(va.dot(va3));;
      
      return lambda;
    }
    
    /// Retreive the barycentric coordinate of a point in a triangular face.
    /// The point is first projected onto the triangle plane.
    ///
    /// @param trisurf the input triangular surface.
    /// @param faceindex the index of the triangular face.
    /// @param lambda the barycentric coordinates in the triangle (same order as trisurf.verticesAroundFace(faceindex))
    /// @return the point lambda[0]*A+lambda[1]*B+lambda[2]*C;
    template <typename Point>
    static
    Point getPointFromBarycentricCoordinatesInFace( const TriangulatedSurf  & trisurf,
                                                   const typename TriangulatedSurf::Face faceindex,
                                                   const  Point &lambda)
    {
      auto vertices = trisurf.verticesAroundFace(faceindex);
      Point A = trisurf.position(vertices[0]);
      Point B = trisurf.position(vertices[1]);
      Point C = trisurf.position(vertices[2]);
      return lambda[0]*A+lambda[1]*B+lambda[2]*C;
    }
    
    
    
    
    
    
  private:
    template <typename T>
    static
    int sgn(T val)
    {
      return (T(0) < val) - (val < T(0));
    }
  }; // end of class MeshTextureHelpers
  
  } // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshTextureHelpers_h

#undef MeshTextureHelpers_RECURSES
#endif // else defined(MeshTextureHelpers_RECURSES)
