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
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/Color.h"

#define STB_IMAGE_IMPLEMENTATION
#include "DGtal/io/readers/misc/stb_image.h"
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
    
    ///UV coordinates buffer
    using UVMap = std::vector<UV>;
    
    ///Normal vectors buffer
    using NormalMap = std::vector<Point>;
    
    ///UV information for face : triple per UV index
    using UVTriangle = std::array<std::size_t,3>;
    
    ///UVMap for a mesh (3 indices per face)
    using UVMesh = typename TriangulatedSurf::template IndexedPropertyMap<UVTriangle>;

    
    ///NormalMap for a mesh (3 indices per face)
    using NormalMesh = typename TriangulatedSurf::template IndexedPropertyMap<UVTriangle>;

    
    ///Texture image
    using TextureImage = ImageContainerBySTLVector<Z2i::Domain, Color>;
    
    
    
    static
    std::tuple<TriangulatedSurf,  UVMesh, NormalMesh, UVMap, NormalMap>
    loadOBJWithTextureCoord(const std::string &filename)
    {
      TriangulatedSurf mesh;

      UVMap textureMap;
      NormalMap normalMap;
      UVMesh textMesh;
      NormalMesh normalMesh;

      std::vector<UVTriangle> originalVertIndices;
      std::vector<UVTriangle> originalUVIndices;
      std::vector<UVTriangle> originalNormalIndices;

      std::ifstream in(filename, std::ios::in);
      if (!in)
      {
        std::cerr << "Cannot open " << filename << std::endl;
        exit(1);
      }
      std::string line;
      double U,V;
      double x,y,z;
      int a,b,c; //to store mesh index
      int A,B,C; //to store texture index
      int nA,nB,nC; //to store thhe normal index
      
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
        else if(line.substr(0,2)=="vn")
        {
          std::istringstream v(line.substr(3));
          v>>x;v>>y;v>>z;
          normalMap.push_back({x,y,z});
        }
        else if(line.substr(0,2)=="f ")
        {
          const char* chh=line.c_str();
          sscanf (chh, "f %i/%i/%i %i/%i/%i %i/%i/%i",&a,&A,&nA,&b,&B,&nB,&c,&C,&nC); //here it read the line start with f and store the corresponding values in the variables
          a--;b--;c--;
          A--;B--;C--;
          nA--;nB--;nC--;
          mesh.addTriangle(a,b,c);
          
          UVTriangle vv={(size_t)a,(size_t)b,(size_t)c};
          UVTriangle vvv={(size_t)A,(size_t)B,(size_t)C};
          UVTriangle vvvv={(size_t)nA,(size_t)nB,(size_t)nC};
          originalVertIndices.push_back(vv);
          originalUVIndices.push_back(vvv);
          originalNormalIndices.push_back(vvvv);
        }
      }
      
      //Building the mesh
      mesh.build();
      
      //Reordering the UV indices per face
      textMesh = mesh.template makeFaceMap<UVTriangle>();
      normalMesh = mesh.template makeFaceMap<UVTriangle>();
      for(auto f=0; f < mesh.nbFaces(); ++f)
      {
        auto a=originalVertIndices[f][0], b=originalVertIndices[f][1];
        auto A=originalUVIndices[f][0],   B=originalUVIndices[f][1],   C=originalUVIndices[f][2];
        auto nA=originalNormalIndices[f][0],   nB=originalNormalIndices[f][1],   nC=originalNormalIndices[f][2];
        auto verts= mesh.verticesAroundFace(f);
        UVTriangle vv;
        UVTriangle nvv;
        if (verts[0]==a)
        {
          vv[0]=A;
          nvv[0]=nA;
        }
        else
          if (verts[0]==b)
          {
            vv[0]=B;
            nvv[0]=nB;
          }
          else
          {
            vv[0]=C;
            nvv[0]=nC;
          }
        if (verts[1]==a)
        {
          vv[1]=A;
          nvv[1]=nA;
        }
        else
          if (verts[1]==b)
          {
            vv[1]=B;
            nvv[1]=nB;
          }
          else
          {
            vv[1]=C;
            nvv[0]=nA;
          }
        if (verts[2]==a)
        {
          vv[2]=A;
          nvv[2]=nA;
        }
        else
          if (verts[2]==b)
          {
            vv[2]=B;
            nvv[2]=nB;
          }
          else
          {
            vv[2]=C;
            nvv[2]=nC;
          }
        textMesh[f] = vv;
        normalMesh[f] = nvv;
      }
      return std::make_tuple(mesh,textMesh,normalMesh,textureMap,normalMap);
    }
    
    
    static
    TextureImage loadTexture(const std::string &filename, const bool silent=true)
    {
      int width,height, nbChannels;
      unsigned char *source = stbi_load(filename.c_str(), &width, &height, &nbChannels, 0);
      if (!silent)
        trace.info()<< "Source image: "<<width<<"x"<<height<<"   ("<<nbChannels<<")"<< std::endl;
      
      Z2i::Domain dom(Z2i::Point(0,0), Z2i::Point(width-1, height-1));
      ASSERT_MSG(nbChannels==3, "RGB input image only");
      TextureImage result(dom);
      for(auto i=0; i<width*height; ++i)
      {
        Z2i::Point p( i % width , i / width);
        result.setValue(p, Color(source[3*i],source[3*i+1],source[3*i+2]));
      }
      return result;
    }
    
    
    static
    Color
    textureFetch(const TextureImage &tex,
                 const UV &uvcoord )
    {
      auto extent = tex.domain().upperBound() -  tex.domain().lowerBound();
      Z2i::Point p = { static_cast<Z2i::Point::Coordinate>(std::round(uvcoord[0]*extent[0])),
                       static_cast<Z2i::Point::Coordinate>(std::round(uvcoord[1]*extent[1]))};
      return tex(p);
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
