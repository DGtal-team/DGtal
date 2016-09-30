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
 * @file MeshVoxelizer.h
 * @brief Compute a voxelization of a Mesh into a DigitalSet
 * @date 2016/01/24
 *
 * Header file for module MeshVoxelizer.ih
 *
 * This file is part of the DGtal library.
 */

#if !defined MeshVoxelizer_h
/** Prevents repeated inclusion of headers. */
#define MeshVoxelizer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/geometry/tools/determinant/PredicateFromOrientationFunctor2.h"
#include "DGtal/geometry/tools/determinant/InHalfPlaneBySimple3x3Matrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  struct arete {
    using PointR3 = PointVector<3, double>;
    PointR3 f, s;

    arete() {}
    arete(PointR3 mf, PointR3 ms) : f(mf), s(ms) {}
  };

  /////////////////////////////////////////////////////////////////////////////
  /**
     Description of template class 'MeshVoxelizer' <p> \brief Aim: A
     class for computing a voxelization of a Mesh, use topological
     target intersection to make a n-separating voxelization
     with n parameterizable

     @tparam TDigitalSet a DigitalSet using std::vector, std::set, or SVO
     @tparam separation strategy of the voxelization (6 or 26)
  */
  template <typename TDigitalSet, size_t separation = 6>
  class MeshVoxelizer
  {
    using Space3Dint = SpaceND<3>;
    using MeshFace = std::vector< unsigned int >;
    using Domain   = HyperRectDomain<Space3Dint>;
    using PointR3  = PointVector<3, double>;
    using VectorR3 = PointVector<3, double>;
    using PointR2  = PointVector<2, double>;
    using PointZ3  = PointVector<3, int>;

    using OrientationFunctor = InHalfPlaneBySimple3x3Matrix<PointR2, double>;

  private:
    std::vector<arete> myIntersectionTarget;
    Mesh<PointR3> myMesh;
    TDigitalSet myDigitalSet;

  public:

    /**
     * @brief Constructor of the voxelizer
     * @param aMesh mesh to voxelize
     * @param aDomain digital space of the voxelization
     * @param aResolution resolution of the voxelization grid
     */
    MeshVoxelizer(const Mesh<PointR3>& aMesh, Domain& aDomain, size_t aResolution);

    /**
     * @brief voxelize the mesh into the digital set
     */
    void voxelize();

    /**
     * @brief getter for digitalSet
     * @return current digital set
     */
    const TDigitalSet& digitalSet() const;
    
    //
    // some internal static functions
    //
    
    /**
     * @brief compute distance between p and plan defined by normal n and point M
     * @param M point
     * @param n normal
     * @param p point p
     * @return distance
     */
    static double distance(const PointR3& M, const VectorR3& n, const PointZ3& p);

    /**
     * @brief predicat to know if p (2D point) is inside ABC (2D triangle)
     * @param p point p
     * @return true if p is inside ABC
     */
    static int pointIsInside2DTriangle(PointR2& A, PointR2& B, PointR2& C, PointR2& p);

    /**
     * @brief predicat to know if point P is inside voxel v
     * @param P point P
     * @param v voxel v
     * @return true if P is inside v
     */
    static bool pointIsInsideVoxel(PointR3& P, PointZ3& v);

    /**
     * @brief voxelize ABC to the digitalSet
     * @param n normal of ABC
     * @param bbox bounding box of ABC
     * @return true if ok
     */
    bool voxelizeTriangle(PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox)
    {
      // tag dispatching
      return voxelizeTriangle(std::integral_constant<size_t, separation>{}, A, B, C, n, bbox);
    }

    /**
     * @brief function specialization of voxelizeTriangle for 6-separated voxelization using tag dispatching
     */
    bool voxelizeTriangle(std::integral_constant<size_t, 6>, PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox);

    /**
     * @brief function specialization of voxelizeTriangle for 26-separated voxelization using tag dispatching
     */
    bool voxelizeTriangle(std::integral_constant<size_t, 26>, PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox);
  };
}

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/MeshVoxelizer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshVoxelizer_h
