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
#include <type_traits>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  struct arete {
    using PointR3 = PointVector<3, double>;
    PointR3 f, s;

    arete() {}
    arete(PointR3 mf, PointR3 ms) : f(mf), s(ms) {}
  };

  /*
  struct OverlapTestFunctor {
    using PointR3 = PointVector<3, double>;
    using PointR2 = PointVector<2, double>;
    using PointZ3 = PointVector<3, int>;
    double d;

    arete target;

    PointR3 A, B, C;
    PointR2 AA, BB, CC;
    PointZ3 v;
  };
*/
  /////////////////////////////////////////////////////////////////////////////
  /**
     Description of template class 'MeshVoxelizer' <p> \brief Aim: A
     class for computing a voxelization of a Mesh, use topological
     target intersection to make a n-separating voxelization
     with n parameterizable

     @tparam TDigitalSet a model of DigitalSet using std::vector, std::set, or an SVO
     @tparam separation strategy of the voxelization (6, 18 or 26)
  */
  template <typename TDigitalSet, size_t separation = 6>
  class MeshVoxelizer
  {
    friend struct OverlapTestFunctor;

    using MeshFace = std::vector< unsigned int >;
    using Space3Dint = SpaceND<3>;
    using Domain = HyperRectDomain<Space3Dint>;
    using PointR3 = PointVector<3, double>;
    using VectorR3 = PointVector<3, double>;
    using PointR2 = PointVector<2, double>;
    using VectorR2 = PointVector<2, double>;
    using PointZ3 = PointVector<3, int>;
    using PointZ2 = PointVector<2, int>;

    using OrientationFunctor = InHalfPlaneBySimple3x3Matrix<PointR2, double>;

  private:
    std::vector<arete> myIntersectionTarget;
    Mesh<PointR3> myMesh;
    TDigitalSet myDigitalSet;

  public:

    /**
     * Constructor
     */
    MeshVoxelizer(const Mesh<PointR3>&, Domain&, size_t resolution);

    /**
     * Voxelize the mesh into the digital set
     */
    void voxelize();

    const TDigitalSet& digitalSet() const;

    static double distance(const PointR3& A, const PointR3& B, const PointR3& C, const VectorR3& n, const PointZ3& voxel);
    static int    pointIsInside2DTriangle(PointR2& A, PointR2& B, PointR2& C, PointR2& v);
    static bool   pointIsInsideVoxel(PointR3& P, PointZ3& v);

    bool voxelizeTriangle(PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox)
    {
      // tag dispatching
      voxelizeTriangle(std::integral_constant<size_t, separation>{}, A, B, C, n, bbox);
    }

    bool voxelizeTriangle(std::integral_constant<size_t, 6>, PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox);
    bool voxelizeTriangle(std::integral_constant<size_t, 26>, PointR3& A, PointR3& B, PointR3& C, const VectorR3& n, std::pair<PointR3, PointR3>& bbox);
  };
}

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/MeshVoxelizer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshVoxelizer_h
