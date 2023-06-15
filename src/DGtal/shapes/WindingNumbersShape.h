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
 * @file WindingNumbersShape.h
 * @brief CEuclideanOrientedShape model using libIGL Winding numbers
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2023/06/14

 *
 * This file is part of the DGtal library.
 */

#if !defined WindingNumbersShape_h
/** Prevents repeated inclusion of headers. */
#define WindingNumbersShape_h

#include <DGtal/base/Common.h>
#include <DGtal/shapes/CEuclideanOrientedShape.h>
#include <igl/fast_winding_number.h>
#include <igl/octree.h>
#include <igl/knn.h>
namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  /**
     Description of template class 'WindingNumbersShape'

     \brief Aim:
   
     @see

     @tparam TDimension dimension of the intersection target
  */
  template<typename TSpace, size_t TDimension = 1>
  struct WindingNumbersShape
  {
    
    ///Types
    using Space       = TSpace;
    using RealPoint   = typename Space::RealPoint;
    using RealVector  = typename Space::RealVector;
    using Orientation = DGtal::Orientation;
    
    //Removing Default constructor
    WindingNumbersShape() = delete;

    WindingNumbersShape(Eigen::MatrixXd &points, Eigen::MatrixXd &normals)
    {
      // Build octree
      std::vector<std::vector<int > > O_PI;
      Eigen::MatrixXi O_CH;
      Eigen::MatrixXd O_CN;
      Eigen::VectorXd O_W;
      igl::octree(P,O_PI,O_CH,O_CN,O_W);
      Eigen::VectorXd A;
      {
        Eigen::MatrixXi I;
        igl::knn(points,20,O_PI,O_CH,O_CN,O_W,I);
        // CGAL is only used to help get point areas
        igl::copyleft::cgal::point_areas(points,I,N,A);
      }
    }
    
    
    /// Orientation of a point using the winding number value from
    /// an oriented pointcloud.
    ///
    /// @param aPoint [in] a point in space
    /// @return a DGtal::Orientation value
    Orientation orientation(const RealPoint aPoint) const
    {
      return DGtal::OUTSIDE;
    }
    
    
  };
}

#endif // !defined WindingNumbersShape_h
