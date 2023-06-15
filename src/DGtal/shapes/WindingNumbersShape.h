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
#include <igl/copyleft/cgal/point_areas.h>
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

    WindingNumbersShape(const Eigen::MatrixXd &points, const Eigen::MatrixXd &normals)
    {
      myPoints  = points;
      myNormals = normals; 
      // Build octree, from libIGL tutorials
      igl::octree(myPoints,myO_PI,myO_CH,myO_CN,myO_W);
      {
        Eigen::MatrixXi I;
        igl::knn(myPoints,20,myO_PI,myO_CH,myO_CN,myO_W,I);
        // CGAL is only used to help get point areas
        igl::copyleft::cgal::point_areas(myPoints,I,myNormals,myPointAreas);
      }
    }
    
    
    /// Orientation of a point using the winding number value from
    /// an oriented pointcloud.
    ///
    /// @param aPoint [in] a point in space
    /// @return a DGtal::Orientation value
    Orientation orientation(const RealPoint aPoint, const double threshold = 0.5) const
    {
      Eigen::MatrixXd queries(3,1);
      queries << aPoint(0) , aPoint(1) , aPoint(2);
      auto singlePoint = orientationBatch(queries, threshold);
      return singlePoint[0];
    }
    
    ///
    std::vector<Orientation> orientationBatch(const Eigen::MatrixXd & queries, 
                                              const double threshold = 0.5) const
    {
      Eigen::VectorXd W;
      std::vector<Orientation> results( queries.rows() );
      Eigen::MatrixXd O_CM;
      Eigen::VectorXd O_R;
      Eigen::MatrixXd O_EC;
      igl::fast_winding_number(myPoints,myNormals,myPointAreas,myO_PI,myO_CH,2,O_CM,O_R,O_EC);
      igl::fast_winding_number(myPoints,myNormals,myPointAreas,myO_PI,myO_CH,O_CM,O_R,O_EC,queries,2,W);

      //Reformating the output
      for(auto i=0u; i < queries.rows(); ++i)
        if (queries(i) < threshold )
          results[i] = DGtal::OUTSIDE;
        else
           if (queries(i) > threshold)
              results[i] = DGtal::INSIDE;
           else
              results[i] = DGtal::ON;    
      return results;
    }

    ///Copy of the points
    Eigen::MatrixXd myPoints;
    ///Copy of the normals
    Eigen::MatrixXd myNormals;

    //libIGL octree for fast queries
    std::vector<std::vector<int > > myO_PI;
    Eigen::MatrixXi myO_CH;
    Eigen::MatrixXd myO_CN;
    Eigen::VectorXd myO_W;
    Eigen::VectorXd myPointAreas;

    
  };
}

#endif // !defined WindingNumbersShape_h
