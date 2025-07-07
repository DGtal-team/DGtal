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

#ifndef DGTAL_WITH_LIBIGL
#error You need to have activated LIBIGL (DGTAL_WITH_LIBIGL flag) to include this file.
#endif

#include <vector>
#include <algorithm>
#include <DGtal/base/Common.h>
#include <DGtal/base/CountedConstPtrOrConstPtr.h>
#include <DGtal/base/ConstAlias.h>
#include <DGtal/shapes/CEuclideanOrientedShape.h>


//Removing Warnings from libIGL for gcc and clang
#pragma GCC system_header  // For GCC
#pragma clang system_header  // For Clang

#include <igl/fast_winding_number.h>
#include <igl/octree.h>
#include <igl/knn.h>
#include <igl/copyleft/cgal/point_areas.h>

#pragma GCC diagnostic pop  // For GCC
#pragma clang diagnostic pop  // For Clang



namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  /**
   Description of template class 'WindingNumbersShape'
   
   \brief Aim: model of a CEuclideanOrientedShape from an implicit
   function from an oriented point cloud. The implicit function is given by the
   generalized winding number  of the oriented point cloud  @cite barill2018fast .
   We use the libIGL implementation.
   
   @see testWindingNumberShape,  windingNumberShape
   
   @tparam TSPace the digital space type (a model  of CSpace)
   */
  template<typename TSpace>
  struct WindingNumbersShape
  {
    ///Types
    using Space       = TSpace;
    using RealPoint   = typename Space::RealPoint;
    using RealVector  = typename Space::RealVector;
    using Orientation = DGtal::Orientation;
    
    //Removing Default constructor
    WindingNumbersShape() = delete;
    
    /// Construct a WindingNumberShape Euclidean shape from an oriented point cloud.
    /// This constructor estimates the @a area @a  of each point using CGAL.
    ///
    /// If the number of points is greater than 20, CGAL is used to estimate the
    /// per-sample area (if skipPointAreas is set to false).
    ///
    /// @param points a "nx3" matrix with the sample coordinates.
    /// @param normals a "nx3" matrix for the normal vectors.
    /// @param skipPointAreas if true, we do not estimate the point areas using CGAL (default: false)
    WindingNumbersShape(ConstAlias<Eigen::MatrixXd> points,
                        ConstAlias<Eigen::MatrixXd> normals,
                        bool skipPointAreas = false)
    {
      myPoints  = points;
      myNormals = normals;
      myPointAreas = Eigen::VectorXd::Ones(myPoints->rows());
      // Build octree, from libIGL tutorials
      igl::octree(*myPoints,myO_PI,myO_CH,myO_CN,myO_W);
      if (skipPointAreas)
        trace.warning()<<"[WindingNumberShape] Skipping the CGAL point area estimation. By default, point areas are set to 1.0"<<std::endl;
      else
        if (points->rows()> 20)
        {
          Eigen::MatrixXi I;
          igl::knn(*myPoints,(int)points->rows(),myO_PI,myO_CH,myO_CN,myO_W,I);
          // CGAL is only used to help get point areas
          igl::copyleft::cgal::point_areas(*myPoints,I,*myNormals,myPointAreas);
          trace.info()<<"[WindingNumberShape] Min/max point area : "<<myPointAreas.minCoeff()<<" -- "<<myPointAreas.maxCoeff()<<std::endl;
        }
        else
        {
          trace.warning()<<"[WindingNumberShape] Too few points to use CGAL point_areas. Using the constant area setting."<<std::endl;
        }
    }
    
    /// Construct a WindingNumberShape Euclidean shape from an oriented point cloud.
    /// For this constructor, the @a area @a of each point is given by the user.
    ///
    /// @param points a "nx3" matrix with the sample coordinates.
    /// @param normals a "nx3" matrix for the normal vectors.
    /// @param areas a "n" vector with the @a area @a of each point.
    WindingNumbersShape(ConstAlias<Eigen::MatrixXd> points,
                        ConstAlias<Eigen::MatrixXd> normals,
                        const Eigen::VectorXd &areas)
    {
      myPoints  = points;
      myNormals = normals;
      myPointAreas = areas;
      igl::octree(*myPoints,myO_PI,myO_CH,myO_CN,myO_W);
    }
    
    
    /// Set the @a area @a map for each point.
    /// @param areas a Eigen vector of estimated area for each input point.
    void setPointAreas(ConstAlias<Eigen::VectorXd> areas)
    {
      myPointAreas = areas;
    }
    
    /// Orientation of a point using the winding number value from
    /// an oriented pointcloud.
    ///
    /// @note For multiple queries, orientationBatch() should be used.
    ///
    /// @param aPoint [in] a point in space
    /// @param threshold [in] the iso-value of the surface of the winding number implicit map (default = 0.3).
    /// @return a DGtal::Orientation value
    Orientation orientation(const RealPoint aPoint,
                            const double threshold = 0.3) const
    {
      Eigen::MatrixXd queries(1,3);
      queries << aPoint(0) , aPoint(1) , aPoint(2);
      auto singlePoint = orientationBatch(queries, threshold);
      return singlePoint[0];
    }
    
    /// Orientation of a set of points (queries) using the winding number value from
    /// an oriented pointcloud.
    ///
    /// @param queries [in] a "nx3" matrix with the query points in space.
    /// @param threshold [in] the iso-value of the surface of the winding number implicit map (default = 0.3).
    /// @return a DGtal::Orientation value vector for each query point.
    std::vector<Orientation> orientationBatch(const Eigen::MatrixXd & queries,
                                              const double threshold = 0.3) const
    {
      Eigen::VectorXd W;
      std::vector<Orientation> results( queries.rows(), DGtal::OUTSIDE );

      rawWindingNumberBatch(queries, W);
     
      //Reformating the output
      for(auto i=0u; i < queries.rows(); ++i)
      {
        if (std::abs(W(i)) < threshold )
          results[i] = DGtal::OUTSIDE;
        else
          if (std::abs(W(i)) > threshold)
            results[i] = DGtal::INSIDE;
          else
            results[i] = DGtal::ON;
      }
      return results;
    }
    

    /// Returns the raw value of the Winding Number funciton at a set of points (queries).
    ///
    /// @param queries [in] a "nx3" matrix with the query points in space.
    /// @param W [out] a vector with all windung number values.
    void rawWindingNumberBatch(const Eigen::MatrixXd & queries,
                               Eigen::VectorXd &W) const
    {
      Eigen::MatrixXd O_CM;
      Eigen::VectorXd O_R;
      Eigen::MatrixXd O_EC;
      
      //Computing the WN values
      igl::fast_winding_number(*myPoints,*myNormals,myPointAreas,myO_PI,myO_CH,2,O_CM,O_R,O_EC);
      igl::fast_winding_number(*myPoints,*myNormals,myPointAreas,myO_PI,myO_CH,O_CM,O_R,O_EC,queries,2,W);
    }
    
    ///Const alias to the points
    CountedConstPtrOrConstPtr<Eigen::MatrixXd> myPoints;
    ///Const alias to the normals
    CountedConstPtrOrConstPtr<Eigen::MatrixXd> myNormals;
    ///Const alias to point area measure
    Eigen::VectorXd myPointAreas;
    
    ///libIGL octree for fast queries data structure
    std::vector<std::vector<int > > myO_PI;
    ///libIGL octree for fast queries data structure
    Eigen::MatrixXi myO_CH;
    ///libIGL octree for fast queries data structure
    Eigen::MatrixXd myO_CN;
    ///libIGL octree for fast queries data structure
    Eigen::VectorXd myO_W;
    
    
  };
}

#endif // !defined WindingNumbersShape_h
