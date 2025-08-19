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
 **/
#pragma once

/**
 * @file PolyscopeViewer.h
 * @author Bastien Doignies <bastien.doignies@liris.cnrs.fr>
 *
 * @date 2025/05/11
 *
 * Header file for 3D Polyscope Viewer
 *
 * This file is part of the DGtal library.
 */

#include "DGtal/io/Display3D.h"

#include "polyscope/polyscope.h"

#include "polyscope/curve_network.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"
#include "polyscope/volume_grid.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"

namespace DGtal {
  namespace drawutils {
    // @brief Convert a DGtal color to a glm color
    glm::vec3 toglm(const DGtal::Color& col);

    // @brief Convert a vector of DGtal color to a vector of glm color
    std::vector<glm::vec3> toglm(const std::vector<DGtal::Color>& col);

    // @brief Convert an Eigen transform to glm matrix
    glm::mat4 toglm(const Eigen::Affine3d& transform);
  }

  template < typename Space = Z3i::Space, typename KSpace = Z3i::KSpace>
  class PolyscopeViewer : public Display3D<Space, KSpace> {
    public:
      PolyscopeViewer() : Display3D<Space, KSpace>() {
        polyscope::init();
      }

      PolyscopeViewer(const KSpace& k) : Display3D<Space, KSpace>(k) {
        polyscope::init();
      }

      // @brief Clear the view (ie. remove all polyscope structures)
      void clearView() override {
        polyscope::removeAllStructures();
      }

      // @brief Starts event loop (first render data)
      void show() override {
        renderNewData();
        renderClippingPlanes();

        polyscope::show();
      }

      // @brief Render new data
      void renderNewData() override;

      // @brief setCallback
      void setCallback(typename Display3D<Space, KSpace>::Callback* callback) override;

      // @brief Renders the clipping planes
      void renderClippingPlanes();

      ~PolyscopeViewer() {
        this->clear();
      }

    public:
      static constexpr double BallToCubeRatio = 0.025;
      static constexpr double VectorScale = 1. / 30.;

    private:
      void setGeneralProperties(polyscope::Structure* s, const DisplayData<typename Space::RealPoint>& d);

      void registerPointCloud (const std::string& n, const DisplayData<typename Space::RealPoint>& d);
      void registerLineNetwork(const std::string& n, const DisplayData<typename Space::RealPoint>& d);
      void registerSurfaceMesh(const std::string& n, const DisplayData<typename Space::RealPoint>& d);
      void registerVolumeMesh (const std::string& n, const DisplayData<typename Space::RealPoint>& d);

      // @brief Global callback handling UI, click and callback calls
      void polyscopeCallback();
  };
}

#include "PolyscopeViewer.ih"
