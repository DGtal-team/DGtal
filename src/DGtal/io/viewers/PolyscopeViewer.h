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
 * Header file for 3D Display
 *
 * This file is part of the DGtal library.
 */

#include <DGtal/io/Display3D.h>

#include "polyscope/polyscope.h"

#include "polyscope/curve_network.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"
#include "polyscope/volume_grid.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"

namespace DGtal {
  namespace drawutils {
    glm::vec3 toglm(const DGtal::Color& col) {
      return glm::vec3{col.r(), col.g(), col.b()};
    }
    std::vector<glm::vec3> toglm(const std::vector<DGtal::Color>& col) {
      std::vector<glm::vec3> colors(col.size());
      for (size_t i = 0; i < colors.size(); ++i) {
        colors[i] = toglm(col[i]);
      }
      return colors;
    }
  }

  template < typename Space = Z3i::Space, typename KSpace = Z3i::KSpace>
  class PolyscopeViewer : public Display<Space, KSpace> {
    public:
      PolyscopeViewer() : Display<Space, KSpace>() {
        polyscope::init();
      }
      
      void show() {
        registerData();
        polyscope::show();
      }
    private:
      void registerData() {
        const double BallToCubeRatio = 0.05;

        for (const auto& [name, data] : this->data) {
          const auto& vertices = data.vertices;
          switch(data.elementSize) {
          case 1: {
              auto* pCloud = polyscope::registerPointCloud(name, vertices);

              if (data.style.color.has_value())
                pCloud->setPointColor(drawutils::toglm(*data.style.color));

              pCloud->setPointRadius(data.style.width * BallToCubeRatio);
            }
            break;
          case 2: {
              auto* cNetwork = polyscope::registerCurveNetwork(name, vertices, drawutils::makeIndices<2>(vertices.size() / 2));

              if (data.style.color.has_value())
                cNetwork->setColor(drawutils::toglm(*data.style.color));
            }
          case 0: {
              auto* mesh = polyscope::registerSurfaceMesh(name, vertices, data.indices);

              if (data.style.color.has_value())
                mesh->setSurfaceColor(drawutils::toglm(*data.style.color));
            }
            break;
          case 3: {
              auto* mesh = polyscope::registerSurfaceMesh(name, vertices, drawutils::makeIndices<3>(vertices.size() / 3));
              if (data.style.color.has_value())
                mesh->setSurfaceColor(drawutils::toglm(*data.style.color));
            }
          break;
          case 4: {
              auto* mesh = polyscope::registerSurfaceMesh(name, vertices, drawutils::makeIndices<4>(vertices.size() / 4));
              if (data.style.color.has_value())
                mesh->setSurfaceColor(drawutils::toglm(*data.style.color));
            }
            break;
          case 8: {
              auto* mesh = polyscope::registerVolumeMesh(name, vertices, drawutils::makeIndices<8>(vertices.size() / 8));

              if (data.style.color.has_value())
                mesh->setColor(drawutils::toglm(*data.style.color));
            }
            break;
          };
          // Apply general parameters
        }
      }
  };
}

