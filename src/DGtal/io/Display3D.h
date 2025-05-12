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
 * @file Display.h
 * @author Bastien Doignies <bastien.doignies@liris.cnrs.fr>
 *
 * @date 2025/05/11
 *
 * Header file for 3D Display
 *
 * This file is part of the DGtal library.
 */

#include <iostream>

#include <optional>
#include <cstdint>
#include <vector>
#include <string>
#include <array>
#include <map>

#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/topology/Object.h"

#include "DGtal/kernel/CanonicEmbedder.h" 
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/Color.h"

namespace DGtal {
    /**
     * @brief Style of display of an element
     */
    struct DisplayStyle {
      std::optional<Color> color; //< Color of the object. Nullptr is unspecified
      double width = 1.0;     //< Width of the object

      /**
       * @brief List available draw modes.
       *
       * Draw mode are meant as a shortcut to change how objects
       * are displayed. 
       * Not all draw modes are supported for every objects. 
       */
      enum DrawMode : size_t {
        DEFAULT      = (1 << 0), //< Default mode
        PAVING       = (1 << 1), //< For voxels, render them as cubes
        BALLS        = (1 << 2), //< For voxels, render them as balls
        ADJACENCIES  = (1 << 3), //< For objects, draws adjacencies
        GRID         = (1 << 4), //< For domains, draws a grid
        SIMPLIFIED   = (1 << 5)  //< For KCell, draws quads instead of prisms
      };

      size_t mode = static_cast<size_t>(DrawMode::DEFAULT);
    };

    template<typename Vertex>
    struct DisplayData {
      std::size_t           elementSize; //< Size for each elements. If 0, elements may have variable size (ie. polygonal faces).
                                         // 1 -> Ball or Cube depending on DrawMode
                                         // 2 -> Lines
                                         // 0, 3, 4 -> Polygonal / Triangular / Quad mesh
                                         // 8 -> Volumetric mesh
      std::vector<std::vector<uint32_t>> indices;     //< Indices for each elements. Only used if elementSize is 0.
      std::vector<Vertex>   vertices;    //< Vertices of the object


      DisplayStyle style;
      std::map<std::string, std::vector<Vertex>> vectorProperties;
      std::map<std::string, std::vector<double>> scalarProperties;
      std::map<std::string, std::vector<Color>>  colorProperties;
    };

    template < typename Space = Z3i::Space, typename KSpace = Z3i::KSpace>
    class Display {
    public:
      Display(const KSpace& space) :
        kspace(space), 
        cellEmbedder(kspace),
        sCellEmbedder(kspace)
      { }

      Display() : Display(KSpace()) {}

      using Point = typename Space::Point;
      using KCell = typename KSpace::Cell;
      using Vertex = typename Space::RealPoint ;

      using Embedder = CanonicEmbedder<Space>;
      using CellEmbedder = CanonicCellEmbedder<KSpace>;
      using SCellEmbedder = CanonicSCellEmbedder<KSpace>;
    public:
      void debug() {
        for (const auto& [name, ddata] : data) {
          std::cout << name << std::endl;
          std::cout << "\tVertices: " << ddata.vertices.size() << std::endl;
          std::cout << "\tSize: " << ddata.elementSize << std::endl;
          std::cout << "\tMode: " << ddata.style.mode << std::endl;
          std::cout << "\tProperties:" << std::endl;
          std::cout << "\t\tScalar: " << ddata.scalarProperties.size() << std::endl;
          std::cout << "\t\tVector: " << ddata.vectorProperties.size() << std::endl;
          std::cout << "\t\tColor: " << ddata.colorProperties.size() << std::endl;
        }
      }
    public:
      template<typename Obj>
      Display& operator<<(const Obj& obj) {
        draw(obj);
      }
     
      void draw(const Point& p) {
        if (shouldCreateNewGroup(1 /* size of each element */)) {
          newCubeGroup("Point_{i}");
        }

        currentData->vertices.push_back(embedder.embed(p));
      }
      
      void draw(const KCell& cell) {
        const Vertex rp = cellEmbedder.embed(cell);
        
        // TODO: Is this necessary (waring with BigIntegers)?
        const bool xodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[0]) & 1);
        const bool yodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[1]) & 1);
        const bool zodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[2]) & 1);
          
        addCell(rp, xodd, yodd, zodd, false, false);
     }

      void addCell(const Vertex& rp, bool xodd, bool yodd, bool zodd, bool hasSign, bool sign) {
        const double scale = 0.9;
        const static std::array<std::array<Vertex, 4>, 3> prismShifts {
            { 
              Vertex(-1, -1, -1),
              Vertex(-1, -1,  1),
              Vertex(-1,  1,  1),
              Vertex(-1,  1, -1)
            }, 
            {
              Vertex(-1, -1, -1),
              Vertex(-1, -1,  1),
              Vertex( 1, -1,  1),
              Vertex( 1, -1, -1)
            }, 
            {
              Vertex(-1, -1, -1),
              Vertex(-1,  1, -1),
              Vertex( 1,  1, -1),
              Vertex( 1, -1, -1)
            }
        };
        const unsigned int dim = xodd + yodd + zodd;
        const unsigned int index = (xodd ? 0 : (yodd ? 1 : 2));
        
        switch(dim) {
          case 0: {
            if (shouldCreateNewGroup(1)) {
              newBallGroup("KCell_{i}_d0");
              currentData->style.width *= scale;
            }

            currentData->vertices.push_back(rp);
          }
          break;
          case 1: {
            if (shouldCreateNewGroup(2)) {
              newLineGroup("KCell_{i}_d1");
              currentData->style.width *= scale;
            }

            const Vertex shift(xodd, yodd, zodd);
            currentData->vertices.push_back(rp - 0.5 * shift);
            currentData->vertices.push_back(rp + 0.5 * shift);
          }
          break;
          case 2: {
            if (shouldCreateNewGroup(8)) {
              newVolumetricGroup("KCell_{i}_d2");
              currentData->style.width *= scale;
            }
            

            const auto& shifts = prismSHifts[index];
            const Vertex x(rp - xodd * 
            
          };
          case 3: {
            if (shouldCreateNewGroup(1)) {
              newCubeGroup("KCell_{i}_d3");
              currentData->style.width *= scale;
            }

            currentData->vertices.push_back(rp);
          };
          break;
        };
      }

      void draw(const HyperRectDomain<Space>& domain) {
        drawGenericObject("Domain_{i}", domain);
        //TODO: Grid mode
      }
      
      template<typename Obj, typename Cont>
      void draw(const DigitalSetByAssociativeContainer<Obj, Cont>& set) {
        drawGenericObject("Set_{i}", set);
      }

      template<typename Adj, typename Set>
      void draw(const DGtal::Object<Adj, Set>& obj) {
        std::string name = drawGenericObject("Object_{i}", obj);

        // Draw adjacency if needed
        if (currentStyle.mode & DisplayStyle::DrawMode::ADJACENCIES) {
          newLineGroup(name + "_adj");
          
          for (auto it = obj.begin(); it != obj.end(); ++it) {
              auto neig = obj.properNeighborhood(*it);

              const Vertex pIt = embedder.embed(*it);
              for (auto it2 = neig.begin(); it2 != neig.end(); ++it2) {
                auto p = embedder.embed((*it2) - (*it));

                currentData->vertices.push_back(p);
                currentData->vertices.push_back(pIt);
              }
          }

          currentData = nullptr;
        }
      }
    private:
      template<typename Obj>
      std::string drawGenericObject(const std::string& name, const Obj& obj) {
        std::string newName = newCubeGroup(name);
      
        currentData->vertices.reserve(obj.size());
        for (auto it = obj.begin(); it != obj.end(); ++it) {
          currentData->vertices.push_back(*it);
        }

        currentData = nullptr;
        return newName;
      }
    public:
      /**
       * @brief Set the current group for further updates
       */
      bool setCurrentGroup(const std::string& name) {
        auto it = data.find(name);
        if (it == data.end()) 
          return false;
        
        currentData = &it->second;
        return true;
      }
      
      std::string newCubeGroup(const std::string& name)       { return newGroup(name, 1); }
      std::string newBallGroup(const std::string& name)       { return newGroup(name, 1); }
      std::string newLineGroup(const std::string& name)       { return newGroup(name, 2); }
      std::string newQuadGroup(const std::string& name)       { return newGroup(name, 4); }
      std::string newPolygonGroup(const std::string& name)    { return newGroup(name, 0); }
      std::string newTriangleGroup(const std::string& name)   { return newGroup(name, 3); }
      std::string newVolumetricGroup(const std::string& name) { return newGroup(name, 8); }

    private:
      bool shouldCreateNewGroup(size_t expectedSize) const {
        if (!currentData) return true;
        if (currentData->elementSize != expectedSize) return true;
        return allowReuseGroup;
      }

      /**
       * @brief Create a new group
       *
       * If the name requested already exists; another one is
       * computed as follows:
       * - If the token {i} is present, replace the first occurence with 
       *   the first available index, starting from one.
       * - Otherwise, appends "_i" is the first available index, 
       *   starting from one.
       *
       * @param name The name to insert
       * @param eSize elementSize of the DisplayData 
       *
       * @return The computed name
       */
      std::string newGroup(const std::string& name, size_t eSize = 0) {
        static const std::string token = "{i}";

        auto it = data.find(name);
        std::string newName = name;

        size_t idx = name.find(token);
        if (it != data.end() || idx != std::string::npos) {
          std::string prefix = name;

          if (idx == std::string::npos) {
            idx = prefix.size() + 1;
            prefix = prefix + "_" + token;
          }

          size_t i = 1;
          do {
            std::string tmpPrefix = prefix;
            newName = tmpPrefix.replace(idx, token.size(), std::to_string(i));

            i += 1;
          } while (data.find(newName) != data.end());
        }
        
        // Insert a new empty container
        DisplayData<Vertex> newData;
        newData.style = currentStyle;
        newData.elementSize = eSize;

        currentData = &data.emplace(newName, std::move(newData)).first->second;
        return newName;
      }

    public:
      KSpace kspace;
      Embedder embedder;
      CellEmbedder cellEmbedder;
      SCellEmbedder sCellEmbedder;

      /** For elements that can be manipulated independantly, adds a small shortcuts to allow to build groups iteratively. */
      bool allowReuseGroup = false;

      DisplayStyle currentStyle;
      std::map<std::string, DisplayData<Vertex>> data;
      DisplayData<Vertex>* currentData = nullptr;
    };

}

