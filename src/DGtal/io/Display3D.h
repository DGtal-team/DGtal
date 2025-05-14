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

#include "DGtal/images/ImageAdapter.h"
#include "DGtal/images/ConstImageAdapter.h"

#include "DGtal/kernel/CanonicEmbedder.h" 
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/Color.h"

namespace DGtal {
    namespace drawutils {
      template<size_t I>
      std::vector<std::array<size_t, I>> makeIndices(size_t N) {
        std::vector<std::array<size_t, I>> indices(N);
      
        for (size_t i = 0; i < N; ++i) {
          for (size_t j = 0; j < I; ++j) {
            indices[i][j] = j + i * I;
          }
        }
        return indices;
      }

      template<typename T> 
      std::array<T, 8> getCubeVertices(T center, double size) {
        return {
          center + 0.5 * size * T(-1, -1, -1), 
          center + 0.5 * size * T( 1, -1, -1),
          center + 0.5 * size * T( 1,  1, -1), 
          center + 0.5 * size * T(-1,  1, -1),
          center + 0.5 * size * T(-1, -1,  1), 
          center + 0.5 * size * T( 1, -1,  1),
          center + 0.5 * size * T( 1,  1,  1), 
          center + 0.5 * size * T(-1,  1,  1)
        };
      }

      template<typename T, typename U>
      void insertCubeVertices(U& dest, T center, double scale) {
        auto vertices = getCubeVertices(center, scale);
        dest.insert(dest.end(), vertices.begin(), vertices.end());
      }

      template <typename T>
      std::array<T, 4> getAAQuadVertices(T center, int orientation, double size) {
        switch(orientation) {
        case 0: // Normal in x direction
          return {
            center + 0.5 * size * T(0, -1, -1), 
            center + 0.5 * size * T(0, -1,  1),
            center + 0.5 * size * T(0,  1,  1),
            center + 0.5 * size * T(0,  1, -1)
          };
        case 1: // Normal in y direction
          return {
            center + 0.5 * size * T(-1, 0, -1), 
            center + 0.5 * size * T(-1, 0,  1),
            center + 0.5 * size * T( 1, 0,  1),
            center + 0.5 * size * T( 1, 0, -1)
          };
        case 2: // Normal in z direction
        default:
          return {
            center + 0.5 * size * T(-1, -1, 0), 
            center + 0.5 * size * T(-1,  1, 0),
            center + 0.5 * size * T( 1,  1, 0),
            center + 0.5 * size * T( 1, -1, 0)
          };
        }
      }

      template<typename T>
      std::array<T, 8> getPrism(
          T center, int orientation, 
          double size1, double size2, double shift1, double shift2
      ) {
        T dir(0, 0, 0); dir[orientation] = 1; 

        std::array<T, 8> vertices;
        auto fQuad = getAAQuadVertices(center + shift1 * dir, orientation, size1);
        auto sQuad = getAAQuadVertices(center + shift2 * dir, orientation, size2);

        std::copy(fQuad.begin(), fQuad.end(), vertices.begin());
        std::copy(sQuad.begin(), sQuad.end(), vertices.begin() + fQuad.size());
        return vertices;
      }

      template<typename T, typename U>
      void insertPrism(U& dest, T center, int orientation, 
                       float scale1, float scale2, float shift1, float shift2) {
        auto vertices = getPrism(center, orientation, scale1, scale2, shift1, shift2);
        dest.insert(dest.end(), vertices.begin(), vertices.end());
      }
    };

    /**
     * @brief Style of display of an element
     */
    struct DisplayStyle {
      Color color = Color(200, 200, 200, 255);                 //< Color of the object. 
      bool useDefaultColors = true;                            //< When set, color is ignored and the viewer is free to choose
      
      double width = 1.0;                                      //< Maintains uniform scale of the object independently for easy access
      Eigen::Affine3d transform = Eigen::Affine3d::Identity(); //< Transform (includes scale)

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

    template<typename Vector>
    struct DisplayData {
      std::size_t           elementSize; //< Size for each elements. If 0, elements may have variable size (ie. polygonal faces).
                                         // 1 -> Ball or Cube depending on DrawMode
                                         // 2 -> Lines
                                         // 0, 3, 4 -> Polygonal / Triangular / Quad mesh
                                         // 8 -> Volumetric mesh
      std::vector<std::vector<uint32_t>> indices;     //< Indices for each elements. Only used if elementSize is 0.
      std::vector<Vector>   vertices;    //< Vertices of the object


      DisplayStyle style;
      std::map<std::string, std::vector<Vector>> vectorProperties;
      std::map<std::string, std::vector<double>> scalarProperties;
      std::map<std::string, std::vector<Color>>  colorProperties;
    };
    
    struct ClippingPlane {
      double a, b, c; //< Normal components
      double d;       //< Offset
    };
    
    template<typename T, typename Type>
    struct WithProperty{
      WithProperty(const T& object, const std::string& name, const Type& value) : 
        object(object), name(name) 
      {
        values.push_back(value);
      }
      
      WithProperty(const T& object, const std::string& name, const std::vector<Type>& values) : 
        object(object), name(name) 
      {
        this->values = values;
      }

      T object;
      std::vector<Type> values;
      std::string name;
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
      using SCell = typename KSpace::SCell;
      using Vector = typename Space::RealPoint ;

      using Embedder = CanonicEmbedder<Space>;
      using CellEmbedder = CanonicCellEmbedder<KSpace>;
      using SCellEmbedder = CanonicSCellEmbedder<KSpace>;

      struct Callback{
        virtual void OnUI() {};
        virtual void OnClick(const std::string& name, size_t index, const DisplayData<Vector>& data, void* viewerData) {};
      };
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
        return *this;
      }

      void setCallback(Callback* callback) {
        this->callback = callback;
      }
     
      std::string draw(const Point& p) {
        std::string name = currentName;
        const auto rp = embedder.embed(p);

        if (currentStyle.mode &= DisplayStyle::BALLS) {
          if (shouldCreateNewList(1)) {
            name = newBallList("Point_{i}");
          }
          
          currentData->vertices.push_back(rp);
        }
        else {
          if (shouldCreateNewList(8)) {
            name = newCubeList("Point_{i}");
          }
          
          drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
        }
        return name;
      }
      
      std::string draw(const KCell& cell) {
        const Vector rp = cellEmbedder.embed(cell);
        
        const bool xodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[0]) & 1);
        const bool yodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[1]) & 1);
        const bool zodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[2]) & 1);
          
        return addKCell(rp, xodd, yodd, zodd, false, false);
      }
      
      std::string draw(const SCell& cell) {
        const Vector rp = sCellEmbedder.embed(cell);
        
        const bool xodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[0]) & 1);
        const bool yodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[1]) & 1);
        const bool zodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[2]) & 1);
        
        return addKCell(rp, xodd, yodd, zodd, true, cell.precell().positive);
      }

      std::string draw(const HyperRectDomain<Space>& domain) {
        std::string name = drawGenericObject("Domain_{i}", domain);

        if (currentStyle.mode & DisplayStyle::GRID) {
          newLineList("Domain_{i}");
          
          // Faces YX
          for (auto z = domain.myLowerBound[2]; z <= domain.myUpperBound[2]; z++) {
            for (auto x = domain.myLowerBound[0];  x <= domain.myUpperBound[0]; x++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(x, domain.myLowerBound[1], z) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(x, domain.myUpperBound[1], z) );
              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }

            for (auto y = domain.myLowerBound[1]; y <= domain.myUpperBound[1]; y++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(domain.myLowerBound[0], y, z) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(domain.myUpperBound[0], y, z) );
              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }
          }

          // Faces XZ
          for (auto y = domain.myLowerBound[1]; y <= domain.myUpperBound[1]; y++) {
            for (auto x = domain.myLowerBound[0]; x <= domain.myUpperBound[0]; x++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(x, y, domain.myLowerBound[2]) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(x, y, domain.myLowerBound[2]) );

              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }
            for (auto z =  domain.myLowerBound[2]; z <= domain.myUpperBound[2]; z++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(domain.myLowerBound[0], y, z) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(domain.myUpperBound[0], y, z) );

              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }
          }

          // Faces YZ
          for (auto x = domain.myLowerBound[0]; x <= domain.myUpperBound[0]; x++) {
            for (auto y = domain.myLowerBound[1];  y <= domain.myUpperBound[1]; y++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(x, y, domain.myLowerBound[2]) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(x, y, domain.myUpperBound[2]) );

              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }
            for (auto z = domain.myLowerBound[2]; z <= domain.myUpperBound[2]; z++) {
              DGtal::Z3i::RealPoint rp1 = embedder.embed( DGtal::Z3i::Point(x, domain.myLowerBound[1], z) );
              DGtal::Z3i::RealPoint rp2 = embedder.embed( DGtal::Z3i::Point(x, domain.myLowerBound[1], z) );

              currentData->vertices.push_back(rp1);
              currentData->vertices.push_back(rp2);
            }
          }
          currentData = nullptr;
        }
        return name;
      }
      
      template<typename Obj, typename Cont>
      std::string draw(const DigitalSetByAssociativeContainer<Obj, Cont>& set) {
        return drawGenericObject("Set_{i}", set);
      }
      
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV,
                typename TFunctorVm1>
      std::string draw(const ImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV, TFunctorVm1>& adapter) {
        return drawImage(adapter);
      }
            
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV>
      std::string draw(const ConstImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV>& adapter) {
        return drawImage(adapter);
      }

      template<typename Adj, typename Set>
      std::string draw(const DGtal::Object<Adj, Set>& obj) {
        std::string name = drawGenericObject("Object_{i}", obj);

        // Draw adjacency if needed
        if (currentStyle.mode & DisplayStyle::DrawMode::ADJACENCIES) {
          newLineList(name + "_adj");
          
          for (auto it = obj.begin(); it != obj.end(); ++it) {
              auto neig = obj.properNeighborhood(*it);

              const Vector p = embedder.embed(*it);
              for (auto it2 = neig.begin(); it2 != neig.end(); ++it2) {
                auto p2 = embedder.embed(*it2);

                currentData->vertices.push_back(p);
                currentData->vertices.push_back(p2);
              }
          }

          currentData = nullptr;
        }
        return name;
      }
      
      template<typename T, typename Type>
      std::string draw(const WithProperty<T, Type>& props) {
        std::string name = draw(props.object);
        std::cout << name << std::endl;

        if constexpr (std::is_scalar_v<Type>) {
          auto& loc = data[name].scalarProperties[props.name];
          loc.insert(loc.end(), props.values.begin(), props.values.end());
        }
        else if constexpr(std::is_same_v<Vector, Type>) {
          auto& loc = data[name].vectorProperties[props.name];
          loc.insert(loc.end(), props.values.begin(), props.values.end());
        }
        else if constexpr(std::is_same_v<Color , Type>) {
          auto& loc = data[name].colorProperties[props.name];
          loc.insert(loc.end(), props.values.begin(), props.values.end());
        }
        else {
          trace.error() << "Unknown property type (for: '" << name << "')\n";
        }
        return name;
      }
      
      std::string draw(const ClippingPlane& plane) {
        planes.push_back(plane);
        return "";
      }

      std::string draw(const DGtal::Color& color) {
        drawColor(color);
        return "";
      }

      void drawColor(const DGtal::Color& color) {
        currentStyle.color = color;
        currentStyle.useDefaultColors = false;
      }
      
      void drawAdjacencies(bool toggle = true) {
        if (toggle) currentStyle.mode |=  DisplayStyle::ADJACENCIES;
        else        currentStyle.mode &= ~DisplayStyle::ADJACENCIES;
      }

      void drawAsGrid(bool toggle = true) {
        if (toggle) currentStyle.mode |=  DisplayStyle::GRID;
        else        currentStyle.mode &= ~DisplayStyle::GRID;
      }

      void drawAsPaving() {
        currentStyle.mode &= ~DisplayStyle::BALLS;
        currentStyle.mode |= DisplayStyle::PAVING;
      }

      void drawAsBalls() {
        currentStyle.mode &= ~DisplayStyle::PAVING;
        currentStyle.mode |=  DisplayStyle::GRID;
      }

    private:
      template<typename Obj>
      std::string drawGenericObject(const std::string& name, const Obj& obj) {
        std::string newName; 
        if (currentStyle.mode & DisplayStyle::BALLS) {
          newName = newBallList(name);

          currentData->vertices.reserve(obj.size());
          for (auto it = obj.begin(); it != obj.end(); ++it) {
            const auto rp = embedder.embed(*it);
            currentData->vertices.push_back(rp);
          }
        }
        else {
          newName = newCubeList(name);
        
          currentData->vertices.reserve(obj.size() * 8);
          for (auto it = obj.begin(); it != obj.end(); ++it) {
            const auto rp = embedder.embed(*it);
            const auto vertices = drawutils::getCubeVertices(rp, currentData->style.width);

            currentData->vertices.insert(
                currentData->vertices.end(), vertices.begin(), vertices.end()
            );
          }
        }

        currentData = nullptr;
        return newName;
      }

      template<typename T>
      std::string drawImage(const T& image) {
        std::string name = newCubeList("Image_{i}");

        size_t total = image.domain().size();

        auto it = image.domain().begin();
        auto itend = image.domain().end();
        
        currentData->vertices.reserve(8 * total);
        currentData->scalarProperties["value"].reserve(total);
        for(; it != itend; ++it) {
          auto rp = embedder.embed(*it);
          currentData->scalarProperties["value"].push_back(image(*it));
          drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
        }
        return name;
      }

      std::string addKCell(const Vector& rp, bool xodd, bool yodd, bool zodd, bool hasSign, bool sign) {
        std::string name = currentName;
        static const double scale = 0.9;
        static const double shift = 0.2;
        static const double smallScale = scale * scale;
        static const double smallShift = 2 * shift;

        const unsigned int dim = xodd + yodd + zodd;
        
        switch(dim) {
          case 0: {
            if (shouldCreateNewList(1)) {
              name = newBallList("KCell_{i}_d0");
              currentData->style.width *= scale;
            }

            currentData->vertices.push_back(rp);
          }
          break;
          case 1: {
            if (shouldCreateNewList(2)) {
              name = newLineList("KCell_{i}_d1");
              currentData->style.width *= scale;
            }

            const Vector shift(xodd, yodd, zodd);
            currentData->vertices.push_back(rp - 0.5 * shift);
            currentData->vertices.push_back(rp + 0.5 * shift);
          }
          break;
          case 2: {
            const unsigned int orientation = (!xodd ? 0 : (!yodd ? 1 : 2));
            if (shouldCreateNewList(8)) {
              name = newVolumetricList("KCell_{i}_d2");
              currentData->style.width *= scale;
            }
            
            double scale1 = scale      * currentData->style.width;
            double scale2 = smallScale * currentData->style.width;
            double shift1 = shift;
            double shift2 = smallShift;
            if (hasSign && sign) 
              std::swap(shift1, shift2);

            drawutils::insertPrism(currentData->vertices, orientation, scale, smallScale, shift1, shift2);
          }
          break;
          case 3: {
            if (shouldCreateNewList(1)) {
              name = newCubeList("KCell_{i}_d3");
              currentData->style.width *= scale;
            }

            drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
          };
          break;
        };

        return name;
      }

    public:
      /**
       * @brief Set the current group for further updates
       */
      bool setCurrentList(const std::string& name) {
        auto it = data.find(name);
        if (it == data.end()) 
          return false;
        
        currentData = &it->second;
        return true;
      }
      
      std::string newCubeList(const std::string& name)       { return newList(name, 8); }
      std::string newBallList(const std::string& name)       { return newList(name, 1); }
      std::string newLineList(const std::string& name)       { return newList(name, 2); }
      std::string newQuadList(const std::string& name)       { return newList(name, 4); }
      std::string newPolygonList(const std::string& name)    { return newList(name, 0); }
      std::string newTriangleList(const std::string& name)   { return newList(name, 3); }
      std::string newVolumetricList(const std::string& name) { return newList(name, 8); }

    private:
      bool shouldCreateNewList(size_t expectedSize) const {
        if (!currentData) return true;
        if (currentData->elementSize != expectedSize) return true;
        return !allowReuseList;
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
      std::string newList(const std::string& name, size_t eSize = 0) {
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
        DisplayData<Vector> newData;
        newData.style = currentStyle;
        newData.elementSize = eSize;

        currentData = &data.emplace(newName, std::move(newData)).first->second;
        currentName = newName;
        return newName;
      }
    
    public:
      KSpace kspace;
      Embedder embedder;
      CellEmbedder cellEmbedder;
      SCellEmbedder sCellEmbedder;

      DisplayStyle currentStyle;
      bool allowReuseList = false;
    protected:
      Callback* callback = nullptr;

      std::vector<ClippingPlane> planes;
      std::map<std::string, DisplayData<Vector>> data;

      std::string currentName = "";
      DisplayData<Vector>* currentData = nullptr;
    };
}

