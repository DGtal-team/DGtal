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

#include <Eigen/Geometry>

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/io/Color.h"
#include "DGtal/kernel/CanonicEmbedder.h" 

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"

// Objects to draw
#include "DGtal/base/ConstRangeAdapter.h"

#include "DGtal/shapes/Mesh.h"

#include "DGtal/topology/Object.h"

#include "DGtal/images/ImageAdapter.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/geometry/tools/SphericalAccumulator.h"

#include "DGtal/geometry/curves/StandardDSS6Computer.h"
#include "DGtal/geometry/curves/GridCurve.h"

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

      template<typename U, typename T>
      void insertQuad(U& dest, T center, int orientation, double size) {
        auto vertices = getAAQuadVertices(center, orientation, size);
        dest.insert(dest.end(), vertices.begin(), vertices.end());
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

    template<typename RealPoint>
    struct DisplayData {
      std::size_t           elementSize; //< Size for each elements. If 0, elements may have variable size (ie. polygonal faces).
                                         // 1 -> Ball or Cube depending on DrawMode
                                         // 2 -> Lines
                                         // 0, 3, 4 -> Polygonal / Triangular / Quad mesh
                                         // 8 -> Volumetric mesh
      std::vector<std::vector<uint32_t>> indices;     //< Indices for each elements. Only used if elementSize is 0.
      std::vector<RealPoint>   vertices;    //< Vertices of the object

      Eigen::Affine3d transform = Eigen::Affine3d::Identity(); //< Transform (includes scale)

      DisplayStyle style;
      std::map<std::string, std::vector<RealPoint>> vectorProperties;
      std::map<std::string, std::vector<double>> scalarProperties;
      std::map<std::string, std::vector<Color>>  colorProperties;
    };
    
    struct ClippingPlane {
      double a, b, c; //< Normal components
      double d;       //< Offset
                      //
      DisplayStyle style;
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
    class Display3D {
    public:
      Display3D(const KSpace& space) :
        kspace(space), 
        cellEmbedder(kspace),
        sCellEmbedder(kspace)
      { }

      Display3D() : Display3D(KSpace()) {}

      using Point = typename Space::Point;
      using KCell = typename KSpace::Cell;
      using SCell = typename KSpace::SCell;
      using RealPoint = typename Space::RealPoint ;

      using Embedder = CanonicEmbedder<Space>;
      using CellEmbedder = CanonicCellEmbedder<KSpace>;
      using SCellEmbedder = CanonicSCellEmbedder<KSpace>;

      struct Callback{
        virtual void OnAttach(void* viewer) {};
        virtual void OnUI() {};
        virtual void OnClick(const std::string& name, size_t index, const DisplayData<RealPoint>& data, void* viewerData) {};

        Display3D<Space, KSpace>* viewer;
      };

    public:
      virtual void renderNewData() = 0;
      virtual void clearView() = 0;
      virtual void show() = 0;

      virtual void clear() {
        toRender.clear();
        planes.clear();
        data.clear();
        
        noCurrentGroup();
      }

      template<typename Obj>
      Display3D& operator<<(const Obj& obj) {
        draw(obj);
        return *this;
      }

      void setCallback(Callback* callback) {
        this->callback = callback;
        this->callback->viewer = this;
      }

      std::string draw(const Point& p, const std::string& uname = "Point_{i}") {
        return draw(embedder.embed(p), uname);
      }

      std::string draw(const RealPoint& rp, const std::string& uname = "Point_{i}") {
        std::string name = currentName;

        if (currentStyle.mode &= DisplayStyle::BALLS) {
          if (shouldCreateNewList(1)) {
            name = newBallList(uname);
          }
          
          currentData->vertices.push_back(rp);
        } else {
          if (shouldCreateNewList(8)) {
            name = newCubeList(uname);
          }
          
          drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
        }
        return name;
      }
      
      std::string draw(const std::pair<RealPoint, RealPoint>& arrow, const std::string& uname = "Arrow_{i}") {
        // Warning, this function draw arrows, not lines !
        std::string name = currentName;
        if (shouldCreateNewList(1)) {
          name = newBallList(uname);
          currentData->style.width = 0;
        }
        currentData->vertices.push_back(arrow.first);
        currentData->vectorProperties["data"].push_back(arrow.second);
        return "";
      }

      template<typename Range> 
      std::string drawGenericRange(const Range& range, const std::string& uname) {
        bool save = allowReuseList;

        auto it = range.begin();

        std::string name = currentName; 
        allowReuseList = false; // Force new group !
        if (uname.empty()) {
          name = draw(*it);
        } else {
          name = draw(*it, uname);
        }
        allowReuseList = true; // Reuse group

        for (++it; it != range.end(); ++it) {
          draw(*it, name);
        }

        noCurrentGroup();
        allowReuseList = save; 
        return name;
      }

      template<typename A, typename B, typename C>
      std::string draw(const ConstRangeAdapter<A, B, C> range, const std::string& uname = "") {
        return drawGenericRange(range, uname);
      }

      template<typename A, typename B, typename C>
      std::string draw(const ConstIteratorAdapter<A, B, C>& adapter, const std::string& uname = "") {
        if (uname.empty()) {
          return draw(*adapter);
        }
        // Use default value of draw
        return draw(*adapter, uname);
      }
      
      std::string draw(const GridCurve<KSpace>& curve, const std::string& uname = "GridCurve_{i}") {
        return draw(curve.getSCellsRange(), uname);
      }
      std::string draw(const typename GridCurve<KSpace>::MidPointsRange& range, const std::string& uname = "MidPoints_{i}") {
        return drawGenericRange(range, uname);
      }
      std::string draw(const typename GridCurve<KSpace>::ArrowsRange& range, const std::string& uname = "Arrows_{i}") {
        return drawGenericRange(range, uname);
      }

      std::string draw(const KCell& cell, const std::string& name = "KCell_{i}_{d}d") {
        const RealPoint rp = cellEmbedder.embed(cell);
        
        const bool xodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[0]) & 1);
        const bool yodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[1]) & 1);
        const bool zodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[2]) & 1);
          
        return addKCell(name, rp, xodd, yodd, zodd, false, false);
      }
      
      std::string draw(const SCell& cell, const std::string& name = "SCell_{i}_{d}d") {
        const RealPoint rp = sCellEmbedder.embed(cell);
        
        const bool xodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[0]) & 1);
        const bool yodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[1]) & 1);
        const bool zodd = (NumberTraits<typename KSpace::Integer>::castToInt64_t(cell.preCell().coordinates[2]) & 1);
        
        return addKCell(name, rp, xodd, yodd, zodd, true, cell.preCell().positive);
      }

      std::string draw(const HyperRectDomain<Space>& domain, const std::string& uname = "Domain_{i}") {
        std::string name = drawGenericObject(uname, domain);

        if (currentStyle.mode & DisplayStyle::GRID) {
          newLineList(name + "_grid");
          
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
        }

        noCurrentGroup();
        return name;
      }
    
      template<typename Vec>
      std::string drawPolygon(const std::vector<Vec>& vertices, const std::string& uname = "Polygon_{i}") {
        std::string name = currentName;
        if (shouldCreateNewList(0) /* means variable number of vertices */) {
          name = newPolygonList(uname);
        }
        
        std::vector<unsigned> indices;
        indices.reserve(vertices.size());

        size_t count = currentData->vertices.size();
        for (const auto& vert : vertices) {
          currentData->vertices.push_back(vert);
          indices.push_back(count++);
        }
        currentData->indices.push_back(std::move(indices));
        return name;
      }

      std::string drawBall(const RealPoint& c, const std::string& uname = "Ball_{i}") {
        std::string name = currentName;
        if (shouldCreateNewList(1)) {
          name = newBallList(uname);
        }
        currentData->vertices.push_back(c);
        return name;
      }
      
      std::string drawLine(const RealPoint& a, const RealPoint& b, const std::string& uname = "Line_{i}") {
        std::string name = currentName;
        if (shouldCreateNewList(2)) {
          name = newLineList(uname);
        }

        currentData->vertices.push_back(a);
        currentData->vertices.push_back(b);
        return name;
      }

      std::string drawQuad(const RealPoint& a, const RealPoint& b, const RealPoint& c, const RealPoint& d, const std::string& uname = "Quad_{i}") {
        // Not a draw specialization as it would be confusing with a drawing call of
        // an array of points, or other primitives
        std::string name = currentName;
        if (shouldCreateNewList(4)) {
          name = newQuadList(uname);
        }

        currentData->vertices.push_back(a);
        currentData->vertices.push_back(b);
        currentData->vertices.push_back(c);
        currentData->vertices.push_back(d);

        return name;
      }
      
      template<typename Obj, typename Cont>
      std::string draw(const DigitalSetByAssociativeContainer<Obj, Cont>& set, const std::string& name = "Set_{i}") {
        return drawGenericObject(name, set);
      }

      template<typename D, typename T>
      std::string draw(const ImageContainerBySTLVector<D, T>& image, const std::string& name = "Image_{i}") {
        return drawImage(name, image);
      }
      
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV,
                typename TFunctorVm1>
      std::string draw(const ImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV, TFunctorVm1>& adapter, const std::string& name = "Image_{i}") {
        return drawImage(name, adapter);
      }
            
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV>
      std::string draw(const ConstImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV>& adapter, const std::string& name = "Image_{i}") {
        return drawImage(name, adapter);
      }

      template<typename Adj, typename Set>
      std::string draw(const DGtal::Object<Adj, Set>& obj, const std::string& uname = "Object_{i}") {
        std::string name = drawGenericObject(uname, obj);

        // Draw adjacency if needed
        if (currentStyle.mode & DisplayStyle::DrawMode::ADJACENCIES) {
          newLineList(name + "_adj");
          
          for (auto it = obj.begin(); it != obj.end(); ++it) {
              auto neig = obj.properNeighborhood(*it);

              const RealPoint p = embedder.embed(*it);
              for (auto it2 = neig.begin(); it2 != neig.end(); ++it2) {
                auto p2 = embedder.embed(*it2);

                currentData->vertices.push_back(p);
                currentData->vertices.push_back(p2);
              }
          }
        }
        noCurrentGroup();
        return name;
      }
      
      template<typename T, typename Type>
      std::string draw(const WithProperty<T, Type>& props, const std::string& uname = "") {
        std::string name;
        if (uname.empty())
          name = draw(props.object);
        else
          name = draw(props.object, uname);

        if constexpr (std::is_scalar_v<Type>) {
          auto& loc = data[name].scalarProperties[props.name];
          loc.insert(loc.end(), props.values.begin(), props.values.end());
        }
        else if constexpr(std::is_same_v<RealPoint, Type>) {
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
      
      template <typename Pt>
      std::string draw(const Mesh<Pt>& mesh, const std::string& uname = "Mesh_{i}") {
        // A mesh may have quad faces, therefore we render it as a polygonal mesh
        std::string name = newPolygonList(uname);

        currentData->vertices.reserve(mesh.nbVertex());

        currentData->indices.reserve(mesh.nbFaces());
        currentData->colorProperties["color"].reserve(mesh.nbFaces());

        // Can not insert directly vectors because of type mismatch
        for (auto it = mesh.vertexBegin(); it != mesh.vertexEnd(); ++it) {
          currentData->vertices.push_back(*it);
        }
        for (size_t i = 0; i < mesh.nbFaces(); ++i) {
          const auto& face = mesh.getFace(i);
          std::vector<unsigned int> fIdx;
          fIdx.reserve(face.size());
          for (auto i : face) {
            fIdx.push_back(i);
          }
          currentData->indices.push_back(std::move(fIdx));
          currentData->colorProperties["color"].push_back(mesh.getFaceColor(i));
        }
        noCurrentGroup();
        return name;
      }

      template<typename It, typename Int, int Con>
      std::string draw(const StandardDSS6Computer<It, Int, Con>& computer, const std::string& uname = "Computer_{i}") {
        std::string name;
        if (currentStyle.mode & DisplayStyle::BALLS) {
          name = newBallList(uname);

          for (auto it = computer.begin(); it != computer.end(); ++it) {
            const auto rp = embedder.embed(*it);
            currentData->vertices.push_back(rp);
          }
        } else { // Default mode
          name = newLineList(uname);

          auto it = computer.begin();
          RealPoint a = embedder.embed(*it);
          RealPoint b = a;

          for (++it; it != computer.end(); ++it) {
            b = embedder.embed(*it);
            currentData->vertices.push_back(a);
            currentData->vertices.push_back(b);

            std::swap(a, b);
          }
        }
        
        noCurrentGroup();
        return name;
      }
      
      std::string draw(const ClippingPlane& plane, const std::string& name = "") {
        planes.push_back(plane);
        planes.back().style = currentStyle;
        return "";
      }

    
      template<typename T>
      std::string draw(const SphericalAccumulator<T> accumulator, const std::string& uname = "SphericalAccumulator_{i}") {
        std::string name = newQuadList(uname);

        typedef typename SphericalAccumulator<T>::Size Size;
        typedef typename SphericalAccumulator<T>::RealVector Vec;

        Size i, j;
        Vec a, b, c, d;
        for (auto it = accumulator.begin(); it != accumulator.end(); ++it) {
          accumulator.binCoordinates(it, i, j);

          if (accumulator.isValidBin(i, j)) {
            accumulator.getBinGeometry(i, j, a, b, c, d);

            currentData->vertices.push_back(a);
            currentData->vertices.push_back(b);
            currentData->vertices.push_back(c);
            currentData->vertices.push_back(d);
            currentData->scalarProperties["value"].push_back(accumulator.count(i, j));
          }
        }
        return name;
      }

      std::string draw(const DGtal::Color& color, const std::string& name = "") {
        drawColor(color);
        return "";
      }
      
      void drawColor(const DGtal::Color& color) {
        currentStyle.color = color;
        currentStyle.useDefaultColors = false;
      }

      void setDefaultColors() {
        currentStyle.useDefaultColors = true;
      }
      
      void drawAdjacencies(bool toggle = true) {
        if (toggle) currentStyle.mode |=  DisplayStyle::ADJACENCIES;
        else        currentStyle.mode &= ~DisplayStyle::ADJACENCIES;
      }

      void drawAsGrid(bool toggle = true) {
        if (toggle) currentStyle.mode |=  DisplayStyle::GRID;
        else        currentStyle.mode &= ~DisplayStyle::GRID;
      }
      
      void defaultStyle() {
        currentStyle.mode = DisplayStyle::DEFAULT;
      }

      void drawAsPaving() {
        currentStyle.mode &= ~DisplayStyle::BALLS;
        currentStyle.mode |= DisplayStyle::PAVING;
      }

      void drawAsBalls() {
        currentStyle.mode &= ~DisplayStyle::PAVING;
        currentStyle.mode |=  DisplayStyle::BALLS;
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
        } else {
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

        noCurrentGroup();
        return newName;
      }

      template<typename T>
      std::string drawImage(const std::string& uname, const T& image) {
        std::string name = newCubeList(uname);

        size_t total = image.domain().size();

        auto it = image.domain().begin();
        auto itend = image.domain().end();
        constexpr size_t dim = T::Domain::Space::dimension;
        
        currentData->vertices.reserve(8 * total);
        currentData->scalarProperties["value"].reserve(total);
        for(; it != itend; ++it) {
          if constexpr (dim == 3) {
            auto rp = embedder.embed(*it);
            currentData->scalarProperties["value"].push_back(image(*it));
            drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
          } else {
            // We accept to draw theses 2D image, do ask to parametrize to also the embedder...
            auto rp = embedder.embed(Point((*it)[0], (*it)[1], 0)); 
            currentData->scalarProperties["value"].push_back(image(*it));
            drawutils::insertCubeVertices(currentData->vertices, rp, currentData->style.width);
          }
        }
        return name;
      }

      std::string addKCell(std::string uname, const RealPoint& rp, bool xodd, bool yodd, bool zodd, bool hasSign, bool sign) {
        std::string name = currentName;
        static const std::string TOKEN = "{d}";
        static const double scale = 0.9;
        static const double shift = 0.05;
        static const double smallScale = 0.3;
        static const double smallShift = 0.15;
        // For 2D cell, this indicates if the big quad is 
        // inside the cell or outside
        static const int orientationPermut[3][2] = {
          {1, 0}, {0, 1}, {1, 0}
        };

        const unsigned int dim = xodd + yodd + zodd;

        auto tokenPos = uname.find(TOKEN);
        if (tokenPos != std::string::npos) 
          uname.replace(uname.find(TOKEN), TOKEN.size(), std::to_string(dim));

        switch(dim) {
          case 0: {
            if (shouldCreateNewList(1)) {
              name = newBallList(uname);
              currentData->style.width *= scale;
            }

            currentData->vertices.push_back(rp);
          }
          break;
          case 1: {
            if (shouldCreateNewList(2)) {
              name = newLineList(uname);
              currentData->style.width *= scale;
            }

            const RealPoint shift(xodd, yodd, zodd);
            currentData->vertices.push_back(rp - 0.5 * shift);
            currentData->vertices.push_back(rp + 0.5 * shift);
          }
          break;
          case 2: {
            const unsigned int orientation = (!xodd ? 0 : (!yodd ? 1 : 2));
            if (currentStyle.mode & DisplayStyle::SIMPLIFIED || !hasSign) {
              if (shouldCreateNewList(4)) {
                name = newQuadList(uname);
                currentData->style.width *= scale;
              }
              
              double scale1 = currentData->style.width;
              drawutils::insertQuad(currentData->vertices, rp, orientation, scale1);
            } else {
              if (shouldCreateNewList(8)) {
                name = newVolumetricList(uname);
                currentData->style.width *= scale;
              }
              
              const double scales[2] = {
                scale      * currentData->style.width, 
                smallScale * currentData->style.width
              };
              
              // Decide where the big quad goes, in the interior or the exterior
              // of the cell depending on sign and the orientation
              int permut = orientationPermut[orientation][sign];
              double scale1 = scales[    permut];
              double scale2 = scales[1 - permut];
              double shift1 = shift;
              double shift2 = smallShift;

              drawutils::insertPrism(currentData->vertices, rp, orientation, scale1, scale2, shift1, shift2);
            }
          }
          break;
          case 3: {
            if (shouldCreateNewList(8)) {
              name = newCubeList(uname);
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
        currentName = name;
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

      void noCurrentGroup() {
        currentData = nullptr;
        currentName = "";
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
        DisplayData<RealPoint> newData;
        newData.style = currentStyle;
        newData.elementSize = eSize;

        currentData = &data.emplace(newName, std::move(newData)).first->second;
        currentName = newName;
        toRender.push_back(currentName);
        return newName;
      }
    
    public:
      // The use is responsible for not using these wrong...
      DisplayStyle currentStyle;
      bool allowReuseList = false;

      std::vector<ClippingPlane> planes;
      std::map<std::string, DisplayData<RealPoint>> data;
    protected:
      KSpace kspace;
      Embedder embedder;
      CellEmbedder cellEmbedder;
      SCellEmbedder sCellEmbedder;

      Callback* callback = nullptr;
      std::vector<std::string> toRender;

      std::string currentName = "";
      DisplayData<RealPoint>* currentData = nullptr;
    };
}

