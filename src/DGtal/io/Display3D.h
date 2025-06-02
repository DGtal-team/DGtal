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
 * @file Display3D.h
 * @author Bastien Doignies <bastien.doignies@liris.cnrs.fr>
 *
 * @date 2025/05/11
 *
 * Header file for 3D Display
 *
 * This file is part of the DGtal library.
 */

#include <cstdint>
#include <vector>
#include <map>

#include "DGtal/dec/DiscreteExteriorCalculus.h"

#include "Eigen/Geometry"

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/io/Color.h"
#include "DGtal/kernel/CanonicEmbedder.h" 

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"

// Objects to draw
#include "DGtal/base/ConstRangeAdapter.h"

#include "DGtal/shapes/Mesh.h"

#include "DGtal/dec/DiscreteExteriorCalculus.h"

#include "DGtal/topology/Object.h"

#include "DGtal/images/ImageAdapter.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include "DGtal/geometry/curves/StandardDSS6Computer.h"
#include "DGtal/geometry/curves/GridCurve.h"

namespace DGtal {
    namespace drawutils { // Namespace for some utilities
      /**
       * @brief Create a list of indices for a vertex array with independent elements
       * 
       * @tparam I The size of individual elements
       * @param N The number of elements
       */
      template<size_t I>
      std::vector<std::array<size_t, I>> makeIndices(size_t N);
      
      /**
       * @brief Return the vertices of a cube
       *
       * @tparam T The type of vertex
       * @param center The center of the cube
       * @param size The size of the cube
       *
       * @see DGtal::drawutils::insertCubeVertices
       */
      template<typename T> 
      std::array<T, 8> getCubeVertices(T center, double size);
      
      /**
       * @brief Insert cube vertices into an array
       *
       * @tparam T The type of vertex
       * @tparam U The container to insert vertices into
       * 
       * @param dest The container where the vertices shoudl be inserted
       * @param center The center of the cube
       * @param scale The size of the cube
       *
       * @see DGtal::drawutils::getCubeVertices
       */
      template<typename T, typename U>
      void insertCubeVertices(U& dest, T center, double scale);
      
      /**
       * @brief Return the vertices of an axis aligned square 
       *
       * @tparam T The type of vertex
       * 
       * @param center The center of the quad
       * @param orientation 0 means normal in x direction, 1 in y-direction, 2 in z-direction
       * @param size The size of the square
       *
       * @see DGtal::drawutils::insertAASquare
       */
      template <typename T>
      std::array<T, 4> getAASquareVertices(T center, int orientation, double size);
      
      /**
       * @brief Insert vertices of a square into a container
       *
       * @tparam U The container
       * @tparam T The type of vertex
       *
       * @param dest The container where the vertices shoudl be inserted
       * @param center The center of the square
       * @param orientation 0 means normal in x direction, 1 in y-direction, 2 in z-direction
       * @param size The size of the square
       *
       * @see DGtal::drawutils::getAASquareVertices
       */
      template<typename U, typename T>
      void insertAASquare(U& dest, T center, int orientation, double size);
      
      /**
       * @brief Return the vertices of a prism
       *
       * Here, a prism is meant to display a signed Khalimsky cell. 
       * It is draw as two square, one of which is smaller than the other, 
       * connected by 4 other rectular shapes. 
       *
       * @tparam T The vertex type
       *
       * @param center The center of the cell on which the prism should be drawn
       * @param orientation The orientation of the cell (x, y, z)
       * @param size1 Size of the first square
       * @param size2 Size of the second square
       * @param shift1 Shift (relative to center) of the first square
       * @param shift2 Shift (relative to center) of the second square
       */
      template<typename T>
      std::array<T, 8> getPrism(
          T center, int orientation, 
          double size1, double size2, double shift1, double shift2
      );
      
      /**
       * @brief Insert the vertices of a prism into a container
       *
       * @see DGtal::drawutils::getPrism
       *
       * @tparam U The container type
       * @tparam T The vertex type
       *
       * @param dest The container where the vertices shoudl be inserted
       * @param center The center of the cell on which the prism should be drawn
       * @param orientation The orientation of the cell (x, y, z)
       * @param size1 Size of the first square
       * @param size2 Size of the second square
       * @param shift1 Shift (relative to center) of the first square
       * @param shift2 Shift (relative to center) of the second square
       */ 
      template<typename T, typename U>
      void insertPrism(U& dest, T center, int orientation, 
          double size1, double size2, double shift1, double shift2);
    } // drawutils

    /**
     * @brief Style of display of an element
     */
    struct DisplayStyle {
      // Color of an object
      Color color = Color(200, 200, 200, 255);
      // When set, color is ignored and the viewer is free to choose
      bool useDefaultColors = true;            
      // Maintains uniform scale of the object independently for easy access
      double width = 1.0;                      

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
    
    // Small trick to store scales. Enums is not convertible to int anymore. Using a map for quantities would work, but it does not create element, hence would add code to check if the scale alreay exists.
    
    namespace QuantityScale {
      static constexpr int VERTEX = 0;
      static constexpr int EDGES = 1;
      static constexpr int FACES = 2;
      static constexpr int CELL = 3;
      static constexpr int UNKNOWN = 4;
    }
 
    template<typename T>
    using Quantity = std::array<
      std::map<std::string, std::vector<T>>, QuantityScale::UNKNOWN
    >;

    /**
     * @brief Data required to display an object
     * 
     * @tparam RealPoint Vector type for points and vector
     */
    template<typename RealPoint>
    struct DisplayData {
      /**
       * Size of each elements. 
       * 
       * 0: elements may have variable size given by DisplayData::Indices (polygonal mesh)
       * 1: elements are drawn as points (Ball)
       * 2: elements are drawn as lines
       * 3: elements are drawn as triangle (mesh)
       * 4: elements are drawn as quads (mesh)
       * 8: elements are drawn as cubes / prisms (volumetric mesh)
       */
      std::size_t           elementSize; 
      
      // Return the default quantity level associated with 
      // a given element size.
      static constexpr auto getDefaultQuantityLevel = [](size_t eSize) {
        switch(eSize) {
          case 1: return QuantityScale::VERTEX;
          case 2: return QuantityScale::EDGES;
          case 0: // Polygonal meshes
          case 3: // Triangle meshes
          case 4: // Quad meshes
                  return QuantityScale::FACES;
          case 8: return QuantityScale::CELL;
          default: 
                  return QuantityScale::UNKNOWN;
        };
      };
      
      // Indices for elements when elementSize is 0
      std::vector<std::vector<uint32_t>> indices;
      
      // List of vertices
      std::vector<RealPoint>   vertices;

      // Transform of the object
      Eigen::Affine3d transform = Eigen::Affine3d::Identity(); 
      
      // Draw style of the object (color, scale)
      DisplayStyle style;
      
      // Color to apply to each element
      Quantity<Color>  colorQuantities;
      // Vector to attach to each element
      Quantity<RealPoint> vectorQuantities;
      // Values to attach to each element (may serve for coloring)
      Quantity<double> scalarQuantities;
    };
    
    /**
     * @brief Clipping plane
     */
    struct ClippingPlane {
      double a, b, c; //< Normal components
      double d;       //< Offset
      
      // Some style for rendering (colors)
      DisplayStyle style;
    };
    
    /**
     * @brief Attach a property to an element
     *
     * This class can be used with singletons and composite elements; 
     * as long as the correct number of values are provided.
     * 
     * This class can be nested to add multiple properties:
     * ```code
     *  WithQuantity(
     *    WithQuantity(
     *      obj, "value", scalar
     *    ), 
     *    "normal", normal
     *  )
     * 
     * @tparam T The type of element
     * @tparam Type the type of property
     */
    template<typename T, typename Type>
    struct WithQuantity {
      WithQuantity(const T& object, const std::string& name, const Type& value, int s = QuantityScale::UNKNOWN) : 
        scale(s), object(object), name(name) 
      {
        values.push_back(value);
      }
      
      WithQuantity(const T& object, const std::string& name, const std::vector<Type>& values, int s = QuantityScale::UNKNOWN) : 
        scale(s), object(object), name(name) 
      {
        this->values = values;
      }
      
      // Scale to apply the quantity at
      int scale;

      // Copy of the object
      T object;
      // Copy of the values to attach
      std::vector<Type> values;
      // Name of the property
      std::string name;
    };

    /**
     * @brief Base class for viewing DGtal objects
     *
     * @tparam Space Space of draw objects
     * @tparam KSpace Khalimsky space of drawn objects
     * 
     * Drawing:
     * 
     * Lists:
     *  Lists are a key part of this class. A list is a collection of object
     *  that are linked together. As such, they should be managed, updated 
     *  with the same options.
     *  
     *  Each DisplayData elements corresponds to a list of elements. However
     *  the intent behind subsequent drawcalls can be to groups elements 
     *  together. For example, looping through a Digital surface doing a
     *  computation and then display the current element. 
     *  For this reason, the field "allowReuseList" can be set to true. This
     *  will try, if possible to use reuse the current element if possible. 
     *  This may not success (common cases are if elements of different size 
     *  were inserted, or after drawing an object which always create its own
     *  list). 
     *  If this does not fit the need, use setCurrentList.
     *  
     */
    template < typename Space = Z3i::Space, typename KSpace = Z3i::KSpace>
    class Display3D {
    public:
      Display3D(const KSpace& space) :
        myKSpace(space), 
        myCellEmbedder(myKSpace),
        mySCellEmbedder(myKSpace)
      { }

      Display3D() : Display3D(KSpace()) {}

      // Usefull definitions
      using Point = typename Space::Point;
      using KCell = typename KSpace::Cell;
      using SCell = typename KSpace::SCell;
      using RealPoint = typename Space::RealPoint ;

      using Embedder = CanonicEmbedder<Space>;
      using CellEmbedder = CanonicCellEmbedder<KSpace>;
      using SCellEmbedder = CanonicSCellEmbedder<KSpace>;

      /**
       * @brief A general callback for the viewer to give control to the user
       * 
       * There are no guarentees on the thread-safeness of this class. 
       */
      struct Callback{
        /**
         * @brief Called when setCallback is performed on the viewer
         * 
         * This is callback can be used to store the pointer to a particular
         * instance of the viewer.
         * 
         * If only general properties are needed, use the Callback::viewer member
         * which is set prior calling this function. 
         * 
         * @param viewer A pointer to the viewer on which this callback is attached
         */
        virtual void OnAttach(void* viewer) {};
        /**
         * @brief Called to render or interact with some UI
         *
         * @param viewerData Some viewer-dependent data to draw UI (context)
         */
        virtual void OnUI(void* viewerData) {};
        /**
         * @brief Called when an element is clicked
         * 
         * @see DGtal::Display3D::renderNewData
         * 
         * @param name The name of the structure containing the element
         * @param index The index within the clicked structure
         * @param data The Display3D data associated with this structure
         * @param viewerData Viewer-dependent data associated with the click
         */
        virtual void OnClick(const std::string& name, size_t index, const DisplayData<RealPoint>& data, void* viewerData) {};

        // Pointer to the Display3D instance the callback is attached to
        Display3D<Space, KSpace>* viewer = nullptr;
      };
    public: // General commands
      /**
       * @brief Starts the event loop and display of elements
       * 
       * It is recommended that this functions calls renderNewData
       */
      virtual void show() = 0;
      /**
       * @brief Renders newly added data
       */
      virtual void renderNewData() = 0;

      /**
       * @brief (Re)Render all data 
       *
       * If any modification were made within the viewer they can be lost
       */
      virtual void renderAll() {
        myToRender.clear();
        myToRender.reserve(data.size());

        for (const auto& m : data) 
          myToRender.push_back(m.first);

        renderNewData();
      }
      /**
       * @brief Clear the screen
       */
      virtual void clearView() = 0;
      /**
       * @brief Clear the viewer, including screen and internal data
       */
      virtual void clear();
      /** 
       * @brief Sets callback
       */
      virtual void setCallback(Callback* callback);

    public: // Group/Lists managements
      /**
       * @brief Create a new group
       *
       * If the name requested already exists; another one is
       * computed as follows:
       * - If the token {i} is present, replace the first occurence with 
       *   the first available index, starting from one.
       * - Otherwise, appends "_i" where i the first available index, 
       *   starting from one.
       *
       * This function sets the current data.
       * 
       * @param name The name to insert
       * @param eSize elementSize of the DisplayData 
       *
       * @return The computed name
       */
      std::string newList(const std::string& name, size_t eSize = 0);

      /**
       * @brief Set the current group for further updates
       * 
       * @param name The name of the group
       */
      bool setCurrentList(const std::string& name);

      /**
       * @brief Tells if a list of a given elementSize can be reused
       * 
       * @param elementSize The size of each elements
       */
      bool canCreateNewList(size_t elementSize) const;

      /**
       * @brief Reuse a list if possible, otherwise create a new one
       * 
       * @param name The name of the created list if needed
       * @param elementSize The size of each element for the list 
       * 
       * @return The name of the list to use
       */
      std::string createOrReuseList(const std::string& name, size_t elementSize);

      /**
       * @brief End current group and sets an invalid current group
       * 
       * The purpose of this function is to disallow further
       * automatic update on a group.
       */
      void endCurrentGroup();

      // Some shortcuts for clearer code

      std::string newCubeList(const std::string& name)       { return newList(name, 8); }
      std::string newBallList(const std::string& name)       { return newList(name, 1); }
      std::string newLineList(const std::string& name)       { return newList(name, 2); }
      std::string newQuadList(const std::string& name)       { return newList(name, 4); }
      std::string newPolygonList(const std::string& name)    { return newList(name, 0); }
      std::string newTriangleList(const std::string& name)   { return newList(name, 3); }
      std::string newVolumetricList(const std::string& name) { return newList(name, 8); }

      std::string createOrReuseCubeList(const std::string& name)       { return createOrReuseList(name, 8); }
      std::string createOrReuseBallList(const std::string& name)       { return createOrReuseList(name, 1); }
      std::string createOrReuseLineList(const std::string& name)       { return createOrReuseList(name, 2); }
      std::string createOrReuseQuadList(const std::string& name)       { return createOrReuseList(name, 4); }
      std::string createOrReusePolygonList(const std::string& name)    { return createOrReuseList(name, 0); }
      std::string createOrReuseTriangleList(const std::string& name)   { return createOrReuseList(name, 3); }
      std::string createOrReuseVolumetricList(const std::string& name) { return createOrReuseList(name, 8); }
    
    public: // Draw commands

      /**
       * @brief Draw object with stream API
       * 
       * @tparam Obj Any type of object
       */ 
      template<typename Obj>
      Display3D& operator<<(const Obj& obj);

      // @brief Draws a Point with integer coodinates
      std::string draw(const Point& p, const std::string& uname = "Point_{i}");

      // @brief Draws a RealPoint with real coodinates
      std::string draw(const RealPoint& rp, const std::string& uname = "Point_{i}");
      
      // @brief Draws a vector of objects
      template<typename T>
      std::string draw(const std::vector<T>& vec, const std::string& uname = "");

      // @brief Draws a range of any object
      template<typename A, typename B, typename C>
      std::string draw(const ConstRangeAdapter<A, B, C> range, const std::string& uname = "");

      // @brief Draws any object provided through a ConstIteratorAdapter
      template<typename A, typename B, typename C>
      std::string draw(const ConstIteratorAdapter<A, B, C>& adapter, const std::string& uname = "");
      
      // @brief Draws a grid curve
      std::string draw(const GridCurve<KSpace>& curve, const std::string& uname = "GridCurve_{i}");
      
      // @brief Draws a grid curve as mid points 
      std::string draw(const typename GridCurve<KSpace>::MidPointsRange& range, const std::string& uname = "MidPoints_{i}");
      
      // @brief Draws a grid curve as arrows
      std::string draw(const typename GridCurve<KSpace>::ArrowsRange& range, const std::string& uname = "Arrows_{i}");

      // @brief Draws a DiscreteExteriorCalculus
      template<DGtal::Dimension emb, DGtal::Dimension amb, typename Algebra, typename Int>
      std::string draw(const DiscreteExteriorCalculus<emb, amb, Algebra, Int>& calc, const std::string& uname = "Calculus_{i}");

      // @brief Draws a KForm
      template<typename Calculus, DGtal::Order order, DGtal::Duality duality>
      std::string draw(const KForm<Calculus, order, duality>& kform, const std::string& uname = "KForm_{i}");

      // @brief Draws a VectorField
      template<typename Calculus, DGtal::Duality dual> 
      std::string draw(const VectorField<Calculus, dual>& field, const std::string& uname = "Field_{i}");

      // @brief Draws an unsigned KCell
      std::string draw(const KCell& cell, const std::string& name = "KCell_{i}_{d}d");
      
      // @brief Draws a singed KCell
      std::string draw(const SCell& cell, const std::string& name = "SCell_{i}_{d}d");
 
      // @brief Draws a Domain
      //
      // Note: the default name has a special hex code in the begining
      // so that string based view-order draws domain in the background
      std::string draw(const HyperRectDomain<Space>& domain, const std::string& uname = "\xff Domain_{i}");

      /**
       * @brief Draws a polygon
       * 
       * @tparam Vec Type of vertex
       */
      template<typename Vec>
      std::string drawPolygon(const std::vector<Vec>& vertices, const std::string& uname = "Polygon_{i}");

      // @brief Draws a ball
      std::string drawBall(const RealPoint& c, const std::string& uname = "Ball_{i}");
      
      // @brief Draws a line
      std::string drawLine(const RealPoint& a, const RealPoint& b, const std::string& uname = "Line_{i}");

      // @brief Draws a quad
      std::string drawQuad(const RealPoint& a, const RealPoint& b, const RealPoint& c, const RealPoint& d, const std::string& uname = "Quad_{i}");
      
      // @brief Draws a DigitalSet
      template<typename Obj, typename Cont>
      std::string draw(const DigitalSetByAssociativeContainer<Obj, Cont>& set, const std::string& name = "Set_{i}");

      // @brief Draws an Object
      template<typename Adj, typename Set>
      std::string draw(const DGtal::Object<Adj, Set>& obj, const std::string& uname = "Object_{i}");

      // @brief Draws an Image
      template<typename D, typename T>
      std::string draw(const ImageContainerBySTLVector<D, T>& image, const std::string& name = "Image_{i}");
            
      // @brief Draws an Image
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV,
                typename TFunctorVm1>
      std::string draw(const ImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV, TFunctorVm1>& adapter, const std::string& name = "Image_{i}");
            
      // @brief Draws an Image
      template <typename TImageContainer,
                typename TNewDomain,
                typename TFunctorD,
                typename TNewValue,
                typename TFunctorV>
      std::string draw(const ConstImageAdapter<TImageContainer, TNewDomain, TFunctorD, TNewValue, TFunctorV>& adapter, const std::string& name = "Image_{i}");

      // @brief Draws a mesh
      template <typename Pt>
      std::string draw(const Mesh<Pt>& mesh, const std::string& uname = "Mesh_{i}");

      // @brief Draws 
      template<typename It, typename Int, int Con>
      std::string draw(const StandardDSS6Computer<It, Int, Con>& computer, const std::string& uname = "Computer_{i}");
      

      // @brief Draws any object with a property
      template<typename T, typename Type>
      std::string draw(const WithQuantity<T, Type>& props, const std::string& uname = "");

      template<typename Type>
      void addQuantity(const std::string& oName, const std::string& qName, const Type& value, int scale = QuantityScale::UNKNOWN);

      template<typename Type>
      void addQuantity(const std::string& oName, const std::string& qName, const std::vector<Type>& value, int scale = QuantityScale::UNKNOWN);

      
      // @brief Adds a clipping plane
      std::string draw(const ClippingPlane& plane, const std::string& name = "");
    
      // @brief Draws a Spherical Accumulator
      template<typename T>
      std::string draw(const SphericalAccumulator<T> accumulator, const std::string& uname = "SphericalAccumulator_{i}");

      // @brief Set the current draw color 
      std::string draw(const DGtal::Color& color, const std::string& name = "");
      
      // @brief Set the current draw color 
      void drawColor(const DGtal::Color& color);

      // @brief Use default colors
      void setDefaultColors();
      
      // @brief Draws adjacencies of further Object
      void drawAdjacencies(bool toggle = true);
      
      // @brief Draws 2D KCell as simplifed mode
      void drawAsSimplified(bool toggle = true);

      // @brief Draws grid of further domains
      void drawAsGrid(bool toggle = true);

      // @brief Reset style
      void defaultStyle();

      // @brief Draws voxels of further object, domains and points
      void drawAsPaving();

      // @brief Draws balls of further object, domains and points
      void drawAsBalls();
    private: // Draw commands
      // To avoid confusion, keep this function as private: 

      // @brief Draws an /!\ arrow (NOT A LINE)
      std::string draw(const std::pair<RealPoint, RealPoint>& arrow, const std::string& uname = "Arrow_{i}");

      /**
       * @brief Draws a range of object to the screen
       */
      template<typename Range> 
      std::string drawGenericRange(const Range& range, const std::string& uname);

      // @brief Draws an image through an iterator
      template<typename T>
      std::string drawImage(const std::string& uname, const T& image);

      // @brief Draws a KCell (signed or not)
      std::string drawKCell(std::string uname, const RealPoint& rp, bool xodd, bool yodd, bool zodd, bool hasSign, bool sign);

    public:
      // The user is responsible for using these wrong..
      //
      DisplayStyle currentStyle;
      bool allowReuseList = false;

      std::vector<ClippingPlane> planes;
      // Leave access to the user for thin modifications
      std::map<std::string, DisplayData<RealPoint>> data;

    protected:
      KSpace myKSpace;
      Embedder myEmbedder;
      CellEmbedder myCellEmbedder;
      SCellEmbedder mySCellEmbedder;

      Callback* myCallback = nullptr;
      std::vector<std::string> myToRender;

      std::string myCurrentName = "";
      DisplayData<RealPoint>* myCurrentData = nullptr;
    }; // Display3D
} // DGtal

#include "Display3D.ih"

