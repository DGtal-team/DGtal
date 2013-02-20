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
 * @file MeshFromPoints.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/06/29
 *
 * Header file for module MeshFromPoints.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MeshFromPoints_RECURSES)
#error Recursive header files inclusion detected in MeshFromPoints.h
#else // defined(MeshFromPoints_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeshFromPoints_RECURSES

#if !defined MeshFromPoints_h
/** Prevents repeated inclusion of headers. */
#define MeshFromPoints_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  
  /////////////////////////////////////////////////////////////////////////////
  // template class MeshFromPoints
  /**
   * Description of template class 'MeshFromPoints' <p> \brief Aim:
   * This class is defined to represent a surface mesh through a set a
   * vertex and a set of faces represented by its vertex index. By
   * default it does not memorize the color Face and all faces will
   * have the white color. 
   *
   *
   * This class was defined to import and display a mesh from different formats like OFF file format. 
   * Since it realized the concept of CDrawableWithDisplay3D we can display an MeshFromPoints with a Display3D object:
   *
   * First we have to include the following header files:
   @snippet examples/shapes/Mesh3DConstructionAndVisualisation.cpp MeshFromPointsUseInclude 
   *
   * Prepare display using QGLviewer: Viewer3D
   @snippet examples/shapes/Mesh3DConstructionAndVisualisation.cpp MeshFromPointsUseInitDisplay 
   *
   * Construct a MeshFromPoints with various faces:
   @snippet examples/shapes/Mesh3DConstructionAndVisualisation.cpp MeshFromPointsUseMeshCreation
   
   * Displaying the result:
   @snippet examples/shapes/Mesh3DConstructionAndVisualisation.cpp MeshFromPointsUseDisplay
   *
   * 
   *
   * @see  MeshReader MeshWriter meshFromOFF.
   *
   */
  template <typename TPoint >
  class MeshFromPoints
  {
    
    
    
    
    // ----------------------- associated types ------------------------------
  public:
    /**
     * Structure for representing the faces from the vertex index.
     **/

    typedef std::vector<unsigned int> MeshFace;
    



    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Constructor.
     * By default the constructed mesh does not contain nor store color information about the mesh.
     * If you want to include color in the MeshFromPoint object you have to set the constructor parameter saveFaceColor to true. 
     * 
     * @param saveFaceColor used to memorize the color of a face (default= false) 
     */
    MeshFromPoints(bool saveFaceColor=false);    

    /**
     * Constructor.
     * The constructed mesh will store an unique default color information about the mesh.
     * 
     * @param aColor used to memorize the color of a face (default= false) 
     */
    MeshFromPoints(const DGtal::Color &aColor);    
    
    /**
     * Constructor by usung a vertex set as init.
     * The color are not stored in this case.
     * @param vertexSet the set of vertex. 
     */    
    MeshFromPoints(const std::vector<TPoint> &vertexSet);

    /**
     * Destructor.
     */
    ~MeshFromPoints();




    // --------------- CDrawableWithDisplay3D  realization -------------------
  public:    
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;



    
    // ----------------------- Interface --------------------------------------
  public:
    
    
    /**
     * Adding new vertex.
     *
     **/
    void addVertex(const TPoint &vertex);
      
    
  
    /**
     * Add a triangle face given from index position.
     *
     * @param indexVertex1 the index of the first vertex face.
     * @param indexVertex2 the index of the second vertex face.
     * @param indexVertex3 the index of the second vertex face.
     * 
     **/    
    void addTriangularFace(unsigned int indexVertex1, unsigned int indexVertex2, unsigned int indexVertex3, 
			   const DGtal::Color &aColor=DGtal::Color::White);


    /**
     * Add a quad face given from index position.
     *
     * @param indexVertex1 the index of the first vertex face.
     * @param indexVertex2 the index of the second vertex face.
     * @param indexVertex3 the index of the second vertex face.
     * 
     **/    
    void addQuadFace(unsigned int indexVertex1, unsigned int indexVertex2, 
		     unsigned int indexVertex3, unsigned int indexVertex4,
		     const DGtal::Color & aColor=DGtal::Color::White);
    
    
   /**
    * Add a quad face given from index position.
    * 
    **/    
    void addFace(const MeshFace &aFace, const DGtal::Color &aColor=DGtal::Color::White);
    
   
    
    /**
     * Return a reference to the vertex of index i.
     * @param i the index of the vertex.
     * @return the vertex of index i. 
     **/
    const TPoint & getVertex(unsigned int i) const;
    
    
    
    /**
     * Return a reference to a face of index i.
     * @param i the index of the face.
     * @return the face of index i. 
     **/
    const MeshFace & getFace(unsigned int i) const;



    /**
     * Return a reference to a  face Color of index i.
     * @param i the index of the face.
     * @return the color of the face of index i. 
     **/
    const Color & getFaceColor(unsigned int i) const;
    


    
    
    
    /**
     * Return the number of faces contained on the mesh object.
     * @return the number of faces.
     **/
    unsigned int nbFaces() const;

    /**
     * Return the number of faces contained on the mesh object.
     * @return the number of faces.
     **/
    unsigned int  nbVertex() const;
    
    
    
    /**
     * Invert the face order (useful when normal is deducted from vertex order).
     *
     **/
    
    void invertVertexFaceOrder();
    
    
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:

    


    // ------------------------- Private Datas --------------------------------
  private:
    std::vector<MeshFace>  myFaceList;
    std::vector<TPoint>  myVertexList;
    std::vector<DGtal::Color> myFaceColorList;
    bool mySaveFaceColor;
    DGtal::Color myDefaultColor;
    

    
    // ------------------------- Hidden services ------------------------------
  protected:

  


          




  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    MeshFromPoints ( const MeshFromPoints & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    MeshFromPoints & operator= ( const MeshFromPoints & other );

    // ------------------------- Internals ------------------------------------
  private:


    
    




  }; // end of class MeshFromPoints


  /**
   * Overloads 'operator<<' for displaying objects of class 'MeshFromPoints'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'MeshFromPoints' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint>
  std::ostream&
  operator<< ( std::ostream & out, const MeshFromPoints<TPoint> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/fromPoints/MeshFromPoints.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshFromPoints_h

#undef MeshFromPoints_RECURSES
#endif // else defined(MeshFromPoints_RECURSES)
