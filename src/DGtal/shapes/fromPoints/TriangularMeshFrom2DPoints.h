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
 * @file TriangularMeshFrom2DPoints.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/12/27
 *
 * Header file for module TriangularMeshFrom2DPoints.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(TriangularMeshFrom2DPoints_RECURSES)
#error Recursive header files inclusion detected in TriangularMeshFrom2DPoints.h
#else // defined(TriangularMeshFrom2DPoints_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TriangularMeshFrom2DPoints_RECURSES

#if !defined TriangularMeshFrom2DPoints_h
/** Prevents repeated inclusion of headers. */
#define TriangularMeshFrom2DPoints_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
#include "DGtal/kernel/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  
  /////////////////////////////////////////////////////////////////////////////
  // template class TriangularMeshFrom2DPoints
  /**
   * Description of template class 'TriangularMeshFrom2DPoints' <p> \brief Aim:
  
   @endcode 
   *
   * 
   *
   * @see  ...
   *
   */
  template <typename TPoint >
  class TriangularMeshFrom2DPoints
  {
    


    
    
    
    // ----------------------- associated types ------------------------------
  public:
  


    /**
     * Structure for representing the faces from the vertex index.
     **/

    
    struct MeshTriangle{
      unsigned int indexesPt [3];
      unsigned int indexAdjTriangles [3];
      bool isActive;
    };
      

    
    
    struct IndexOfCreatedTriangle{
      unsigned int indexTr1;
      unsigned int indexTr2;
      unsigned int indexTr3;
    };
    
    

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * By default the constructed mesh does not contain nor store color information about the mesh.
     * If you want to include color in the MeshFromPoint object you have to set the constructor parameter saveFaceColor to true. 
     * 
     */
    TriangularMeshFrom2DPoints();    



    /**
     * Constructor.
     * By default the constructed mesh does not contain nor store color information about the mesh.
     * If you want to include color in the MeshFromPoint object you have to set the constructor parameter saveFaceColor to true. 
     * 
     */
    TriangularMeshFrom2DPoints(const TPoint &ptUpper, const TPoint & ptLower);    
    

    

    


    /**
     * Destructor.
     */
    ~TriangularMeshFrom2DPoints();




    // --------------- CDrawableWithDisplay2D  realization -------------------
  public:    
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;



    
    // ----------------------- Interface --------------------------------------
  public:
    


    IndexOfCreatedTriangle addPointInMesh(const TPoint & vertex );



    IndexOfCreatedTriangle addPointInDelaunayMesh(const TPoint & vertex );

    
    
    
    /**
     * Adding new vertex.
     *
     **/
    void addVertex(const TPoint &vertex);
    


    
    /**
     * Adding new Triangle.
     *
     **/
    void addTriangle(unsigned int index1, unsigned int index2,  unsigned int index3 );


    /**
     * Adding new Triangle.
     *
     **/
    void addTriangle(unsigned int index1, unsigned int index2,  unsigned int index3,
		     MeshTriangle *t1, MeshTriangle *t2, MeshTriangle t3);

    
    

    int getTriangleIndexInclosing(TPoint p);
    
    

    bool flipTriangleOnEdge(unsigned int indexTriangle, unsigned int num);
    
    
    
    
    int getIndexAdjacentVertex(unsigned int indexTriangle, unsigned int num) const;
    
    int getNumFaceFromIndexVertex(unsigned int indexTriangle, unsigned int indPt1, unsigned int indPt2); 
    


    TPoint getAdjacentVertex(unsigned int indexTriangle, unsigned int num);



    
    int getIndexAdjacentTriangle(unsigned int indexTriangle, unsigned int num);
    

    std::vector<TPoint> getTrianglePointsAdj(unsigned int i, unsigned int adjNum );
    std::vector<TPoint> getTrianglePoints(unsigned int i);
    std::vector<TPoint> getTrianglePoints(const MeshTriangle &tr);
    
    
    
    std::vector<TPoint> getTrianglesFromVertex() const;
    


    unsigned int getNumTriangles() const;

    
    bool isInTriangle(unsigned int indexTriangle, const  TPoint &pt);
 
    bool isInTriangle(const  MeshTriangle &triangle, const TPoint &pt);
    

    bool isInCircle(unsigned int indexTriangle, const  TPoint &ptD) const;


    bool isInCircle(unsigned int indexTriangle, unsigned int indexPtD) const;
    

    bool isOkForDelaunayTriangulation( unsigned int indexTriangle);
    

    void swapTest(unsigned int indexTriangle, unsigned int numFace);

    
    

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
 
    


    // ------------------------- Private Datas --------------------------------
  private:
    std::vector<TPoint>  myVertexList;
    std::vector<MeshTriangle> myTrianglesList;
    

    
    // ------------------------- Hidden services ------------------------------
  protected:

  

double isSameSide(const TPoint &ptA, const TPoint &ptB, const TPoint & pt1, const TPoint &pt2);

    
          




  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    TriangularMeshFrom2DPoints ( const TriangularMeshFrom2DPoints & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    TriangularMeshFrom2DPoints & operator= ( const TriangularMeshFrom2DPoints & other );

    // ------------------------- Internals ------------------------------------
  private:


  


    




  }; // end of class TriangularMeshFrom2DPoints


  /**
   * Overloads 'operator<<' for displaying objects of class 'TriangularMeshFrom2DPoints'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'TriangularMeshFrom2DPoints' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint>
  std::ostream&
  operator<< ( std::ostream & out, const TriangularMeshFrom2DPoints<TPoint> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/fromPoints/TriangularMeshFrom2DPoints.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TriangularMeshFrom2DPoints_h

#undef TriangularMeshFrom2DPoints_RECURSES
#endif // else defined(TriangularMeshFrom2DPoints_RECURSES)



