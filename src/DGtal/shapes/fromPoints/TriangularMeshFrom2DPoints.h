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
#include <math.h>
#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
#include "DGtal/kernel/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  
  /////////////////////////////////////////////////////////////////////////////
  // template class TriangularMeshFrom2DPoints
  /**
   * Description of template class 'TriangularMeshFrom2DPoints' <p>
   * \brief Aim: This class is mainly defined to apply reconstruction
   * of Delaunay 2D triangular Mesh.
   *
   *
   *
   * This classe proposes to use the incremental construction of
   * [Guibas, D. E. Knuth, and M. Sharir (1992): \cite Guibas1992]
   * which permits to obtain a Delaunay triangular mesh. The algorithm
   * is given here in 2D even the type of point can be given in nD, but
   * in such case only  the two first coordinates will be considered.
   *
   * To use this class to construct an Delaunay Triangular Mesh you
   * have first to include the following header files:
   * 
   *  @snippet examples/shapes/DelaunayTriangulationAndVoronoi.cpp TriangularMeshFrom2DPointsINC 
   *
   * Then you construct an empty mesh from two bounding box points:
   *
   *  @snippet examples/shapes/DelaunayTriangulationAndVoronoi.cpp TriangularMeshFrom2DPointsINIT 
   *
   * And adding point to mesh:
   * @snippet examples/shapes/DelaunayTriangulationAndVoronoi.cpp TriangularMeshFrom2DPointsADDPOINT
   *
   * Finally display the results:
   * @snippet examples/shapes/DelaunayTriangulationAndVoronoi.cpp TriangularMeshFrom2DPointsDISPLAYRES
   *
   *
   */
  template <typename TPoint >
  class TriangularMeshFrom2DPoints
  {

 public: 

   
    // ----------------------- associated types ------------------------------
  public:
  
    /**
     * Structure for representing the faces from the vertex index.
     **/
    struct MeshTriangleWithIndex{
      unsigned int indexesPt [3];
      unsigned int indexAdjTriangles [3];
      bool isActive;
    };      
    
    // Used by iterators
    struct MeshTriangle{
      TPoint point1;
      TPoint point2;
      TPoint point3;
    };      
    
    struct IndexOfCreatedTriangle{
      unsigned int indexTr1;
      unsigned int indexTr2;
      unsigned int indexTr3;
    };

    // Used for generating Voronoi diagram
    struct CompToOrderTriangle{
      TPoint pointCenter;
      bool operator() ( MeshTriangle t1, MeshTriangle t2 )
      {
	double p1x = (t1.point1[0]+t1.point2[0]+ t1.point3[0])/3.0;
	double p1y = (t1.point1[1]+t1.point2[1]+ t1.point3[1])/3.0;
      
	double p2x = (t2.point1[0]+t2.point2[0]+ t2.point3[0])/3.0;
	double p2y = (t2.point1[1]+t2.point2[1]+ t2.point3[1])/3.0;

	double dx1 = p1x-pointCenter[0];
	double dx2 = p2x-pointCenter[0];
	
	double dy1 = p1y-pointCenter[1];
	double dy2 = p2y-pointCenter[1];
	
	  double norm1 = sqrt(dx1*dx1+dy1*dy1);
	  double norm2 = sqrt(dx2*dx2+dy2*dy2);

	  double angle1 = atan2((double)(dy1/norm1),(double)(dx1/norm1));
	  double angle2 = atan2((double)(dy2/norm2),(double)(dx2/norm2));
	  if(angle1<0.0) angle1+= 2.0* 3.14159265;
	  if(angle2<0.0) angle2+= 2.0*3.14159265;
	  
	  return angle1< angle2;
	};
    };
    
    typedef typename std::vector<MeshTriangleWithIndex> StorageTr; 
    typedef unsigned int Size;
    
    
  private:

    
    // ----------------------- Standard services ------------------------------
  public:




    /**
     * Default Constructor.
     * @param saveMapPointToTriangle when set to true, it saves for each point the set of triangle incident to it (default no saving). 
     * 
     */
    TriangularMeshFrom2DPoints( bool saveMapPointToTriangle=false );    
    

    /**
     * Constructor from a bounding box.
     * This constructor adds two new triangles in the mesh associated to the bounding box given by the upper and lower points.
     *
     *  @param ptUpper upper bounding box point.
     *  @param ptLower lower bounding box point.     
     *  @param saveMapPointToTriangle when set to true, it saves for each point the set of triangle incident to it (default no saving). 
     *"
     */
    TriangularMeshFrom2DPoints(const TPoint &ptUpper, const TPoint & ptLower, bool saveMapPointToTriangle=false);    
    
    
    
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




    
  // ----------------------- Storage service ------------------------------

    
    class TriangularMeshIterator
    {
    private:
      
      const TriangularMeshFrom2DPoints * myTriangularMesh;
      
      // The current position in the mesh (according the triangle numbers)
      unsigned int myPos;
      
      // The current triangle
      MeshTriangle  myCurrentTriangle;

      

      
      
    public:
      TriangularMeshIterator(): myTriangularMesh( NULL ), myPos(1){}
      /**
       * Constructor.
       * Nb: complexity in O(1).
       *
       * @param aTrMesh a TriangularMeshFrom2DPoints,
       * @param n the position in [TriangularMeshFrom2DPoints] (within 0 and numberOfTriangle()).
       */
      TriangularMeshIterator( const TriangularMeshFrom2DPoints & aTrMesh, unsigned int n =0);
      
        /**
         * Destructor. Does nothing.
         */
        ~TriangularMeshIterator()
        { }
      
    public:

      /**
       * @return the current coordinates.
       */
      const MeshTriangle& operator*() const
      {
	return myCurrentTriangle;
      }
      
      /**
       * @return the current coordinates.
       */
      const TPoint& get() const
      {
	return myCurrentTriangle;
      }
       /**
         * Pre-increment.
         * Goes to the next point on the TriangleMesh.
         */
        TriangularMeshIterator & operator++()
        {
          this->next();
          return *this;
        }

        /**
         * Post-increment.
         * Goes to the next point on the TriangleMesh.
         */
        TriangularMeshIterator operator++(int)
        {
          TriangularMeshIterator tmp(*this);
          this->next();
          return tmp;
        }
      /**
       * Copy constructor.
         * @param other the iterator to clone.
         */
       TriangularMeshIterator( const TriangularMeshIterator & anOther )
          : myTriangularMesh( anOther.myTriangularMesh ), myPos( anOther.myPos ), myCurrentTriangle( anOther.myCurrentTriangle )
        { } 

       
      /**
       * Copy operator
       **/
      TriangularMeshIterator &  operator= ( const TriangularMeshIterator & anOther );
      
        /**
         * Goes to the next point on the chain.
         */
        void next();


        /**
         * Equality operator.
         *
         * @param aOther the iterator to compare with (must be defined on
         * the same chain).
         *
         * @return 'true' if their current positions coincide.
         */
        bool operator== ( const TriangularMeshIterator & anOther ) const
        {
          ASSERT( myTriangularMesh == anOther.myTriangularMesh );
          return myPos == anOther.myPos;
        }


        /**
         * Inequality operator.
         *
         * @param aOther the iterator to compare with (must be defined on
         * the same chain).
         *
         * @return 'true' if their current positions differs.
         */
        bool operator!= ( const TriangularMeshIterator & anOther ) const
        {
          ASSERT( myTriangularMesh == anOther.myTriangularMesh );
          return myPos != anOther.myPos;
        }


    };

    





    // ----------------------- Iterator services--------------------------------   
    
    /**
     *   Iterator service
     *   @return begin iterator.
     **/
    TriangularMeshIterator begin() const;

    /**
     *   Iterator service
     *   @return end iterator.
     **/
    TriangularMeshIterator end() const;



    
    // ----------------------- Interface --------------------------------------
  public:
        
    /**
     * Add a vertex inside the mesh. From the triangle containing the
     * given vertex (given by basic search), it constructs three new
     * triangles starting from the first triangle point and following
     * a clockwise order. Adjacency of the new triangles are also
     * updated. 

     * Notes that when the point is outside to the mesh or when it is
     * along an existing edge mesh; the vertex is simply ignored and
     * in this case the resulting index of created face is set to 0.
     * 
     * This operation is performed * in O(n) with n is the number of
     * triangle mesh.
     *
     *  @param vertex: The new point.
     *  @return the index (IndexOfCreatedTriangle) of the created triangles. 
     */

    IndexOfCreatedTriangle addPointInsideMesh(const TPoint & vertex );

    
    /**
     * Add a vertex inside the mesh with checking and maintaining
     * Delaunay triangulation . From the triangle containing the given
     * vertex (given by basic search), it constructs three new
     * triangles starting from the first triangle point and following
     * a clockwise order. If the triangle does not satisfy the
     * Delaunay condition the triangle is transformed with adjacent
     * triangle. Adjacency of the new triangles are also updated. 
     *
     * Notes that when the point is outside of any triangle mesh or
     * when it is along an existing edge mesh; the vertex is simply
     * ignored and in this case the resulting index of created face is
     * set to 0.
     * 
     * This operation is performed * in O(n) with n is the number of
     * triangle mesh.
     *
     *  @param vertex: The new point.
     *  @return the index (IndexOfCreatedTriangle) of the created triangles. 
     */
   
    
    IndexOfCreatedTriangle addPointInsideDelaunayMesh(const TPoint & vertex );

       
    
    /**
     * Return the index of the triangle containing the given point as
     * parameter. If no triangle exists it returns 0 which is
     * associated to the index of an empty triangle.
     *
     * @param aPoint: the point for which we search the inclosing triangle.
     * @return the index of the triangle containing the point.  
     **/
    
    unsigned int getTriangleIndexInclosingPoint(const TPoint & aPoint) const;
    
    
    /**
     * Returns the index of the adjacent vertex of the triangle given
     * at index indexTriangle according a edge number. If no point
     * exits it returns 0 which is associated to the index of an empty
     * point.
     *
     * @param indexTriangle: the index of the considered triangle.
     * @param numEdge: the edge number (starting from 1 to 3), the
     * edges are ordered according the first triangle point in
     * clockwise order.
     * @return the index of the adjacent point.  
     **/
        
    unsigned int getIndexAdjacentVertex(unsigned int indexTriangle, unsigned int numEdge) const;
        

    /**
     *  Returns the index of the adjacent triangle according to the
     *  triangle edge number. If no triangle exits, it returns 0 which
     *  is associated to the index of an empty triangle.
     *
     * @param numEdge: the edge number (starting from 1 to 3), the
     * edges are ordered according the first triangle point in
     * clockwise order.
     *
     * @return the index of the adjacent triangle.  
     *
     **/
    
    unsigned int getIndexAdjacentTriangle(unsigned int indexTriangle, unsigned int numEdge) const;
    
    
    /**
     * Returns the number associated to the triangle edge which
     * contains the two points of index indPt1 and indPt2. The order
     * of points has no importance. If such edge does not exit in the
     * given triangle it returns 0.
     * 
     * @param indexTriangle: index of the considered triangle (starting from 1, 0 is the empty triangle)
     * @param indexPt1: index of the a vertex.
     * @param indexPt2: index of another vertex.
     * @return the number associated to the triangle edge containing the points.
     *
     **/

    unsigned int getNumEdgeFromIndexVertex(unsigned int indexTriangle, unsigned int indPt1, unsigned int indPt2) const ; 


     /**
     * Returns the adjacent vertex of the triangle given at index
     * indexTriangle according a edge number. If no point exits it
     * returns the empty point of index 0.
     *
     * @param indexTriangle: the index of the considered triangle.
     * @param numEdge: the edge number (starting from 1 to 3), the
     * edges are ordered according the first triangle point in
     * clockwise order.
     * @return the adjacent point.  
     **/
    
    TPoint getAdjacentVertex(unsigned int indexTriangle, unsigned int numEdge) const;
    

    /**
     *  Transform the given triangle into another one constructed from
     *  the adjacent vertex given according the edge number.
     * 
     *  @param indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     *  @param numEdge:  numEdge: the edge number (starting from 1 to 3), the
     * edges are ordered according the first triangle point in
     * clockwise order.
     *  @return true: if the swap was possible or not.
     **/
    
    bool swapTriangleOnEdge(unsigned int indexTriangle, unsigned int numEdge);        
    
    
    /**
     * Returns a vector containing the three vertex associated to the triangle of index i.
     * @param indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     * @return a vector containing the three triangle vertex.
     **/

    std::vector<TPoint> getTrianglePoints(unsigned int indexTriangle) const;


    /**
     * Returns a vector containing the three vertex associated to the triangle tr.
     * @param tr: the triangle.
     * @return a vector containing the three triangle vertex.
     **/

    std::vector<TPoint> getTrianglePoints(const MeshTriangleWithIndex &tr) const;
        
    

    
    /**
     * @return the set of vertex. 
     * 
     **/   
    std::vector<TPoint> getAllVertex() const;
    

    
    /**
     * @return the current number of triangles 
     * 
     **/   
    unsigned int getNumTriangles() const;

    

    /**
     * Check if a point is located stricly inside a triangle (return false if the point belongs to an triangle edge). 
     * @param indexTriangle: the index of a triangle.
     * @param tp: the point to tested.
     **/
    
    bool isInTriangle(unsigned int indexTriangle, const  TPoint &pt) const;
    
    

    /**
     * Check if a point is located stricly inside a triangle (return false if the point belongs to an triangle edge). 
     * @param indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     * @param tp: the point to tested.
     **/
    
    bool isInTriangle(const  MeshTriangleWithIndex &triangle, const TPoint &pt) const;
    
    
    
    /** 
     * Check if a point is included in the circumcirle of a given triangle.
     * @param indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     * @param pt: the tested point.
     *
     **/
    
    bool isInCircle(unsigned int indexTriangle, const  TPoint &pt) const;

    /** 
     * Check if a point is included in the circumcirle of a given triangle.
     * @param indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     * @param indexPt: the index of the tested point.
     *
     **/

    bool isInCircle(unsigned int indexTriangle, unsigned int indexPt) const;
    


    /**
     * Test and apply triangle swap according to the Delaunay
     * condition (point inside or outside the circumcirle). The tested
     * point is the one which is adjacent to the edge identified with
     * numEdge. If the swap is performed, it also applied the test on
     * new adjacent faces.
     * 
     * @param  indexTriangle:  the index of the considered triangle (starting from 1, 0 is the empty triangle).
     * @param  numEdge: the edge number (starting from 1 to 3), the
     * edges are ordered according the first triangle point in
     * clockwise order. 
     **/

    void swapTest(unsigned int indexTriangle, unsigned int numEdge);

    
    /**
     * Add a triangle in the mesh.
     * @param the new triangle.
     * @param addToVertexMap if true the multi map associating each point to its incident triangles is saved.
     **/
    
    void addTriangle(const MeshTriangleWithIndex &tr, bool addToVertexMap=false);


    /**
     * Remove the triangle from the given index;
     * 
     * @param index the triangle index to be removed
     **/
    
    void removeTriangle(unsigned int index);
    



    /**
     * Remove all triangles which are incident to vertex.
     *
     * NB: It only works if the saving map point to vertex option
     * (saveMapPointToTriangle) was set in the constructor (not done
     * by default)
     *
     * @param indexVertex 
     * @return the number of triangles which were removed.
     **/
    
    unsigned int removeTrianglesIncidentToVertex(unsigned int indexVertex);
    


    
    /**
     * Remove all triangles which are incident to bounding vertex.
     *
     * NB: It only works if the saving map point to vertex option
     * (saveMapPointToTriangle) was set in the constructor (not done
     * by default)
     *
     * @return the number of triangles which were removed.
     **/
    
    unsigned int removeTrianglesOfBoundingVertex();
    


    /**
     * Return a vector containing a set of MeshTriangle. 
     *
     * Nb: It only works if the saving map point to vertex option
     * (saveMapPointToTriangle) was set in the constructor (not done
     * by default)
     *
     * @parameter aVertexIndex 
     * @return the set of triangles
     **/
    std::vector<MeshTriangle> getTrianglesIncidentToVertex(unsigned int aVertexIndex);


    
    /**
     * Return a vector containing all the MeshTriangle of the mesh. 
     *
     * @return the set of triangles
     **/
    std::vector<MeshTriangle> getAllTriangles() const;


    /**
     * Return the voronoi diagram defined as a set of polygons.
     *
     * Nb: It only works if the saving map point to vertex option
     * (saveMapPointToTriangle) was set in the constructor (not done
     * by default)
     *
     *  @return the vector containing the contour polygons given as vector of points. 
     *
     **/
    std::vector< std::vector<TPoint> > getVoronoiDiagram();
    
    

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
    StorageTr  myTrianglesList;
    unsigned int myNumberOfTriangle;
    bool myIsSavingMapPointToTriangle;
    // The map for recovering for each point a source triangle
    std::multimap<TPoint, unsigned int> myMapVertexToTriangle;
    
    // ------------------------- Hidden services ------------------------------
  protected:

  

    /**
     * Return a positive or negative number according two point pt1
     * and pt2 are on the same side in reference to a segment.
     *
     * @param ptA a point.
     * @param ptB a point.
     * @param pt1 a  point to be tested.
     * @param pt2 the another point to be tested.
     * @return a positive number if 
     * 
     **/
    

    double isSameSide(const TPoint &ptA, const TPoint &ptB, const TPoint & pt1, const TPoint &pt2) const ;
    
    
    
    
    /**
     * @return the size of the triangle container.
     *
     **/
    
    unsigned int getTriangleContainerSize() const;
    
    
          




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



