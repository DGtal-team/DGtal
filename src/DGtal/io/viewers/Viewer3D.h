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
 * @file Viewer3D.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Header file for module Viewer3D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Viewer3D_RECURSES)
#error Recursive header files inclusion detected in Viewer3D.h
#else // defined(Viewer3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Viewer3D_RECURSES

#if !defined Viewer3D_h
/** Prevents repeated inclusion of headers. */
#define Viewer3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <algorithm>


#include <QGLViewer/qglviewer.h>
#include <QColor>
#include <QGLWidget>
#include <QKeyEvent>

#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"




//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{
 /**
   * Base class specifying the methods for classes which intend to
   * modify a Viewer3D stream.
   * 
   */
  struct DrawWithViewer3DModifier {
    std::string styleName() const
    {
      return "DrawWithViewer3DModifier";
    }

    DrawableWithViewer3D* defaultStyleViewer3D( std::string = "" ) const
    {
      return 0;
    }

    virtual void selfDrawViewer3D( Viewer3D &  ) const 
    {}
    
};







/////////////////////////////////////////////////////////////////////////////
// class Viewer3D
/**
 * Description of class 'Viewer3D' <p>
 * \brief Aim:
 */
  class Viewer3D : public QGLViewer
{
    // ----------------------- Standard services ------------------------------
public:


  enum StreamKey {addNewList, updateDisplay, shiftSurfelVisu};
  
  /**
   * The associated map type for storing possible modes used for
   * displaying for digital objects.
   */
  typedef std::map< std::string, std::string > ModeMapping;
  

  
  /**
   * The associated map type for storing the default styles of
   * digital objects.
   */
  typedef std::map< std::string,CountedPtr<DrawableWithViewer3D> > StyleMapping;
  
  QColor myDefaultBackgroundColor;
  QColor myDefaultColor;
  QColor myCurrentFillColor;
  QColor myCurrentLineColor;
  bool myIsBackgroundDefault;


  /**
   * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  

  void createNewVoxelList(bool depthTest=true);
  
  
  /**
   * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  

  void createNewLineList();

  /**
   * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  

  void createNewPointList();

  


    /**
     * @param objectName the name of the object (generally obtained
     * with a 'object.styleName()').
     *
     * @return the current mode for the given object name or "" if no
     * specific mode has been set.
     */
    std::string getMode( const std::string & objectName ) const;


  

  
  
  /**
   * Set the default color for future drawing.
   *
   * @param aColor: a QColor (allow to set a trasnparency value).
   *
   **/  
  
  Viewer3D & operator<<(const QColor & aColor);
  
  

  /**
   * Set the default color for future drawing.
   *
   * @param aColor: a QColor (allow to set a trasnparency value).
   *
   **/
  
  Viewer3D & operator<<(const Viewer3D::StreamKey  & key);
  
  
  


  /**
   *  Add a point as a 3d voxel using default color in the current openGL list.
   *  A voxel can be added in a new list by calling: @createNewList().
   *  @param aPoint: the center of the voxel.
   *  @param width: the width of the voxel (default 0.5)
   *
   **/
  
  
  void addVoxel(int x, int y, int z, QColor color= QColor(220, 220, 220), double width=0.5);
  
  void addPoint(double x, double y, double z ,const QColor &color=QColor(200,20,20), double size=40);

  
  void addLine(double x1, double y1, double z1, double x2, double y2, double z2, 
	       const QColor &color=QColor(20,20,20,200), double width=1.5);
  
 
  void addQuad(double x1, double y1, double z1,  double x2, double y2, double z2,
  	       double x3, double y3, double z3,  double x4, double y4, double z4, QColor aColor);

  
  void addKSSurfel(double x, double y, double z, 
		   bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor, 
		   bool isOriented= false, bool isOrientedPositively=true, bool basicMode=false);
  
  void addKSVoxel(int x, int y, int z);
  
  void addKSPointel(double x, double y, double z, double size=0.1,
		    bool isSigned=false, bool signPos=true);
  
  void addKSLinel(double x1, double y1, double z1,
		  double x2, double y2, double z2,
		  double width=0.02, bool isSigned=false, bool signPos=true);
  
  
  
  /**
   * Add a new 3D Clipping plane represented by ax+by+cz+d = 0 
   * A maximal of five clipping plane can be added.
   *
   * @param a, b, c, d : plane equation.
   **/
  
  void addClippingPlane(double a, double b, double c, double d, bool drawPlane);
  

  

  void setLineColor(QColor aColor) ;
  QColor getLineColor() ;

  void setFillColor(QColor aColor) ;
  QColor getFillColor() ;






  

  /**
   *  Sort all surfels from the camera.
   *  
   *
   **/
  
  void sortSurfelFromCamera();
  


  
  /**
   * Draws the drawable [object] in this board. It should satisfy
   * the concept CDrawableWithViewer3D, which requires for instance a
   * method selfDraw( Viewer3D & ).
   *
   * @param object any drawable object.
   * @return a reference on 'this'.
   */
  template <typename TDrawableWithViewer3D>
  Viewer3D & operator<<( const  TDrawableWithViewer3D & object );
  


    // ----------------------- Interface --------------------------------------
public:
  
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
  


public:
  
  ModeMapping myModes;  
      /**
     * For instance, may associate a new style object T1 to the class
     * "HyperRectDomain": myStyles[ "HyperRectDomain" ] = T1.
     *
     * One can also store a new style T2 for a specific mode used for
     * drawing a class:  myStyles[ "HyperRectDomain/Paving" ] = T2.
     *
     * Modes may only be used in objects implementing the concept
     * CDrawableWithDGtalBoard.
     */
    StyleMapping myStyles;

  
    // ------------------------- Private Datas --------------------------------
private:
  
  struct lineGL{
    double x1, y1, z1;
    double x2, y2, z2;
    double width;
    unsigned int R,G,B,T;
    bool isSigned;
    bool signPos;
  };
    
  struct voxelGL{
    int x, y,z;
    unsigned int R,G,B,T;
    double width;
  };
  
  struct pointGL{
    double  x, y,z;
    unsigned int R,G,B,T;
    bool isSigned;
    bool signPos;
    double size;
  };
  
  struct clippingPlaneGL{
    double a,b,c,d;
  };

  struct  quadGL{
    double x1,y1,z1;
    double x2,y2,z2;
    double x3,y3,z3;
    double x4,y4,z4;    
    double nx, ny, nz;
    unsigned int R,G,B,T;
  };
    
  // Used to represent all the list used in the display.
  std::vector< std::vector<voxelGL> > myVoxelSetList;
  
  // Used to represent all the list of line primitive
  std::vector< std::vector<lineGL> > myLineSetList;
  
  // Used to represent all the list of line primitive
  std::vector< std::vector<pointGL> > myPointSetList;

  // Represent all the clipping planes added to the scene (of maxSize=5).
  std::vector< clippingPlaneGL > myClippingPlaneList;
  
  // For saving all voxels of Khalimsky space (used to display Khalimsky Space Cell)
  // see. myVoxelSetList (first vector)
  
  
  // For saving all surfels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< quadGL > myKSSurfelList;

  // For saving all pointels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< pointGL > myKSPointelList;

  // For saving all linels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< lineGL > myKSLinelList;

  
 
  
  

  // Represent all the drawed planes
  std::vector< quadGL > myQuadList;
  
  
  //Used to define if GL_TEST_DEPTH is used. 
  std::vector<bool> myListVoxelDepthTest;

  qglviewer::Vec myBoundingPtUp;
  qglviewer::Vec myBoundingPtLow;

  GLuint myListToAff;
  unsigned int myNbListe;
  qglviewer::Vec myOrig, myDir, myDirSelector, mySelectedPoint;  
  QPoint myPosSelector;
  
    // ------------------------- Hidden services ------------------------------
protected:
  
  
  /**
   *  Permit to update the OpenGL list to be displayed. 
   *  Need to called after a number of addVoxel or after a sortSurfelFromCamera().
   *
   **/
  void updateList(bool updateBoundingBox=true);
  
  
  /**
   * Draw a linel by using the 	[gluCylinder] primitive.
   * 
   *
   **/
  void glDrawGLLinel(lineGL aLinel);
  
  
 
 
  /**
   * Draw a linel by using the 	[gluCSphere] primitive.
   * 
   *
   **/
  void glDrawGLPointel(pointGL pointel);



  
  


  
  virtual void keyPressEvent(QKeyEvent *e);
  
  struct compFarthestFromCamera{
    qglviewer::Vec posCam;
    bool operator() ( voxelGL s1, voxelGL s2){
      double dist1= sqrt((posCam.x-s1.x)*(posCam.x-s1.x)+ (posCam.y-s1.y)*(posCam.y-s1.y)+(posCam.z-s1.z)*(posCam.z-s1.z));
      double dist2= sqrt((posCam.x-s2.x)*(posCam.x-s2.x)+ (posCam.y-s2.y)*(posCam.y-s2.y)+(posCam.z-s2.z)*(posCam.z-s2.z));
      return dist1>dist2;
    }  
  }
;
  
struct compFarthestSurfelFromCamera{
  qglviewer::Vec posCam;
  bool operator() ( quadGL q1, quadGL q2){
    qglviewer::Vec center1((q1.x1+q1.x2+q1.x3+q1.x4)/4.0, (q1.y1+q1.y2+q1.y3+q1.y4)/4.0, (q1.z1+q1.z2+q1.z3+q1.z4)/4.0 );
    qglviewer::Vec center2((q2.x1+q2.x2+q2.x3+q2.x4)/4.0, (q2.y1+q2.y2+q2.y3+q2.y4)/4.0, (q2.z1+q2.z2+q2.z3+q2.z4)/4.0 );
    
    
    double dist1= sqrt((posCam.x-center1.x)*(posCam.x-center1.x)+ (posCam.y-center1.y)*(posCam.y-center1.y)+(posCam.z-center1.z)*(posCam.z-center1.z));
    double dist2= sqrt((posCam.x-center2.x)*(posCam.x-center2.x)+ (posCam.y-center2.y)*(posCam.y-center2.y)+(posCam.z-center2.z)*(posCam.z-center2.z));
    return dist1>dist2;
  }  
};




protected :
  virtual void drawWithNames();
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;
  virtual void postSelection(const QPoint& point);
  


    // ------------------------- Internals ------------------------------------
private:

  
  /**
   * Used to define update the scene bounding box when objects are added 
   *
   **/
  
  void updateBoundingBox(int x, int y, int z);

  double myCurrentfShiftVisuKSSurfels;
  

  }; // end of class Viewer3D



 

 /**
   * Modifier class in a Viewer3D stream. Useful to choose your
   * own mode for a given class. Realizes the concept
   * CDrawableWithViewer3D.
   */
  struct SetMode3D : public DrawWithViewer3DModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    SetMode3D( std::string classname, std::string mode )
      : myClassname( classname ), myMode( mode )
    {}
    void selfDrawViewer3D( Viewer3D & viewer ) const
    {
      viewer.myModes[ myClassname ] = myMode;
    }
  private:
    std::string myClassname;
    std::string myMode;
  };







  /**
   * Modifier class in a Viewer3D stream. Useful to choose your own
   * style for a given class. Realizes the concept
   * CDrawableWithViewer3D.
   */
  struct CustomStyle3D : public DrawWithViewer3DModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    CustomStyle3D( std::string classname, DrawableWithViewer3D* style )
      : myClassname( classname ), myStyle( style )
    {}

    std::string styleName() const
    {
      return "CustomStyle3D";
    }

    void selfDrawViewer3D( Viewer3D & viewer ) const
    {
      viewer.myStyles[ myClassname ] = myStyle;
    }
  private:
    std::string myClassname;
    CountedPtr<DrawableWithViewer3D> myStyle;
  };




  /**
   * Custom style class redefining the fill color and the
   * gl_LINE/gl_POINT color. You can use Qcolor with alpha
   * transparency value but you nedd to take into account the z-buffer
   * during the Open-GL based rendering.
   *
   \code
   Viewer3D viewer;
   viewer << CustomColors3D(QColor(250, 0,0),QColor(250, 0,0));
   ...
   \endcode
   * @see Viewer3D
   */
  struct CustomColors3D : public DrawWithViewer3DModifier
  {
    QColor myPenColor;
    QColor myFillColor;

    /**
     * Constructor.
     *
     * @param penColor specifies the pen color.
     * @param fillColor specifies the fill color.
     */
    CustomColors3D( const QColor & penColor,
		    const QColor & fillColor )
      : myPenColor( penColor ), myFillColor( fillColor )
    {}
    
    virtual void selfDrawViewer3D( Viewer3D & viewer) const
    {
      viewer.setFillColor(myFillColor);
      viewer.setLineColor(myPenColor);
    }
  };

 /**
   * Class for adding a Clipping plane through the Viewer3D
   * stream. Realizes the concept CDrawableWithViewer3D.
   */


  struct ClippingPlane : public DrawWithViewer3DModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    ClippingPlane( double a, double b, double c, double d, bool drawPlane=true )
      : myA( a ), myB( b ), myC( c ), myD ( d ), myDrawPlane(drawPlane)  
    {}
    void selfDrawViewer3D( Viewer3D & viewer ) const
    {
      viewer.addClippingPlane(myA, myB, myC, myD, myDrawPlane);
      
    }
    double * getEquation(){
      double *r = new double[4];
      r[0] = myA;
      r[1] = myB;
      r[2] = myC;
      r[3] = myD;
      return r;
    } 
  
  private:
    double myA;
    double myB;
    double myC;
    double myD;
    bool myDrawPlane;
    
  };


/**
 * Overloads 'operator<<' for displaying objects of class 'Viewer3D'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Viewer3D' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const Viewer3D & object );


} // namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/Viewer3D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Viewer3D_h

#undef Viewer3D_RECURSES
#endif // else defined(Viewer3D_RECURSES)
