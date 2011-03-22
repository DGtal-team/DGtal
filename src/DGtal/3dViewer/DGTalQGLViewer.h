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
 * @file DGTalQGLViewer.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Header file for module DGTalQGLViewer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DGTalQGLViewer_RECURSES)
#error Recursive header files inclusion detected in DGTalQGLViewer.h
#else // defined(DGTalQGLViewer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DGTalQGLViewer_RECURSES

#if !defined DGTalQGLViewer_h
/** Prevents repeated inclusion of headers. */
#define DGTalQGLViewer_h

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
   * modify a DGTalQGLViewer stream.
   * 
   */
  struct DrawWithQGLViewerModifier {
    std::string styleName() const
    {
      return "DrawWithQGLViewerModifier";
    }

    DrawableWithDGtalQGLViewer* defaultStyleQGL( std::string = "" ) const
    {
      return 0;
    }

    virtual void selfDrawQGL( DGTalQGLViewer &  ) const 
    {}
  };







/////////////////////////////////////////////////////////////////////////////
// class DGTalQGLViewer
/**
 * Description of class 'DGTalQGLViewer' <p>
 * \brief Aim:
 */
  class DGTalQGLViewer : public QGLViewer
{
    // ----------------------- Standard services ------------------------------
public:

  enum StreamKey {addNewList, updateDisplay};
  
  /**
   * The associated map type for storing possible modes used for
   * displaying for digital objects.
   */
  typedef std::map< std::string, std::string > ModeMapping;
  

  
  /**
   * The associated map type for storing the default styles of
   * digital objects.
   */
  typedef std::map< std::string,CountedPtr<DrawableWithDGtalQGLViewer> > StyleMapping;
  
  
  QColor myDefaultColor;

  QColor myCurrentFillColor;
  QColor myCurrentLineColor;



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
   * Allow to change the order between displayed objects in order to
   * fit good OpenGL display constraint (first position: the object
   * inside another one) .
   *
   **/

  void inverseListGLOrder();



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
  
  DGTalQGLViewer & operator<<(const QColor & aColor);
  
  

  /**
   * Set the default color for future drawing.
   *
   * @param aColor: a QColor (allow to set a trasnparency value).
   *
   **/
  
  DGTalQGLViewer & operator<<(const DGTalQGLViewer::StreamKey  & key);
  
  
  


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
  
  
  /**
   * Add a new 3D Clipping plane represented by ax+by+cz+d = 0 
   * A maximal of five clipping plane can be added.
   *
   * @param a, b, c, d : plane equation.
   **/
  
  void addClippingPlane(double a, double b, double c, double d);
  

  

  void setLineColor(QColor aColor) ;
  QColor getLineColor() ;

  void setFillColor(QColor aColor) ;
  QColor getFillColor() ;



  /**
   *  Permit to update the OpenGL list to be displayed. 
   *  Need to called after a number of addVoxel or after a sortSurfelFromCamera().
   *
   **/
  
  void updateList();




  

  /**
   *  Sort all surfels from the camera.
   *  
   *
   **/
  
  void sortSurfelFromCamera();
  


  
  /**
   * Draws the drawable [object] in this board. It should satisfy
   * the concept CDrawableWithDGtalQGLViewer, which requires for instance a
   * method selfDraw( DGTalQGLViewer & ).
   *
   * @param object any drawable object.
   * @return a reference on 'this'.
   */
  template <typename TDrawableWithDGtalQGLViewer>
  DGTalQGLViewer & operator<<( const  TDrawableWithDGtalQGLViewer & object );
  


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
    uint R,G,B,T;
  };
    
  struct voxelGL{
    int x, y,z;
    uint R,G,B,T;
    double width;
  };
  
  struct pointGL{
    double  x, y,z;
    uint R,G,B,T;
    double size;
  };

  struct clippingPlaneGL{
    double a,b,c,d;
  };
  
    
  // Used to represent all the list used in the display.
  std::vector< std::vector<voxelGL> > myVoxelSetList;
  
  // Used to represent all the list of line primitive
  std::vector< std::vector<lineGL> > myLineSetList;
  
  // Used to represent all the list of line primitive
  std::vector< std::vector<pointGL> > myPointSetList;

  // Represent all the clipping planes added to the scene (of maxSize=5).
  std::vector< clippingPlaneGL > myClippingPlaneList;
  

  //Used to define if GL_TEST_DEPTH is used. 
  std::vector<bool> myListVoxelDepthTest;

  qglviewer::Vec myBoundingPtUp;
  qglviewer::Vec myBoundingPtLow;

  GLuint myListToAff;
  bool myReverseOrderList;  
  
  uint myNbListe;
  
  
    // ------------------------- Hidden services ------------------------------
protected:

  virtual void keyPressEvent(QKeyEvent *e);
  
  struct compFarthestFromCamera{
    qglviewer::Vec posCam;
    bool operator() ( voxelGL s1, voxelGL s2){
      float dist1= sqrt((posCam.x-s1.x)*(posCam.x-s1.x)+ (posCam.y-s1.y)*(posCam.y-s1.y)+(posCam.z-s1.z)*(posCam.z-s1.z));
      float dist2= sqrt((posCam.x-s2.x)*(posCam.x-s2.x)+ (posCam.y-s2.y)*(posCam.y-s2.y)+(posCam.z-s2.z)*(posCam.z-s2.z));
      return dist1>dist2;
    }  
  };
  



protected :
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;



    // ------------------------- Internals ------------------------------------
private:

  }; // end of class DGTalQGLViewer



 

 /**
   * Modifier class in a DGTalQGLViewer stream. Useful to choose your
   * own mode for a given class. Realizes the concept
   * CDrawableWithDGtalQGLViewer.
   */
  struct SetMode3D : public DrawWithQGLViewerModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    SetMode3D( std::string classname, std::string mode )
      : myClassname( classname ), myMode( mode )
    {}
    void selfDrawQGL( DGTalQGLViewer & viewer ) const
    {
      viewer.myModes[ myClassname ] = myMode;
    }
  private:
    std::string myClassname;
    std::string myMode;
  };







  /**
   * Modifier class in a DGTalQGLViewer stream. Useful to choose your own
   * style for a given class. Realizes the concept
   * CDrawableWithDGtalQGLViewer.
   */
  struct CustomStyle3D : public DrawWithQGLViewerModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    CustomStyle3D( std::string classname, DrawableWithDGtalQGLViewer* style )
      : myClassname( classname ), myStyle( style )
    {}

    std::string styleName() const
    {
      return "CustomStyle3D";
    }

    void selfDrawQGL( DGTalQGLViewer & viewer ) const
    {
      viewer.myStyles[ myClassname ] = myStyle;
    }
  private:
    std::string myClassname;
    CountedPtr<DrawableWithDGtalQGLViewer> myStyle;
  };




  /**
   * Custom style class redefining the fill color and the
   * gl_LINE/gl_POINT color. You can use Qcolor with alpha
   * transparency value but you nedd to take into account the z-buffer
   * during the Open-GL based rendering.
   *
   \code
   DGTalQGLviewer viewer;
   viewer << CustomColors3D(QColor(250, 0,0),QColor(250, 0,0));
   ...
   \endcode
   * @see DGTalQGLviewer
   */
  struct CustomColors3D : public DrawWithQGLViewerModifier
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
    
    virtual void selfDrawQGL( DGTalQGLViewer & viewer) const
    {
      viewer.setFillColor(myFillColor);
      viewer.setLineColor(myPenColor);
    }
  };

 /**
   * Class for adding a Clipping plane through the DGTalQGLViewer
   * stream. Realizes the concept CDrawableWithDGtalQGLViewer.
   */


  struct ClippingPlane : public DrawWithQGLViewerModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    ClippingPlane( double a, double b, double c, double d )
      : myA( a ), myB( b ), myC( c ), myD ( d )  
    {}
    void selfDrawQGL( DGTalQGLViewer & viewer ) const
    {
      viewer.addClippingPlane(myA, myB, myC, myD);
      
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
    
    
  };


/**
 * Overloads 'operator<<' for displaying objects of class 'DGTalQGLViewer'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'DGTalQGLViewer' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const DGTalQGLViewer & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/3dViewer/DGTalQGLViewer.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DGTalQGLViewer_h

#undef DGTalQGLViewer_RECURSES
#endif // else defined(DGTalQGLViewer_RECURSES)
