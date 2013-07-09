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

#ifdef APPLE
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif


#include <QGLViewer/qglviewer.h>
#include <QGLWidget>
#include <QKeyEvent>

#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/math/BasicMathFunctions.h"

//#include "DGtal/io/DrawWithDisplay3DModifier.h"


//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{



/////////////////////////////////////////////////////////////////////////////
// class Viewer3D
/**
   * Description of class 'Viewer3D' <p>
   * \brief Aim: Display 3D
   * primitive (like PointVector, DigitalSetBySTLSet, Object ...). This
   * class uses the libQGLViewer library (@see http://www.libqglviewer.com ). It inherits of the
   * class Display3D and permits to display object using a simple
   * stream mechanism of "<<".
   *
   * For instance you can display objects as follows:
   *
   * @code
   * #include <QtGui/qapplication.h>
   * #include "DGtal/helpers/StdDefs.h"
   * #include "DGtal/io/viewers/Viewer3D.h"
   * ...
   * using namespace Z3i;
   * ...
   *
   * Point p1( 0, 0, 0 );
   * Point p2( 5, 5 ,5 );
   * Point p3( 2, 3, 4 );
   * Domain domain( p1, p2 );
   * Viewer3D<S, KS> viewer;
   * viewer.show();
   * viewer << domain;
   * viewer << p1 << p2 << p3;
   * viewer<< Viewer3D::updateDisplay;
   * return application.exec();
   *
   * @endcode
   *
   *
   * @see Display3D, Board3DTo2D
   */
template < class S, class KS>
class Viewer3D : public QGLViewer, public Display3D<S, KS>
{


  //---------------overwritting some functions of Display3D -------------------



  // ----------------------- Standard services ------------------------------
public:

  /**
     * Constructor
     */


  Viewer3D() :QGLViewer(), Display3D<S, KS>()
  //Viewer3D() :QGLViewer(), Display3D()
  {};

  /**
      *Constructor with a khalimsky space
      * @param kspace the Khalimsky space
      */
  Viewer3D(KS KSEmb):QGLViewer(), Display3D<S,KS>(KSEmb)
  {};

  /**
      *Constructor with a space and a khalimsky space
      *@param space a space
      *@param kspace a khalimsky space
      **/
  Viewer3D( S SEmb, KS KSEmb) : QGLViewer(), Display3D<S,KS>(SEmb, KSEmb)
  {};



  /**
     * Set camera position.
     * @param x x position.
     * @param y y position.
     * @param z z position.
     */
  void setCameraPosition(double x, double y, double z)
  { camera_position[0] = x; camera_position[1] = y; camera_position[2] = z; }

  /**
     * Set camera direction.
     * @param x x direction.
     * @param y y direction.
     * @param z z direction.
     */
  void setCameraDirection(double x, double y, double z)
  { camera_direction[0] = x; camera_direction[1] = y; camera_direction[2] = z; }

  /**
     * Set camera up-vector.
     * @param x x coordinate of up-vector.
     * @param y y coordinate of up-vector.
     * @param z z coordinate of up-vector.
     */
  void setCameraUpVector(double x, double y, double z)
  { camera_upVector[0] = x; camera_upVector[1] = y; camera_upVector[2] = z; }

  /**
     * Set near and far distance.
     * @param _near near distance.
     * @param _far far distance.
     */
  void setNearFar(double _near, double _far) { ZNear = _near; ZFar = _far; }




  DGtal::Color myDefaultBackgroundColor;
  DGtal::Color myDefaultColor;
  bool myIsBackgroundDefault;
  bool myViewWire;

  /**
     * Set the default color for future drawing.
     *
     * @param aColor a DGtal::Color (allow to set a trasnparency value).
     *
     **/

  Viewer3D<S, KS> & operator<< ( const DGtal::Color & aColor );



  /**
     * Set the default color for future drawing.
     *
     * @param key a stream key
     *
     **/

  Viewer3D<S, KS> & operator<< ( const typename Viewer3D<S, KS>::StreamKey  & key );


  /**
     *  Sort all surfels from the camera.
     *
     *
     **/

  void sortSurfelFromCamera();

  /**
     *  Sort all triangle from the camera.
     *
     *
     **/

  void sortTriangleFromCamera();
  /**
     *  Sort all triangle from the camera.
     *
     *
     **/

  void sortQuadFromCamera();


  /**
     *  Sort all polygons from the camera.
     *
     *
     **/
  void sortPolygonFromCamera();




  /**
     * Draws the drawable [object] in this board. It should satisfy
     * the concept CDrawableWithViewer3D, which requires for instance a
     * method setStyle( Viewer3D<S, KS> & ).
     *
     * @param object any drawable object.
     * @return a reference on 'this'.
     */
  template <typename TDrawableWithViewer3D>
  Viewer3D<S, KS> & operator<< ( const  TDrawableWithViewer3D & object );



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





  // ------------------------- Hidden services ------------------------------
  //protected:


  /**
     *  Permit to update the OpenGL list to be displayed.
     *  Need to called after a number of addVoxel or after a sortSurfelFromCamera().
     *
     **/
  void updateList ( bool needToUpdateBoundingBox=true );


  /**
     * Draw a linel by using the   [gluCylinder] primitive.
     *
     *
     **/
  void glDrawGLLinel ( typename Viewer3D<S,KS>::lineD3D aLinel );




  /**
     * Draw a linel by using the   [gluCShere] primitive.
     *
     *
     **/
  void glDrawGLPointel ( typename Viewer3D<S,KS>::ballD3D pointel );




  /**
     * Used to manage new key event (wich are added from the default
     * QGLviewer keys).
     *
     * Note that when a new key event is taken into account it could be
     * added in the QGLviewer init() method to update automatically the
     * key description in the help QGLviewer window.  For instance when
     * a new key is processed in this method you simply should add the following
     * code in the init() method:

     @code
     setKeyDescription(Qt::Key_NEW, "Description of the new Key.");
     @endcode

     *
     * @param e the QKeyEvent
     **/

  virtual void keyPressEvent ( QKeyEvent *e );


  /**
     * Used to sort pixel from camera
     **/

  struct compFarthestVoxelFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() (typename Viewer3D<S,KS>::cubeD3D s1, typename Viewer3D<S,KS>::cubeD3D s2 )
    {
      double dist1= sqrt ( ( posCam.x-s1.x ) * ( posCam.x-s1.x ) + ( posCam.y-s1.y ) * ( posCam.y-s1.y ) + ( posCam.z-s1.z ) * ( posCam.z-s1.z ) );
      double dist2= sqrt ( ( posCam.x-s2.x ) * ( posCam.x-s2.x ) + ( posCam.y-s2.y ) * ( posCam.y-s2.y ) + ( posCam.z-s2.z ) * ( posCam.z-s2.z ) );
      return dist1>dist2;
    }
  }
  ;


  struct compFarthestTriangleFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() ( typename Viewer3D<S,KS>::triangleD3D t1, typename Viewer3D<S,KS>::triangleD3D t2 )
    {
      qglviewer::Vec center1 ( ( t1.x1+t1.x2+t1.x3 ) /3.0, ( t1.y1+t1.y2+t1.y3 ) /3.0, ( t1.z1+t1.z2+t1.z3 ) /3.0 );
      qglviewer::Vec center2 ( ( t2.x1+t2.x2+t2.x3 ) /3.0, ( t2.y1+t2.y2+t2.y3 ) /3.0, ( t2.z1+t2.z2+t2.z3 ) /3.0 );
      double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
      double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );

      return dist1>dist2;
    }
  };

  struct compFarthestSurfelFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() (typename Viewer3D<S,KS>::quadD3D q1, typename Viewer3D<S,KS>::quadD3D q2 )
    {

      qglviewer::Vec center1 ( ( q1.x1+q1.x2+q1.x3+q1.x4 ) /4.0, ( q1.y1+q1.y2+q1.y3+q1.y4 ) /4.0, ( q1.z1+q1.z2+q1.z3+q1.z4 ) /4.0 );
      qglviewer::Vec center2 ( ( q2.x1+q2.x2+q2.x3+q2.x4 ) /4.0, ( q2.y1+q2.y2+q2.y3+q2.y4 ) /4.0, ( q2.z1+q2.z2+q2.z3+q2.z4 ) /4.0 );

      double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
      double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );
      return dist1>dist2;
    }
  };




  struct compFarthestPolygonFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() ( typename Viewer3D<S,KS>::polygonD3D q1, typename Viewer3D<S,KS>::polygonD3D q2 )
    {
      double c1x, c1y, c1z=0.0;
      double c2x, c2y, c2z=0.0;
      for(unsigned int i=0; i< q1.vectBalls.size(); i++){
        c1x+=q1.vectBalls.at(i).x;
        c1y+=q1.vectBalls.at(i).y;
        c1z+=q1.vectBalls.at(i).z;
      }
      for(unsigned int i=0; i< q2.vectBalls.size(); i++){
        c2x+=q2.vectBalls.at(i).x;
        c2y+=q2.vectBalls.at(i).y;
        c2z+=q2.vectBalls.at(i).z;
      }

      qglviewer::Vec center1 ( c1x/(double)q1.vectBalls.size(),c1y/(double)q1.vectBalls.size(), c1z/(double)q1.vectBalls.size() );
      qglviewer::Vec center2 ( c2x/(double)q2.vectBalls.size(),c2y/(double)q2.vectBalls.size(), c2z/(double)q2.vectBalls.size() );

      double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
      double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );
      return dist1>dist2;
    }
  };



  virtual void drawWithNames();
  virtual void draw();
  virtual void init();
  virtual QString helpString() const;
  virtual void postSelection ( const QPoint& point );





  // ------------------------- Internals ------------------------------------
private:
  /**
     * Used to display in OPENGL an image as a textured quad image.
     *
     **/
  struct GLTextureImage {
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double x4, y4, z4;
    typename Viewer3D<S, KS>::ImageDirection myDirection;
    unsigned int myImageWidth;
    unsigned int myImageHeight;

    unsigned int myBufferWidth;
    unsigned int myBufferHeight;
    GLuint  myTextureName;
    typename Viewer3D<S, KS>::TextureMode myMode;
    unsigned char *  myTextureImageBufferGS;
    unsigned char *  myTextureImageBufferRGB;
    double vectNormal[3];


    // By definition in OpenGL the image size of texture should power of 2
    double myTextureFitX;
    double myTextureFitY;


    // Destructor
    ~GLTextureImage(){
      if(myMode== Viewer3D<S, KS>::GrayScaleMode){
        if(myTextureImageBufferGS!=0)
          delete [] myTextureImageBufferGS;
      }
      if(myMode== Viewer3D<S, KS>::RGBMode){
        if(myTextureImageBufferRGB!=0)
          delete [] myTextureImageBufferRGB;
      }

    }

    //Copy constructor from a GLTextureImage
    GLTextureImage(const GLTextureImage &aGLImg): myBufferHeight(aGLImg.myBufferHeight),
      myBufferWidth(aGLImg.myBufferWidth),
      myTextureName(aGLImg.myTextureName),
      myTextureFitX(aGLImg.myTextureFitX),
      myTextureFitY(aGLImg.myTextureFitY),
      myMode(aGLImg.myMode)

    {
      x1=aGLImg.x1; y1=aGLImg.y1; z1=aGLImg.z1;
      x2=aGLImg.x2; y2=aGLImg.y2; z2=aGLImg.z2;
      x3=aGLImg.x3; y3=aGLImg.y3; z3=aGLImg.z3;
      x4=aGLImg.x4; y4=aGLImg.y4; z4=aGLImg.z4;
      myImageWidth=aGLImg.myImageWidth; myImageHeight=aGLImg.myImageHeight;
      myDirection = aGLImg.myDirection;
      vectNormal[0]=aGLImg.vectNormal[0];
      vectNormal[1]=aGLImg.vectNormal[1];
      vectNormal[2]=aGLImg.vectNormal[2];

      if(myMode==Viewer3D<S, KS>::GrayScaleMode){
        myTextureImageBufferGS = new unsigned char [myBufferHeight*myBufferWidth];
        for(unsigned int i=0; i<myBufferHeight*myBufferWidth;i++){
          myTextureImageBufferGS[i]=aGLImg.myTextureImageBufferGS[i];
        }
      }else if(myMode==Viewer3D<S, KS>::RGBMode){
        myTextureImageBufferRGB = new unsigned char [3*myBufferHeight*myBufferWidth];
        for(unsigned int i=0; i<3*myBufferHeight*myBufferWidth;i+=3){
          myTextureImageBufferRGB[i]=aGLImg.myTextureImageBufferRGB[i];
          myTextureImageBufferRGB[i+1]=aGLImg.myTextureImageBufferRGB[i+1];
          myTextureImageBufferRGB[i+2]=aGLImg.myTextureImageBufferRGB[i+2];
        }
      }



    }


    //Copy constructor from a TextureImage
    GLTextureImage(const typename Viewer3D<S, KS>::TextureImage &aGSImage)
    {
      x1=aGSImage.x1; y1=aGSImage.y1; z1=aGSImage.z1;
      x2=aGSImage.x2; y2=aGSImage.y2; z2=aGSImage.z2;
      x3=aGSImage.x3; y3=aGSImage.y3; z3=aGSImage.z3;
      x4=aGSImage.x4; y4=aGSImage.y4; z4=aGSImage.z4;
      myImageWidth=aGSImage.myImageWidth; myImageHeight=aGSImage.myImageHeight;
      myDirection = aGSImage.myDirection;
      myMode= aGSImage.myMode;
      vectNormal[0]= (myDirection == Viewer3D<S, KS>::xDirection)? 1.0: 0.0;
      vectNormal[1]= (myDirection == Viewer3D<S, KS>::yDirection)? -1.0: 0.0;
      vectNormal[2]= (myDirection == Viewer3D<S, KS>::zDirection)? 1.0: 0.0;
      myBufferWidth = BasicMathFunctions::roundToUpperPowerOfTwo(myImageWidth);
      myBufferHeight = BasicMathFunctions::roundToUpperPowerOfTwo(myImageHeight);

      if(myMode==Viewer3D<S, KS>::GrayScaleMode){
        myTextureImageBufferGS = new unsigned char [myBufferHeight*myBufferWidth];
        unsigned int pos=0;
        for (unsigned int i=0; i<myBufferHeight; i++){
          for (unsigned int j=0; j<myBufferWidth; j++){
            if(i<myImageHeight && j<  myImageWidth){
              myTextureImageBufferGS[pos]= aGSImage.myTabImage[i*myImageWidth+j];
            }else{
              myTextureImageBufferGS[pos]=0;
            }
            pos++;
          }
        }
      }else if(myMode==Viewer3D<S, KS>::RGBMode){
        myTextureImageBufferRGB = new unsigned char [3*myBufferHeight*myBufferWidth];
        unsigned int pos=0;
        for (unsigned int i=0; i<myBufferHeight; i++){
          for (unsigned int j=0; j<myBufferWidth; j++){
            if(i<myImageHeight && j<  myImageWidth){
              DGtal::Color aCol(aGSImage.myTabImage[i*myImageWidth+j]);
              myTextureImageBufferRGB[pos]= aCol.red();
              myTextureImageBufferRGB[pos+1]= aCol.green();
              myTextureImageBufferRGB[pos+2]= aCol.blue();
            }else{
              myTextureImageBufferRGB[pos]=0;
              myTextureImageBufferRGB[pos+1]=0;
              myTextureImageBufferRGB[pos+2]=0;
            }
            pos+=3;
          }
        }
      }

      myTextureFitX = 1.0-((myBufferWidth-myImageWidth)/(double)myBufferWidth);
      myTextureFitY = 1.0-((myBufferHeight-myImageHeight)/(double)myBufferHeight);
    }

  };

  // ------------------------- Private Datas --------------------------------
private:

  GLuint myListToAff;
  unsigned int myNbListe;
  qglviewer::Vec myOrig, myDir, myDirSelector, mySelectedPoint;
  QPoint myPosSelector;
  std::vector<GLTextureImage> myVectTextureImage;
  bool myIsDoubleFaceRendering;

  double camera_position[3];  //!< camera position
  double camera_direction[3];  //!< camera direction
  double camera_upVector[3];  //!< camera up-vector

  double ZNear;      //!< znear distance
  double ZFar;      //!< zfar distance


}; // end of class Viewer3D








/**
   * Overloads 'operator<<' for displaying objects of class 'Viewer3D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Viewer3D' to write.
   * @return the output stream after the writing.
   */
template < typename S, typename KS>
std::ostream&
operator<< ( std::ostream & out, const Viewer3D<S, KS> & object );


} // namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/Viewer3D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Viewer3D_h

#undef Viewer3D_RECURSES
#endif // else defined(Viewer3D_RECURSES)
