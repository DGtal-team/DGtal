/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESSpace FOR A PARTICULAR PURPOSE.  See the
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

#include "DGtal/kernel/CSpace.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Viewer3D
/**
   * Description of class 'Viewer3D' <p>
   * Aim: Display 3D
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
   * Viewer3D<> viewer;
   * viewer.show();
   * viewer << domain;
   * viewer << p1 << p2 << p3;
   * viewer<< Viewer3D<>::updateDisplay;
   * return application.exec();
   *
   * @endcode
   *
   *
   * @see Display3D, Board3DTo2D
   */
template < typename  Space = Z3i::Space, typename KSpace = Z3i::KSpace>
class Viewer3D : public QGLViewer, public Display3D<Space, KSpace>
{

  BOOST_CONCEPT_ASSERT((CSpace<Space>));

  //---------------overwritting some functions of Display3D -------------------

  // ----------------------- Standard services ------------------------------
public:

  /**
     * Constructor
     */
  Viewer3D() :QGLViewer(), Display3D<Space, KSpace>()
  //Viewer3D() :QGLViewer(), Display3D()
  {};

  /**
      *Constructor with a khalimsky space
      * @param KSEmb the Khalimsky space
      */
  Viewer3D(KSpace KSEmb):QGLViewer(), Display3D<Space,KSpace>(KSEmb)
  {};

  /**
      *Constructor with a space and a khalimsky space
      *@param SEmb a space
      *@param KSEmb a khalimsky space
      **/
  Viewer3D( Space SEmb, KSpace KSEmb) : QGLViewer(), Display3D<Space,KSpace>(SEmb, KSEmb)
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

  enum ImageDirection {xDirection, yDirection, zDirection };
  enum TextureMode {RGBMode, GrayScaleMode };


  //the default background color of the viewer
  DGtal::Color myDefaultBackgroundColor;
  //the default color of the viewer
  DGtal::Color myDefaultColor;
  // true if the background is default
  bool myIsBackgroundDefault;
  //TODO desc attributes
  bool myViewWire;

  /**
     *  Used to display the 2D domain of an image.
     *
     **/
  //have to be public because of external functions
  struct Image2DDomainD3D
  {
    // The image domain coordinates
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double x4, y4, z4;
    unsigned char R,G,B,T;

    // the width and height of the image domain
    unsigned int myDomainWidth;
    unsigned int myDomainHeight;

    //TODO uncommented sub attributes
    ImageDirection myDirection;

    //the mode of the image
    std::string myMode;

    //TODO uncommented sub attributes
    unsigned int myLineSetIndex;

    /**
        * Constructor
         * @param aDomain
         * @param normalDir
         * @param xBottomLeft
         * @param yBottomLeft
         * @param zBottomLeft
         * @param mode
         */
    template<typename TDomain>
    Image2DDomainD3D( TDomain aDomain, Viewer3D::ImageDirection normalDir=zDirection,
                      double xBottomLeft=0.0, double yBottomLeft=0.0, double zBottomLeft=0.0, std::string mode= "BoundingBox")
    {
      BOOST_CONCEPT_ASSERT(( CDomain < TDomain >));
      myMode = mode;
      myDirection=normalDir;
      myDomainWidth = (aDomain.upperBound())[0]-(aDomain.lowerBound())[0]+1;
      myDomainHeight = (aDomain.upperBound())[1]-(aDomain.lowerBound())[1]+1;
      updateDomainOrientation(normalDir, xBottomLeft, yBottomLeft, zBottomLeft);
    }

    /**
     * Update the domain direction from a specific normal direction
     * (Viewer3D::xDirection, Viewer3D::yDirection or Viewer3D::zDirection) and image position
     * from the botton left point.
     * @param normalDir give a predifined normal orientation can be (Viewer3D::xDirection, Viewer3D::yDirection or Viewer3D::zDirection)
     *  @param xBottomLeft the x coordinate of bottom left image point.
     *  @param yBottomLeft the x coordinate of bottom left image point.
     *  @param zBottomLeft the x coordinate of bottom left image point.
     **/
    void updateDomainOrientation( Viewer3D::ImageDirection normalDir, double xBottomLeft, double yBottomLeft, double zBottomLeft);


    /**
     *  Translate domain postion.
     *  @param xTranslation the image translation in the  x direction (default 0).
     *  @param yTranslation the image translation in the  y direction (default 0).
     *  @param zTranslation the image translation in the  z direction (default 0).
     **/
    void translateDomain(double xTranslation=0.0, double yTranslation=0.0, double zTranslation=0.0);

  };


  /**
   * Used to display an image as a textured quad image.
   *
   **/
  struct TextureImage
  {

    // The quad coordinates should be given in counter clockwise order
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double x4, y4, z4;

    //TODO uncommented sub attributes
    ImageDirection myDirection;

    // the width and height of the image
    unsigned int myImageWidth;
    unsigned int myImageHeight;
    unsigned int * myTabImage;

    //TODO uncommented sub attributes
    bool myDrawDomain;
    unsigned int myIndexDomain;
    TextureMode myMode;

    ~TextureImage()
    {
      delete [] myTabImage;
    };

    /**
     * Copy constructor (needed due to myTabImage)
   * @param img
   */
    TextureImage(const TextureImage & img):x1(img.x1), y1(img.y1), z1(img.z1),
      x2(img.x2), y2(img.y2), z2(img.z2),
      x3(img.x3), y3(img.y3), z3(img.z3),
      x4(img.x4), y4(img.y4), z4(img.z4),
      myDirection(img.myDirection), myImageWidth(img.myImageWidth),
      myImageHeight(img.myImageHeight),
      myTabImage(img.myTabImage),
      myDrawDomain(img.myDrawDomain),
      myIndexDomain(img.myIndexDomain),
      myMode(img.myMode)
    {

      if(img.myImageHeight>0 && img.myImageWidth>0)
      {
        myTabImage = new  unsigned int [img.myImageWidth*img.myImageHeight];
        for(unsigned int i=0; i<img.myImageWidth*img.myImageHeight; i++)
        {
          myTabImage[i] = img.myTabImage[i];
        }
      }else
      {
        myTabImage=img.myTabImage;
      }
    };



    /**
     *  Constructor that fills image parameters from std image (image buffer, dimensions, vertex coordinates, orientation)
     *
     *  @tparam TImageType the type of the image given for the constructor (should follow the CConstImage concept).
     *  @tparam TFunctor the functor type (should follow the
     *  CUnaryFunctor concept with image value type as input type
     *  and unsigned int as output type).
     *
     *
     *  @param image the source image.
     *  @param aFunctor a functor to transform input values to the output displayed values.
     *  @param normalDir the direction of normal vector of the image plane (xDirection, yDirection or zDirection (default)) .
     *  @param xBottomLeft the x coordinate of bottom left image point (default 0).
     *  @param yBottomLeft the x coordinate of bottom left image point (default 0).
     *  @param zBottomLeft the x coordinate of bottom left image point (default 0).
     *  @param aMode
*/
    template <typename TImageType, typename TFunctor>

    TextureImage( const TImageType & image, const TFunctor &aFunctor,
                  Viewer3D::ImageDirection normalDir=zDirection,
                  double xBottomLeft=0.0, double yBottomLeft=0.0, double zBottomLeft=0.0,
                  TextureMode aMode= Viewer3D::GrayScaleMode)
    {
      BOOST_CONCEPT_ASSERT(( CConstImage < TImageType > ));
      BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, typename TImageType::Value, unsigned int> )) ;
      myDrawDomain=false;
      myDirection=normalDir;
      myImageWidth = (image.domain().upperBound())[0]-(image.domain().lowerBound())[0]+1;
      myImageHeight = (image.domain().upperBound())[1]-(image.domain().lowerBound())[1]+1;
      myTabImage = new  unsigned int [myImageWidth*myImageHeight];
      updateImageOrientation(normalDir, xBottomLeft, yBottomLeft, zBottomLeft);
      myMode=aMode;
      updateImageDataAndParam(image, aFunctor);
    };

    /**
     * Update the image direction from a specific normal direction
     * (Viewer3D::xDirection, Viewer3D::yDirection or Viewer3D::zDirection) and image position
     * from the botton left point.
     * @param normalDir give a predifined normal orientation can be (Viewer3D::xDirection, Viewer3D::yDirection or Viewer3D::zDirection)
     *  @param xBottomLeft the x coordinate of bottom left image point.
     *  @param yBottomLeft the x coordinate of bottom left image point.
     *  @param zBottomLeft the x coordinate of bottom left image point.
     **/
    void updateImageOrientation( Viewer3D::ImageDirection normalDir, double xBottomLeft, double yBottomLeft, double zBottomLeft);


    /**
     *  Update the  image parameters from std image (image buffer, vertex coordinates)
     *  The new image should be with same dimension than the original.
     *
     *  @tparam TImageType the type of the image given for the constructor (should follow the CConstImage concept).
     *  @tparam TFunctor the functor type (should follow the
     *  CUnaryFunctor concept with image value type as input type
     *  and unsigned int as output type).
     *
     *  @param image the source image.
     *  @param aFunctor a functor to transform input values to the output displayed values.
     *  @param xTranslation the image translation in the  x direction (default 0).
     *  @param yTranslation the image translation in the  y direction (default 0).
     *  @param zTranslation the image translation in the  z direction (default 0).
     **/
    template <typename TImageType, typename TFunctor>
    void updateImageDataAndParam(const TImageType & image, const TFunctor &aFunctor, double xTranslation=0.0,
                                 double yTranslation=0.0, double zTranslation=0.0)
    {
      BOOST_CONCEPT_ASSERT(( CConstImage < TImageType > ));
      BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, typename TImageType::Value, unsigned int> )) ;
      assert ( (image.domain().upperBound())[0]-(image.domain().lowerBound())[0]+1== myImageWidth &&
          (image.domain().upperBound())[1]-(image.domain().lowerBound())[1]+1== myImageHeight);

      x1 += xTranslation; y1 += yTranslation; z1 += zTranslation;
      x2 += xTranslation; y2 += yTranslation; z2 += zTranslation;
      x3 += xTranslation; y3 += yTranslation; z3 += zTranslation;
      x4 += xTranslation; y4 += yTranslation; z4 += zTranslation;

      unsigned int pos=0;
      for(typename TImageType::Domain::ConstIterator it = image.domain().begin(), itend=image.domain().end();
          it!=itend; ++it)
      {
        myTabImage[pos]= aFunctor(image(*it));
        pos++;
      }
    };

    /**
     * return the class name to implment the CDrawableWithViewer3D concept.
     **/
    std::string className() const;

  private:
    /**
   * default constructor
   * TextureImage
   */
    TextureImage()
    {};
  };

  /**
     * Set the default color for future drawing.
     * @param aColor a DGtal::Color (allow to set a trasnparency value).
     * @return a reference on 'this'.
     **/
  Viewer3D<Space, KSpace> & operator<< ( const DGtal::Color & aColor );


  /**
     * Set the default color for future drawing.
     * @param key a stream key
     * @return a reference on 'this'.
     **/
  Viewer3D<Space, KSpace> & operator<< ( const typename Viewer3D<Space, KSpace>::StreamKey  & key );


  /**
     *  Sort all surfels from the camera.
     **/
  void sortSurfelFromCamera();

  /**
     *  Sort all triangle from the camera.
     **/
  void sortTriangleFromCamera();

  /**
     *  Sort all triangle from the camera.
     **/
  void sortQuadFromCamera();

  /**
     *  Sort all polygons from the camera.
     **/
  void sortPolygonFromCamera();



  template <typename TDrawableWithViewer3D>
  /**
     * Draws the drawable [object] in this board. It should satisfy
     * the concept CDrawableWithViewer3D, which requires for instance a
     * method setStyle( Viewer3D<Space, KSpace> & ).
     *
     * @param object any drawable object.
     * @return a reference on 'this'.
     */
  Viewer3D<Space, KSpace> & operator<< ( const  TDrawableWithViewer3D & object );



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



  virtual unsigned int getCurrentDomainNumber();

  virtual unsigned int getCurrentGLImageNumber();


  /**
     * Add a TextureImage in the list of image to be displayed.
     * @param image a TextureImage including image data buffer and position, orientation.
     *
     **/
  void addTextureImage(const TextureImage &image);


  /**
     * Update the  image parameters from std image (image buffer, vertex coordinates)
     * The new image should be with same dimension than the original.
     * @param imageIndex corresponds to the chronoloigic index given by the fuction (addTextureImage).
     * @param image the new image containing the new buffer (with same dimensions than the other image).
     * @param aFunctor
     * @param xTranslation the image translation in the  x direction (default 0).
     * @param yTranslation the image translation in the  y direction (default 0).
     * @param zTranslation the image translation in the  z direction (default 0).
     **/
  template <typename TImageType, typename TFunctor>

  void updateTextureImage(unsigned int imageIndex, const  TImageType & image, const  TFunctor & aFunctor,
                          double xTranslation=0.0, double yTranslation=0.0, double zTranslation=0.0);




  /**
     * Update the  image parameters from std image (image buffer, vertex coordinates)
     * The new image should be with same dimension than the original.
     * @param imageIndex corresponds to the chronoloigic index given by the fuction (addTextureImage).
     * @param xPosition the image translation in the  x direction (default 0).
     * @param yPosition the image translation in the  y direction (default 0).
     * @param zPosition the image translation in the  z direction (default 0).
   * @param newDirection
   */
  void updateOrientationTextureImage(unsigned int imageIndex,
                                     double xPosition, double yPosition, double zPosition, ImageDirection newDirection);



  /**
   * add an 2D image in a 3d space
   * @param anImageDomain an image and its domain
   * @param mode mode of representation
   * @param aColor color of the image
   */
  template<typename TDomain>
  void addImage2DDomainD3D(const TDomain &anImageDomain, std::string mode,
                           const DGtal::Color &aColor=DGtal::Color::Red );


  /**
   * update a 2D domain orientation
   * @param imageIndex index of the image in the list
   * @param xPosition
   * @param yPosition
   * @param zPosition
   * @param newDirection the new direction
   */
  void updateAn2DDomainOrientation(unsigned int imageIndex,
                                   double xPosition, double yPosition, double zPosition, ImageDirection newDirection);

  /**
   * translate a 2D domain
   * @param domainIndex the index of the domain in the list
   * @param xTranslation x part of the translation vector
   * @param yTranslation y part of the translation vector
   * @param zTranslation z part of the translation vector
   */
  void translateAn2DDomain(unsigned int domainIndex, double xTranslation, double yTranslation, double zTranslation);

  /**
   * @brief compute2DDomainLineRepresentation
   * @param anImageDomain
   * @param delta
   * @return
   */
  std::vector<typename DGtal::Viewer3D< Space , KSpace >::lineD3D>  compute2DDomainLineRepresentation( Image2DDomainD3D &anImageDomain, double delta );

  /**
   * @brief compute2DDomainLineRepresentation
   * @param anImageDomain
   * @return
   */
  std::vector<typename DGtal::Viewer3D< Space , KSpace >::lineD3D>  compute2DDomainLineRepresentation( Image2DDomainD3D &anImageDomain);






  // ------------------------- Protected Datas ------------------------------
private:



public:





  // ------------------------- Hidden services ------------------------------
  //protected:


  /**
     *  Permit to update the OpenGL list to be displayed.
     *  Need to called after a number of addVoxel or after a sortSurfelFromCamera().
   * @param needToUpdateBoundingBox
   */
  void updateList ( bool needToUpdateBoundingBox=true );


  /**
     * Draw a linel by using the   [gluCylinder] primitive.
     * @param aLinel
     **/
  void glDrawGLLinel ( typename Viewer3D<Space,KSpace>::lineD3D aLinel );




  /**
     * Draw a linel by using the   [gluCShere] primitive.
   * @param pointel
   */
  void glDrawGLPointel ( typename Viewer3D<Space,KSpace>::ballD3D pointel );




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
    bool operator() (typename Viewer3D<Space,KSpace>::cubeD3D s1, typename Viewer3D<Space,KSpace>::cubeD3D s2 )
    {
      double dist1= sqrt ( ( posCam.x-s1.x ) * ( posCam.x-s1.x ) + ( posCam.y-s1.y ) * ( posCam.y-s1.y ) + ( posCam.z-s1.z ) * ( posCam.z-s1.z ) );
      double dist2= sqrt ( ( posCam.x-s2.x ) * ( posCam.x-s2.x ) + ( posCam.y-s2.y ) * ( posCam.y-s2.y ) + ( posCam.z-s2.z ) * ( posCam.z-s2.z ) );
      return dist1>dist2;
    }
  };


  /**
     * Used to sort pixel from camera
     **/
  struct compFarthestTriangleFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() ( typename Viewer3D<Space,KSpace>::triangleD3D t1, typename Viewer3D<Space,KSpace>::triangleD3D t2 )
    {
      qglviewer::Vec center1 ( ( t1.x1+t1.x2+t1.x3 ) /3.0, ( t1.y1+t1.y2+t1.y3 ) /3.0, ( t1.z1+t1.z2+t1.z3 ) /3.0 );
      qglviewer::Vec center2 ( ( t2.x1+t2.x2+t2.x3 ) /3.0, ( t2.y1+t2.y2+t2.y3 ) /3.0, ( t2.z1+t2.z2+t2.z3 ) /3.0 );
      double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
      double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );

      return dist1>dist2;
    }
  };

  /**
     * Used to sort pixel from camera
     **/
  struct compFarthestSurfelFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() (typename Viewer3D<Space,KSpace>::quadD3D q1, typename Viewer3D<Space,KSpace>::quadD3D q2 )
    {

      qglviewer::Vec center1 ( ( q1.x1+q1.x2+q1.x3+q1.x4 ) /4.0, ( q1.y1+q1.y2+q1.y3+q1.y4 ) /4.0, ( q1.z1+q1.z2+q1.z3+q1.z4 ) /4.0 );
      qglviewer::Vec center2 ( ( q2.x1+q2.x2+q2.x3+q2.x4 ) /4.0, ( q2.y1+q2.y2+q2.y3+q2.y4 ) /4.0, ( q2.z1+q2.z2+q2.z3+q2.z4 ) /4.0 );

      double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
      double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );
      return dist1>dist2;
    }
  };

  /**
     * Used to sort pixel from camera
     **/
  struct compFarthestPolygonFromCamera
  {
    qglviewer::Vec posCam;
    bool operator() ( typename Viewer3D<Space,KSpace>::polygonD3D q1, typename Viewer3D<Space,KSpace>::polygonD3D q2 )
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



  /**
   * @brief drawWithNames
   */
  virtual void drawWithNames();

  /**
   * @brief draw
   */
  virtual void draw();

  /**
   * @brief init
   */
  virtual void init();

  /**
   * @return astring corresponding to the help of the viewer (list of commands, etc)
   */
  virtual QString helpString() const;

  /**
   * @brief postSelection
   * @param point
   */
  virtual void postSelection ( const QPoint& point );





  // ------------------------- Internals ------------------------------------
private:


  /**
     * Used to display in OPENGL an image as a textured quad image.
     *
     **/
  struct GLTextureImage
  {
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double x4, y4, z4;
    typename Viewer3D<Space, KSpace>::ImageDirection myDirection;
    unsigned int myImageWidth;
    unsigned int myImageHeight;

    unsigned int myBufferWidth;
    unsigned int myBufferHeight;
    GLuint  myTextureName;
    typename Viewer3D<Space, KSpace>::TextureMode myMode;
    unsigned char *  myTextureImageBufferGS;
    unsigned char *  myTextureImageBufferRGB;
    double vectNormal[3];


    // By definition in OpenGL the image size of texture should power of 2
    double myTextureFitX;
    double myTextureFitY;


    // Destructor
    ~GLTextureImage()
    {
      if(myMode== Viewer3D<Space, KSpace>::GrayScaleMode)
      {
        if(myTextureImageBufferGS!=0)
          delete [] myTextureImageBufferGS;
      }
      if(myMode== Viewer3D<Space, KSpace>::RGBMode)
      {
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

      if(myMode==Viewer3D<Space, KSpace>::GrayScaleMode)
      {
        myTextureImageBufferGS = new unsigned char [myBufferHeight*myBufferWidth];
        for(unsigned int i=0; i<myBufferHeight*myBufferWidth;i++)
        {
          myTextureImageBufferGS[i]=aGLImg.myTextureImageBufferGS[i];
        }
      }else if(myMode==Viewer3D<Space, KSpace>::RGBMode)
      {
        myTextureImageBufferRGB = new unsigned char [3*myBufferHeight*myBufferWidth];
        for(unsigned int i=0; i<3*myBufferHeight*myBufferWidth;i+=3)
        {
          myTextureImageBufferRGB[i]=aGLImg.myTextureImageBufferRGB[i];
          myTextureImageBufferRGB[i+1]=aGLImg.myTextureImageBufferRGB[i+1];
          myTextureImageBufferRGB[i+2]=aGLImg.myTextureImageBufferRGB[i+2];
        }
      }
    }


    //Copy constructor from a TextureImage
    GLTextureImage(const typename Viewer3D<Space, KSpace>::TextureImage &aGSImage)
    {
      x1=aGSImage.x1; y1=aGSImage.y1; z1=aGSImage.z1;
      x2=aGSImage.x2; y2=aGSImage.y2; z2=aGSImage.z2;
      x3=aGSImage.x3; y3=aGSImage.y3; z3=aGSImage.z3;
      x4=aGSImage.x4; y4=aGSImage.y4; z4=aGSImage.z4;
      myImageWidth=aGSImage.myImageWidth; myImageHeight=aGSImage.myImageHeight;
      myDirection = aGSImage.myDirection;
      myMode= aGSImage.myMode;
      vectNormal[0]= (myDirection == Viewer3D<Space, KSpace>::xDirection)? 1.0: 0.0;
      vectNormal[1]= (myDirection == Viewer3D<Space, KSpace>::yDirection)? -1.0: 0.0;
      vectNormal[2]= (myDirection == Viewer3D<Space, KSpace>::zDirection)? 1.0: 0.0;
      myBufferWidth = BasicMathFunctions::roundToUpperPowerOfTwo(myImageWidth);
      myBufferHeight = BasicMathFunctions::roundToUpperPowerOfTwo(myImageHeight);

      if(myMode==Viewer3D<Space, KSpace>::GrayScaleMode)
      {
        myTextureImageBufferGS = new unsigned char [myBufferHeight*myBufferWidth];
        unsigned int pos=0;
        for (unsigned int i=0; i<myBufferHeight; i++)
        {
          for (unsigned int j=0; j<myBufferWidth; j++)
          {
            if(i<myImageHeight && j<  myImageWidth)
            {
              myTextureImageBufferGS[pos]= aGSImage.myTabImage[i*myImageWidth+j];
            }else{
              myTextureImageBufferGS[pos]=0;
            }
            pos++;
          }
        }
      }else if(myMode==Viewer3D<Space, KSpace>::RGBMode)
      {
        myTextureImageBufferRGB = new unsigned char [3*myBufferHeight*myBufferWidth];
        unsigned int pos=0;
        for (unsigned int i=0; i<myBufferHeight; i++)
        {
          for (unsigned int j=0; j<myBufferWidth; j++)
          {
            if(i<myImageHeight && j<  myImageWidth)
            {
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

  //TODO desc attributes
  GLuint myListToAff;
  //TODO desc attributes
  unsigned int myNbListe;
  //TODO desc attributes
  qglviewer::Vec myOrig, myDir, myDirSelector, mySelectedPoint;
  //TODO desc attributes
  QPoint myPosSelector;
  //TODO desc attributes
  std::vector<GLTextureImage> myVectTextureImage;
  //TODO desc attributes
  bool myIsDoubleFaceRendering;

  double camera_position[3];  //!< camera position
  double camera_direction[3];  //!< camera direction
  double camera_upVector[3];  //!< camera up-vector

  double ZNear;      //!< znear distance
  double ZFar;      //!< zfar distance

  //the default width of the mesh line
  float myMeshDefaultLineWidth;

  /// Used to store all displayed images
  std::vector<TextureImage> myGSImageList;
  /// Used to store all the domain
  std::vector<Image2DDomainD3D> myImageDomainList;


}; // end of class Viewer3D



template < typename Space, typename KSpace>
/**
   * Overloads 'operator<<' for displaying objects of class 'Viewer3D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Viewer3D' to write.
   * @return the output stream after the writing.
   */
std::ostream&
operator<< ( std::ostream & out, const Viewer3D<Space, KSpace> & object );
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/Viewer3D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Viewer3D_h

#undef Viewer3D_RECURSES
#endif // else defined(Viewer3D_RECURSES)
