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
   * Viewer3D viewer;
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
  class Viewer3D : public QGLViewer, public Display3D
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     */

    Viewer3D() :QGLViewer(), Display3D()
    {

    };




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

    Viewer3D & operator<< ( const DGtal::Color & aColor );



    /**
     * Set the default color for future drawing.
     *
     * @param key a stream key
     *
     **/

    Viewer3D & operator<< ( const Display3D::StreamKey  & key );





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
     * method setStyle( Viewer3D & ).
     *
     * @param object any drawable object.
     * @return a reference on 'this'.
     */
    template <typename TDrawableWithDisplay3D>
    Viewer3D & operator<< ( const  TDrawableWithDisplay3D & object );



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



    // ------------------------- Private Datas --------------------------------
  private:


    GLuint myListToAff;
    unsigned int myNbListe;
    qglviewer::Vec myOrig, myDir, myDirSelector, mySelectedPoint;
    QPoint myPosSelector;
  
  public:

    // ------------------------- Hidden services ------------------------------
  protected:
  

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
    void glDrawGLLinel ( lineD3D aLinel );




    /**
     * Draw a linel by using the   [gluCSphere] primitive.
     *
     *
     **/
    void glDrawGLPointel ( pointD3D pointel );




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
      bool operator() ( voxelD3D s1, voxelD3D s2 )
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
      bool operator() ( triangleD3D t1, triangleD3D t2 )
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
      bool operator() ( quadD3D q1, quadD3D q2 )
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
      bool operator() ( polygonD3D q1, polygonD3D q2 )
      {
	double c1x, c1y, c1z=0.0;
	double c2x, c2y, c2z=0.0;
	for(unsigned int i=0; i< q1.vectPoints.size(); i++){
	  c1x+=q1.vectPoints.at(i).x;
	  c1y+=q1.vectPoints.at(i).y;
	  c1z+=q1.vectPoints.at(i).z;
	}
	for(unsigned int i=0; i< q2.vectPoints.size(); i++){
	  c2x+=q2.vectPoints.at(i).x;
	  c2y+=q2.vectPoints.at(i).y;
	  c2z+=q2.vectPoints.at(i).z;
	}
	
	qglviewer::Vec center1 ( c1x/(double)q1.vectPoints.size(),c1y/(double)q1.vectPoints.size(), c1z/(double)q1.vectPoints.size() );
	qglviewer::Vec center2 ( c2x/(double)q2.vectPoints.size(),c2y/(double)q2.vectPoints.size(), c2z/(double)q2.vectPoints.size() );

	double dist1= sqrt ( ( posCam.x-center1.x ) * ( posCam.x-center1.x ) + ( posCam.y-center1.y ) * ( posCam.y-center1.y ) + ( posCam.z-center1.z ) * ( posCam.z-center1.z ) );
	double dist2= sqrt ( ( posCam.x-center2.x ) * ( posCam.x-center2.x ) + ( posCam.y-center2.y ) * ( posCam.y-center2.y ) + ( posCam.z-center2.z ) * ( posCam.z-center2.z ) );
	return dist1>dist2;
      }
    };







  protected :
    virtual void drawWithNames();
    virtual void draw();
    virtual void init();
    virtual QString helpString() const;
    virtual void postSelection ( const QPoint& point );



    // ------------------------- Internals ------------------------------------
  private:






  }; // end of class Viewer3D








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
