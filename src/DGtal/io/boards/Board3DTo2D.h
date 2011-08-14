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
 * @file   Board3DTo2D.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 22 juin 2011
 * 
 * @brief
 *
 * Header file for module Board3DTo2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Board3DTo2D_RECURSES)
#error Recursive header files inclusion detected in Board3DTo2D.h
#else // defined(Board3DTo2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Board3DTo2D_RECURSES

#if !defined Board3DTo2D_h
/** Prevents repeated inclusion of headers. */
#define Board3DTo2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


/////////////////////////////////////////////////////////////////////////////
// class Board3DTo2D
/**
 * Description of class 'Board3DTo2D' <p>
 * @brief Class for PDF, PNG, PS, EPS, SVG export drawings with 3D->2D projection.
 */
  class Board3DTo2D : public Display3D
{
public:
  /**
   * Cairo type for save files.
   */
  enum CairoType { CairoPDF, CairoPNG, CairoPS, CairoEPS, CairoSVG };
  
  /*!
   * \brief Constructor.
   */
  Board3DTo2D();
  
  
  ~Board3DTo2D(){};
  

  /**
    * @return the style name used for drawing this object.
    */
  std::string styleName() const
  {
    return "Board3DTo2D";
  }
  
  /**
  * Set camera position.
  * @param x x position.
  * @param y y position.
  * @param z z position.
  */
  void setCameraPosition(double x, double y, double z) { camera_position[0] = x; camera_position[1] = y; camera_position[2] = z; }
  
  /**
  * Set camera direction.
  * @param x x direction.
  * @param y y direction.
  * @param z z direction.
  */
  void setCameraDirection(double x, double y, double z) { camera_direction[0] = x; camera_direction[1] = y; camera_direction[2] = z; }
  
  /**
  * Set camera up-vector.
  * @param x x coordinate of up-vector.
  * @param y y coordinate of up-vector.
  * @param z z coordinate of up-vector.
  */
  void setCameraUpVector(double x, double y, double z) { camera_upVector[0] = x; camera_upVector[1] = y; camera_upVector[2] = z; }
  
  /**
  * Set near and far distance.
  * @param near near distance.
  * @param far far distance.
  */
  void setNearFar(double near, double far) { ZNear = near; ZFar = far; }
  
  /**
  * Save a Cairo image.
  * @param filename filename of the image to save.
  * @param type type of the image to save (CairoPDF, CairoPNG, CairoPS, CairoEPS, CairoSVG).
  * @param width width of the image to save.
  * @param height height of the image to save.
  */
  void saveCairo(const char *filename, CairoType type, int width, int height);
  
  /**
   * The associated map type for storing possible modes used for
   * displaying for digital objects.
   */
  //typedef std::map< std::string, std::string > ModeMapping;

 //  /**
//    * The associated map type for storing the default styles of
//    * digital objects.
//    */
//   typedef std::map< std::string,CountedPtr<DrawableWithDisplay3D> > StyleMapping;
  
  DGtal::Color myDefaultColor;	//!< default color

  
  /**
   * Used to create a new list containing new Voxel objects
   * (useful to use transparency between different objects).
   * 
   **/  
  void createNewVoxelList(bool depthTest=true);
  
  /**
   * Used to create a new list containing new Line objects
   * (useful to use transparency between different objects).
   * 
   **/  
  void createNewLineList();

  /**
   * Used to create a new list containing new Point objects
   * (useful to use transparency between different objects).
   * 
   **/  
  void createNewPointList();

 
  /**
   * Set the default color for future drawing.
   *
   * @param aColor: a DGtal::Color (allow to set a trasnparency value).
   *
   **/  
  Board3DTo2D & operator<<(const DGtal::Color & aColor);

  /**
   * Add a point as a 3d voxel using default color in the current list.
   * A voxel can be added in a new list by calling: @createNewList().
   * @param x x position.
   * @param y y position.
   * @param z z position.
   * @param color: the color of the voxelD3D (default: 220, 220, 220).
   * @param width: the width of the voxelD3D (default: 0.5).
   *
   **/
  void addVoxel(int x, int y, int z, DGtal::Color color=DGtal::Color(220, 220, 220), double width=0.5, bool withWire=false);
  
  /**
   * Add a point using default color in the current list.
   * A point can be added in a new list by calling: @createNewList().
   * @param x x position.
   * @param y y position.
   * @param z z position.
   * @param color: the color of the point (default: 200,20,20).
   * @param size: the size of the point (default: 40).
   *
   **/
  void addPoint(double x, double y, double z ,const DGtal::Color &color=DGtal::Color(200,20,20), double size=40);

  /**
   * Add a line using default color in the current list.
   * A line can be added in a new list by calling: @createNewList().
   * @param x1 x position of first point.
   * @param y1 y position of first point.
   * @param z1 z position of first point.
   * @param x2 x position of second point.
   * @param y2 y position of second point.
   * @param z2 z position of second point.
   * @param color: the color of the line (default: 20,20,20,200).
   * @param width: the width of the line (default: 1.5).
   *
   **/
  void addLine(double x1, double y1, double z1, double x2, double y2, double z2, 
	       const DGtal::Color &color=DGtal::Color(20,20,20,200), double width=1.5);
  
  /**
   * Add a quad using default color in the current list.
   * A quad can be added in a new list by calling: @createNewList().
   * @param x1 x position of first point.
   * @param y1 y position of first point.
   * @param z1 z position of first point.
   * @param x2 x position of second point.
   * @param y2 y position of second point.
   * @param z2 z position of second point.
   * @param x3 x position of third point.
   * @param y3 y position of third point.
   * @param z3 z position of third point.
   * @param x4 x position of fourth point.
   * @param y4 y position of fourth point.
   * @param z4 z position of fourth point.
   * @param color: the color of the quad.
   *
   **/
  void addQuad(double x1, double y1, double z1,  double x2, double y2, double z2,
  	       double x3, double y3, double z3,  double x4, double y4, double z4, DGtal::Color aColor);

  
  /**
   * Add a KSSurfel using default color in the current list.
   * A KSSurfel can be added in a new list by calling: @createNewList().
   * @param x1 x position of first point.
   * @param y1 y position of first point.
   * @param z1 z position of first point.
   * @param x2 x position of second point.
   * @param y2 y position of second point.
   * @param z2 z position of second point.
   * @param x3 x position of third point.
   * @param y3 y position of third point.
   * @param z3 z position of third point.
   * @param x4 x position of fourth point.
   * @param y4 y position of fourth point.
   * @param z4 z position of fourth point.
   * @param color: the color of the KSSurfel (default: 180,180,250,255).
   *
   **/
  void addKSSurfel(double x, double y, double z, 
		   bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor, 
		   bool isOriented= false, bool isOrientedPositively=true, bool basicMode=false);

  /**
   * Add KSVoxel using default color in the current list.
   * A KSVoxel can be added in a new list by calling: @createNewList().
   * @param x x position.
   * @param y y position.
   * @param z z position.
   * @param color: the color of the KSVoxel (default: 255,180,250,255).
   *
   **/
  void addKSVoxel(int x, int y, int z);
  
  /**
   * Add a KSPointel using default color in the current list.
   * A KSPointel can be added in a new list by calling: @createNewList().
   * @param x x position.
   * @param y y position.
   * @param z z position.
   * @param size: the size of the KSPointel (default: 0.1).
   * @param color: the color of the KSPointel (default: 200,20,20,255).
   *
   **/
  void addKSPointel(double x, double y, double z, double size=0.1,
		    bool isSigned=false, bool signPos=true);
  
  /**
   * Add a KSLinel using default color in the current list.
   * A KSLinel can be added in a new list by calling: @createNewList().
   * @param x1 x position of first point.
   * @param y1 y position of first point.
   * @param z1 z position of first point.
   * @param x2 x position of second point.
   * @param y2 y position of second point.
   * @param z2 z position of second point.
   * @param width: the width of the KSLinel (default: 0.02).
   * @param color: the color of the KSLinel (default: 20,20,200,255).
   *
   **/
  void addKSLinel(double x1, double y1, double z1,
		  double x2, double y2, double z2,
		  double width, bool isSigned, bool signPos);
  
  /**
   * Add a new 3D Clipping plane represented by ax+by+cz+d = 0 
   * A maximal of five clipping plane can be added.
   *
   * @param a, b, c, d : plane equation.
   **/
  void addClippingPlane(double a, double b, double c, double d, bool drawPlane);

  
 
  

  /**
   * Draws the drawable [object] in this board. It should satisfy
   * the concept CDrawableWithDisplay3D, which requires for instance a
   * method selfDraw( Board3DTo2D & ).
   *
   * @param object any drawable object.
   * @return a reference on 'this'.
   */
  template <typename TDrawableWithDisplay3D>
  Board3DTo2D & operator<<( const  TDrawableWithDisplay3D & object );

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

public:
  

    // ------------------------- Private Datas --------------------------------
private:
  
  
  
  

    
  //!< Used to represent all the list of voxel primitive
  std::vector< std::vector<voxelD3D> > myVoxelSetList;
  
  //!< Used to represent all the list of line primitive
  std::vector< std::vector<lineD3D> > myLineSetList;
  
  //!< Used to represent all the list of point primitive
  std::vector< std::vector<pointD3D> > myPointSetList;

  //!< Represent all the clipping planes added to the scene (of maxSize=5)
  std::vector< clippingPlaneD3D > myClippingPlaneList;
  
  //!< For saving all surfels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< quadD3D > myKSSurfelList;

  //!< For saving all pointels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< pointD3D > myKSPointelList;

  //!< For saving all linels of Khalimsky space (used to display Khalimsky Space Cell)
  std::vector< lineD3D > myKSLinelList;

  //!< Represent all the drawed planes
  std::vector< quadD3D > myQuadList;
  
  //!< Used to define if GL_TEST_DEPTH is used. 
  std::vector<bool> myListVoxelDepthTest;
  
  /**
   * Precompute 4x4 projection matrix for 3D->2D projection.
   */
  void precompute_projection_matrix();
  
  /**
  * Project a 3d point (3D->2D).
  * @param x3d x position of the 3d point.
  * @param y3d y position of the 3d point.
  * @param z3d z position of the 3d point.
  * @param x2d x destination projection position of the 2d point.
  * @param y2d y destination projection position of the 2d point.
  */
  void project(double x3d, double y3d, double z3d, double &x2d, double &y2d);
  
  int Viewport[4];		//!< 2D viewport
  double matrix[16]; 		//!< projection matrix
      
  double camera_position[3];	//!< camera position
  double camera_direction[3];	//!< camera direction
  double camera_upVector[3];	//!< camera up-vector
  
  double ZNear;			//!< znear distance
  double ZFar;			//!< zfar distance
  
protected :
  /*!
   * \brief init function (should be in Constructor).
  */
  virtual void init();

private:

  }; // end of class Board3DTo2D
  
  /**
   * Cairo3dCameraPosition class to set camera position.
   */
  struct Cairo3dCameraPosition : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x position.
     * @param y y position.
     * @param z z position.
     */
    Cairo3dCameraPosition( const double x, const double y, const double z )
    {
      eyex=x; eyey=y; eyez=z;
    }
    
    void selfDrawDisplay3D( Display3D & display) const
    {
      display.setCameraPosition(eyex, eyey, eyez);
    }
    
    private:
      double eyex, eyey, eyez;
  };
  
  /**
   * Cairo3dCameraDirection class to set camera direction.
   */
  struct Cairo3dCameraDirection : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x direction.
     * @param y y direction.
     * @param z z direction.
     */
    Cairo3dCameraDirection( const double x, const double y, const double z )
    {
      dirx=x; diry=y; dirz=z;
    }
    
    virtual void selfDrawDisplay3D( Display3D & display) const
    {
      display.setCameraDirection(dirx, diry, dirz);
    }
    
    private:
      double dirx, diry, dirz;
  };
  
  /**
   * Cairo3dCameraUpVector class to set camera up-vector.
   */
  struct Cairo3dCameraUpVector : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x coordinate of up-vector.
     * @param y y coordinate of up-vector.
     * @param z z coordinate of up-vector.
     */
    Cairo3dCameraUpVector( const double x, const double y, const double z )
    {
      upx=x; upy=y; upz=z;
    }
    
    virtual void selfDrawDisplay3D( Display3D & viewer) const
    {
      viewer.setCameraUpVector(upx, upy, upz);
    }
    
    private:
      double upx, upy, upz;
  };
  
  /**
   * Cairo3dCameraZNearFar class to set near and far distance.
   */
  struct Cairo3dCameraZNearFar : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param near near distance.
     * @param far far distance.
     */
    Cairo3dCameraZNearFar( const double near, const double far )
    {
      ZNear=near; ZFar=far;
    }
    
    virtual void selfDrawDisplay3D( Display3D & viewer) const
    {
      viewer.setNearFar(ZNear, ZFar);
    }
    
    private:
      double ZNear, ZFar;
  };

 

  
/**
 * Overloads 'operator<<' for displaying objects of class 'Board3DTo2D'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Board3DTo2D' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const Board3DTo2D & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/boards/Board3DTo2D.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Board3DTo2D_h

#undef Board3DTo2D_RECURSES
#endif // else defined(Board3DTo2D_RECURSES)
