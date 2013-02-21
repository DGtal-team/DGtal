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
 * @file DrawWithDisplay3DModifier.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/08/08
 *
 * Header file for module DrawWithDisplay3DModifier.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DrawWithDisplay3DModifier_RECURSES)
#error Recursive header files inclusion detected in DrawWithDisplay3DModifier.h
#else // defined(DrawWithDisplay3DModifier_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DrawWithDisplay3DModifier_RECURSES

#if !defined DrawWithDisplay3DModifier_h
/** Prevents repeated inclusion of headers. */
#define DrawWithDisplay3DModifier_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/topology/KhalimskySpaceND.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

 /**
   *@brief Base class specifying the methods for classes which intend to
   * modify a Viewer3D stream.
   * 
   */
  struct DrawWithDisplay3DModifier {
    std::string className() const
    {
      return "DrawWithDisplay3DModifier";
    }

    /*DrawableWithDisplay3D* defaultStyleDisplay3D( std::string = "" ) const
    {
      return 0;
    }*/

    /*virtual void setStyleDisplay3D( Display3D &  ) const 
    {}*/
  };






  /**
   * @brief Modifier class in a Display3D stream. Useful to choose your
   * own mode for a given class. Realizes the concept
   * CDrawableWithDisplay3D.
   */
  
  struct SetMode3D : public DrawWithDisplay3DModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param mode the name of the new mode.
     */
    SetMode3D( std::string classname, std::string mode )
      : myClassname( classname ), myMode( mode )
    {}
    
    /*void setStyleDisplay3D( Display3D & display ) const
    {
      display.myModes[ myClassname ] = myMode;
    }*/
    
    std::string myClassname;
    std::string myMode;
  };




  /**
   * @brief Modifier class in a Display3D stream. Useful to choose your own
   * style for a given class. Realizes the concept
   * CDrawableWithDisplay3D.
   */
  struct CustomStyle3D : public DrawWithDisplay3DModifier {
    /**
     * @param classname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    CustomStyle3D( std::string classname, DrawableWithDisplay3D* style )
      : myClassname( classname ), myStyle( style )
    {}

    std::string className() const
    {
      return "CustomStyle3D";
    }

    /*void setStyleDisplay3D( Display3D & display ) const
    {
      display.myStyles[ myClassname ] = myStyle;
    }*/

    std::string myClassname;
    CountedPtr<DrawableWithDisplay3D> myStyle;
  };



  
  /**
   * @brief Custom style class redefining the fill color and the
   * gl_LINE/gl_POINT color. You can use DGtal::Color with alpha
   * transparency value but you nedd to take into account the z-buffer
   * during the Open-GL based rendering.
   *
   \code
   Display3D display;
   display << CustomColors3D(Color(250, 0,0),Color(250, 0,0));
   ...
   \endcode
   * @see Display3D
   */
  struct CustomColors3D : public DrawWithDisplay3DModifier
  {
    Color myPenColor;
    Color myFillColor;

    /**
     * Constructor.
     *
     * @param penColor specifies the pen color.
     * @param fillColor specifies the fill color.
     */
    CustomColors3D( const Color & penColor,
        const Color & fillColor )
      : myPenColor( penColor ), myFillColor( fillColor )
    {}
    
    /*virtual void setStyleDisplay3D( Display3D & display) const
    {
      display.setFillColor(myFillColor);
      display.setLineColor(myPenColor);
    }*/
  };



  
 /**
  * @brief Class for adding a Clipping plane through the Viewer3D
  * stream. Realizes the concept CDrawableWithViewer3D.
  */
  
  struct ClippingPlane : public DrawWithDisplay3DModifier {
    /**
     *
     */
    ClippingPlane( double a, double b, double c, double d, bool drawPlane=true )
      : myA( a ), myB( b ), myC( c ), myD ( d ), myDrawPlane(drawPlane)  
    {}
    
    // MT: why setStyleViewer3D here and not setStyleDisplay3D ???
    /*void setStyleViewer3D( Display3D & display ) const
    {
      display.addClippingPlane(myA, myB, myC, myD, myDrawPlane);
    }*/
    
    double * getEquation(){
      double *r = new double[4];
      r[0] = myA;
      r[1] = myB;
      r[2] = myC;
      r[3] = myD;
      return r;
    } 
    
    double myA;
    double myB;
    double myC;
    double myD;
    bool myDrawPlane;
  };


  /**
   * @brief  CameraPosition class to set camera position.
   */
  struct CameraPosition : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x position.
     * @param y y position.
     * @param z z position.
     */
    CameraPosition( const double x, const double y, const double z )
    {
      eyex=x; eyey=y; eyez=z;
    }
    
    /*void setStyleDisplay3D( Display3D & display) const
    {
      display.setCameraPosition(eyex, eyey, eyez);
    }*/
    
      double eyex, eyey, eyez;
  };
  
  /**
   * @brief CameraDirection class to set camera direction.
   */
  struct CameraDirection : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x direction.
     * @param y y direction.
     * @param z z direction.
     */
    CameraDirection( const double x, const double y, const double z )
    {
      dirx=x; diry=y; dirz=z;
    }
    
    /*virtual void setStyleDisplay3D( Display3D & display) const
    {
      display.setCameraDirection(dirx, diry, dirz);
    }*/
    
      double dirx, diry, dirz;
  };
  
  /**
   * @brief CameraUpVector class to set camera up-vector.
   */
  struct CameraUpVector : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x coordinate of up-vector.
     * @param y y coordinate of up-vector.
     * @param z z coordinate of up-vector.
     */
    CameraUpVector( const double x, const double y, const double z )
    {
      upx=x; upy=y; upz=z;
    }
    
    /*virtual void setStyleDisplay3D( Display3D & viewer) const
    {
      viewer.setCameraUpVector(upx, upy, upz);
    }*/
    
      double upx, upy, upz;
  };
  
  /**
   * @brief CameraZNearFar class to set near and far distance.
   */
  struct CameraZNearFar : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param _near near distance.
     * @param _far far distance.
     */
    CameraZNearFar( const double _near, const double _far )
    {
      ZNear=_near; ZFar=_far;
    }
    
    /*virtual void setStyleDisplay3D( Display3D & viewer) const
    {
      viewer.setNearFar(ZNear, ZFar);
    }*/
    
      double ZNear, ZFar;
  };

 

 /**
   * @brief class to modify the position and scale to construct better illustration mode.
   * @todo add a constructor to automatically define the shift and the scale according a given associated SCell.
   */
  struct TransformedKSSurfel : public DrawWithDisplay3DModifier
  {
    /**
     * Constructor.
     *
     * @param aSurfel a DGtal::Z3i::SCell ( KhalimskySpaceND< 2, Integer > SCell ) .
     * @param aShift the shift distance (positive or negative).
     * @param aSizeFactor use to change the KSSurfel size (1.0 initial size).
     */
    TransformedKSSurfel( const DGtal::KhalimskySpaceND< 3, int >::SCell  & aSurfel, 
                         double aShift, double aSizeFactor=1.0 )
    {
      mySurfel= aSurfel;
      myShift = aShift;
      mySizeFactor=aSizeFactor;
    }

    /**
     * Constructor.
     *
     * @param aSurfel a DGtal::Z3i::SCell ( KhalimskySpaceND< 2, Integer > SCell ) .
     * @param aVoxel a  DGtal::Z3i::SCell represent the voxel for which the surfel is associated. It permits to determine automatically the shift parameter (the surfel is automatically shifted towards this voxel).
     * @param aShift the shift distance (positive or negative (default 0.05)).
     * @param aSizeFactor use to change the KSSurfel size (default 0.75).
     */
    TransformedKSSurfel( const DGtal::KhalimskySpaceND< 3, int >::SCell  & aSurfel, 
                         const DGtal::KhalimskySpaceND< 3, int >::SCell  & aVoxel, 
			 double aShift=0.05, double aSizeFactor=0.75  )
    {      
      mySurfel= aSurfel;
      myShift = aShift;
      mySizeFactor = aSizeFactor;
      bool xodd = (mySurfel.myCoordinates[ 0 ] & 1 );
      bool yodd = (mySurfel.myCoordinates[ 1 ] & 1 );
      bool zodd = (mySurfel.myCoordinates[ 2 ] & 1 );
      if(!xodd ){
	myShift*= ((aVoxel.myCoordinates[ 0 ]-mySurfel.myCoordinates[ 0 ] <0)? -1.0: 1.0);
      }else if(!yodd ){
	myShift*=((aVoxel.myCoordinates[ 1 ]-mySurfel.myCoordinates[ 1 ] <0)? -1.0: 1.0);
      }else if(!zodd ){
	myShift*=((aVoxel.myCoordinates[ 2 ]-mySurfel.myCoordinates[ 2 ] <0)? -1.0: 1.0);
      }
    }
    
    ///@todo FIX this member
    DGtal::KhalimskySpaceND< 3, int >::SCell mySurfel;
    double myShift;
    double mySizeFactor;
  };

 



} // namespace DGtal




//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DrawWithDisplay3DModifier_h

#undef DrawWithDisplay3DModifier_RECURSES
#endif // else defined(DrawWithDisplay3DModifier_RECURSES)
