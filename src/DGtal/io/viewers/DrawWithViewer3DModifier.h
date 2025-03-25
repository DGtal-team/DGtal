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
 * @file DrawWithViewer3DModifier.h
 * @author Aline Martin
 *
 * @date 2013/07/02
 *
 * Header file for module DrawWithViewer3DModifier.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DrawWithViewer3DModifier_RECURSES)
#error Recursive header files inclusion detected in DrawWithViewer3DModifier.h
#else // defined(DrawWithViewer3DModifier_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DrawWithViewer3DModifier_RECURSES

#if !defined DrawWithViewer3DModifier_h
/** Prevents repeated inclusion of headers. */
#define DrawWithViewer3DModifier_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Alias.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
   *@brief Base class specifying the methods for classes which intend to
   * modify a Viewer3D stream.
   *
   */
  struct DrawWithViewer3DModifier : public DrawWithDisplay3DModifier
  {
    std::string className() const;
  };


#ifndef DrawWithBoard3DTo2DModifier_h
  /**
   * @brief  CameraPosition class to set camera position.
   */
  struct CameraPosition : public DrawWithViewer3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x position.
     * @param y y position.
     * @param z z position.
     */
    CameraPosition( const double x, const double y, const double z ):eyex(x), eyey(y), eyez(z)
    {
    }

    double eyex, eyey, eyez;
  };


  /**
   * @brief CameraDirection class to set camera direction.
   */
  struct CameraDirection : public DrawWithViewer3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x direction.
     * @param y y direction.
     * @param z z direction.
     */
    CameraDirection( const double x, const double y, const double z ): dirx(x), diry(y), dirz(z)
    {
    }

    double dirx, diry, dirz;
  };


  /**
   * @brief CameraUpVector class to set camera up-vector.
   */
  struct CameraUpVector : public DrawWithViewer3DModifier
  {
    /**
     * Constructor.
     *
     * @param x x coordinate of up-vector.
     * @param y y coordinate of up-vector.
     * @param z z coordinate of up-vector.
     */
    CameraUpVector( const double x, const double y, const double z ): upx(x), upy(y), upz(z)
    {
      upx=x; upy=y; upz=z;
    }

    double upx, upy, upz;
  };



  /**
   * @brief CameraZNearFar class to set near and far distance.
   */
  struct CameraZNearFar : public DrawWithViewer3DModifier
  {
    /**
     * Constructor.
     *
     * @param near near distance.
     * @param far far distance.
     */
    CameraZNearFar( const double near, const double far ): ZNear(near), ZFar(far)
    {
    }
    double ZNear, ZFar;
  };

#endif

  /**
   * @brief class to modify the data of an given image and also the
   * possibility to translate it (optional).
   *
   */
  struct Translate2DDomain : public DrawWithViewer3DModifier
  {

    /**
     * Constructor given from an specific image index, a new image
     * (should be of dimension 2 and with the same size than the
     * orginal), and a possible (optional translation).
     *
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Viewer3D).
     * @param translateX the x translation value.
     * @param translateY the y translation value.
     * @param translateZ the y translation value.
     *
     */
    Translate2DDomain(unsigned int anIndex, double translateX=0,
                      double translateY=0, double translateZ=0 ): myIndex(anIndex),
                                                                  myTranslateX (translateX),
                                                                  myTranslateY (translateY),
                                                                  myTranslateZ (translateZ)
    { }

    unsigned int myIndex;
    int myTranslateX;
    int myTranslateY;
    int myTranslateZ;
  };






  /**
   *
   * @brief class to modify the position and orientation of an 2D domain.
   *
   */
  template < typename Space, typename KSpace>
  struct Update2DDomainPosition : public DrawWithViewer3DModifier
  {

    /**
     * Constructor given from an specific 2D domain index, a new direction
     * (associated to the normal of the 2D domain plane), and and a new
     * position of the bottom-left point.
     * @param anIndex the index of the 2D domain to be modified (should be less than the number of domain added in the current Viewer3D).
     * @param newDir give the new direction of the domain normal vector.
     * @param posXbottomLeft the x position of the bottom left point.
     * @param posYbottomLeft the y position of the bottom left point.
     * @param posZbottomLeft the z position of the bottom left point.
     *
     */
    Update2DDomainPosition(unsigned int anIndex, ImageDirection newDir,
                           double posXbottomLeft, double posYbottomLeft, double posZbottomLeft ):  myIndex(anIndex),
                                                                                                   myPosXBottomLeft(posXbottomLeft),
                                                                                                   myPosYBottomLeft(posYbottomLeft),
                                                                                                   myPosZBottomLeft(posZbottomLeft),
      myNewDirection(newDir)
    { }

    unsigned int myIndex;
    double  myPosXBottomLeft;
    double  myPosYBottomLeft;
    double  myPosZBottomLeft;
    ImageDirection myNewDirection;
  };




} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DrawWithViewer3DModifier_h

#undef DrawWithViewer3DModifier_RECURSES
#endif // else defined(DrawWithViewer3DModifier_RECURSES)
