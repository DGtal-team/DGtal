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
#include "DGtal/base/Alias.h"
#include "DGtal/images/CConstImage.h"
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
    std::string className() const;
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
     * @param aClassname the name of the class to which the style is associated.
     *
     * @param style a pointer on a dynamically allocated style, which
     * is acquired by the class.
     */
    CustomStyle3D( std::string aClassname, DrawableWithDisplay3D* style ): myClassname( aClassname ), myStyle( style )
    {}
    
    std::string className() const;
    
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
    
    double * getEquation();
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
    CameraPosition( const double x, const double y, const double z ):eyex(x), eyey(y), eyez(z)
    {
    }
    
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
    CameraDirection( const double x, const double y, const double z ): dirx(x), diry(y), dirz(z)
    {
    }
    
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
    CameraUpVector( const double x, const double y, const double z ): upx(x), upy(y), upz(z)
    {
      upx=x; upy=y; upz=z;
    }
    
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
     * @param near near distance.
     * @param far far distance.
     */
    CameraZNearFar( const double near, const double far ): ZNear(near), ZFar(far)
    {
    }
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
                        double aShift, double aSizeFactor=1.0 ):mySurfel(aSurfel), myShift(aShift), mySizeFactor(aSizeFactor)
    {
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
  
  
  
  
  
  
  /**
   *
   * @brief class to modify the position and orientation of an textured 2D image.
   *
   */
  struct UpdateImagePosition : public DrawWithDisplay3DModifier
  {
    
    /**
     * Constructor given from an specific image index, a new direction
     * (associated to the normal of the image plane), and and a new
     * position of the bottom-left point.
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Display3D).
     * @param newDir give the new direction of the image normal vector.
     * @param posXbottomLeft the x position of the bottom left point.
     * @param posYbottomLeft the y position of the bottom left point.
     * @param posZbottomLeft the z position of the bottom left point.
     *
     */
    UpdateImagePosition(unsigned int anIndex, Display3D::ImageDirection newDir,
                        double posXbottomLeft, double posYbottomLeft, double posZbottomLeft ):  myIndex(anIndex),
    myPosXBottomLeft(posXbottomLeft),
    myPosYBottomLeft(posYbottomLeft),
    myPosZBottomLeft(posZbottomLeft),
    myNewDirection(newDir)
    
    {
      
    }
    unsigned int myIndex;
    double  myPosXBottomLeft;
    double  myPosYBottomLeft;
    double  myPosZBottomLeft;
    Display3D::ImageDirection myNewDirection;
  };
  
  
  
  /**
   *
   * @brief class to insert a custom 2D textured image by using a
   * conversion functor and allows to change the default mode
   * (GrayScale mode) to color mode.
   *
   * @tparam TImageType the type of the used as texture (should follow the concept of CConstImage).
   * @tparam TFunctor  the functor type to transform source image scalar value into the one of the image being displayed.
   *
   * A typical use can be illustrated by displaying a grayscale source
   * image with artificial color defined from a colormap (see
   * viewer3D-8-2Dimages.cpp ):
   * We can first add a functor to convert grayscale value into RGB int:
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DImagesExtractImagesColorHeader
   *
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DImagesExtractImagesColorFct
   *
   * Then you can define and add the object AddTextureImage2DWithFunctor in a viewer:
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DImagesDisplayImagesColor
   *
   * @note If you change the image date don't forget to specify again.
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DModifImagesColor
   *
   * @see AddTextureImage3DWithFunctor viewer3D-8-2Dimages.cpp viewer3D-9-3Dimages.cpp
   */
  template <typename TImageType, typename TFunctor>
  struct AddTextureImage2DWithFunctor : public DrawWithDisplay3DModifier
  {
    BOOST_CONCEPT_ASSERT((  CConstImage<TImageType> )) ;
    
    /**
     * Constructor given from an 2D image and a Functor to apply specific conversion.
     *
     */
    
    AddTextureImage2DWithFunctor(ConstAlias<TImageType> anImage,
                                 ConstAlias<TFunctor> aFunctor,
                                 Display3D::TextureMode aMode=Display3D::GrayScaleMode ): my2DImage(anImage),
    myFunctor(aFunctor),
    myMode(aMode)
    {
      
    }
    const TImageType *my2DImage;
    const TFunctor &myFunctor;
    Display3D::TextureMode myMode;
  };
  
  /**
   *
   * @brief class to insert a custom 3D textured image by using a
   * conversion functor and allows to change the default mode
   * (GrayScale mode) to color mode.
   *
   * @tparam TImageType the type of the used as texture (should follow the concept of CConstImage).
   * @tparam TFunctor  the functor type to transform source image scalar value into the one of the image being displayed.
   *
   * A typical use can be illustrated by displaying a grayscale source
   * image with artificial color defined from a colormap (see
   * viewer3D-8-2Dimages.cpp viewer3D-9-3Dimages.cpp):
   * We can first add a functor to convert grayscale value into RGB int:
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DImagesExtractImagesColorHeader
   *
   * @snippet io/viewers/viewer3D-8-2Dimages.cpp ExampleViewer3D2DImagesExtractImagesColorFct
   *
   * Then you can define and add the object AddTextureImage2DWithFunctor in a viewer:
   * @snippet io/viewers/viewer3D-9-3Dimages.cpp ExampleViewer3D3DImagesDisplayImagesColor
   *
   * @note If you change the image date don't forget to specify again the functor with the UpdateImageData object.
   *
   * @see AddTextureImage2DWithFunctor viewer3D-8-2Dimages.cpp viewer3D-9-3Dimages.cpp
   */
  template <typename TImageType, typename TFunctor>
  struct AddTextureImage3DWithFunctor : public DrawWithDisplay3DModifier
  {
    BOOST_CONCEPT_ASSERT((  CConstImage<TImageType> )) ;
    
    /**
     * Constructor given from an 2D image and a Functor to apply specific conversion.
     *
     */
    
    AddTextureImage3DWithFunctor(ConstAlias<TImageType> anImage,
                                 ConstAlias<TFunctor> aFunctor,
                                 Display3D::TextureMode aMode=Display3D::GrayScaleMode): my3DImage(anImage),
    myFunctor(aFunctor),
    myMode(aMode)
    {
      
    }
    const TImageType *my3DImage;
    const TFunctor &myFunctor;
    Display3D::TextureMode myMode;
  };
  
  
  
  /**
   *
   * @brief class to modify the position and orientation of an textured 2D image.
   *
   */
  struct UpdateLastImagePosition : public DrawWithDisplay3DModifier
  {
    
    /**
     * Constructor given from an specific image index, a new direction
     * (associated to the normal of the image plane), and and a new
     * position of the bottom-left point.
     * @param newDir give the new direction of the image normal vector.
     * @param posXbottomLeft the x position of the bottom left point.
     * @param posYbottomLeft the y position of the bottom left point.
     * @param posZbottomLeft the z position of the bottom left point.
     *
     */
    UpdateLastImagePosition( Display3D::ImageDirection newDir,
                            double posXbottomLeft, double posYbottomLeft, double posZbottomLeft ):
    myPosXBottomLeft(posXbottomLeft),
    myPosYBottomLeft(posYbottomLeft),
    myPosZBottomLeft(posZbottomLeft),
    myNewDirection(newDir)
  {
      
    }
    double  myPosXBottomLeft;
    double  myPosYBottomLeft;
    double  myPosZBottomLeft;
    Display3D::ImageDirection myNewDirection;
  };
  
  
  
  
  /**
   * @brief class to modify the data of an given image and also the
   * possibility to translate it (optional).
   *
   */
  template<typename TImageType, typename TFunctor= CastFunctor<unsigned int> >
  struct UpdateImageData : public DrawWithDisplay3DModifier
  {
    
    /**
     * Constructor given from an specific image index, a new image
     * (should be of dimension 2 and with the same size than the
     * orginal), and a possible (optional translation).
     *
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Display3D).
     * @param anImage the new image which will be used to update the source image  data.
     * @param translateX the x translation value.
     * @param translateY the y translation value.
     * @param translateZ the y translation value.
     *
     */
    UpdateImageData(unsigned int anIndex, const  TImageType &anImage, double translateX=0,
                    double translateY=0, double translateZ=0, const TFunctor &aFunctor=TFunctor() ): myIndex(anIndex),
    myImage(&anImage),
    myTranslateX (translateX),
    myTranslateY (translateY),
    myTranslateZ (translateZ),
    myFunctor(aFunctor)
    {
      
    }
    unsigned int myIndex;
    int myTranslateX;
    int myTranslateY;
    int myTranslateZ;
    const TImageType *myImage;
    const TFunctor &myFunctor;
  };
  
  
  
  
  /**
   * @brief class to modify the data of an given image and also the
   * possibility to translate it (optional).
   *
   */
  struct Translate2DDomain : public DrawWithDisplay3DModifier
  {
    
    /**
     * Constructor given from an specific image index, a new image
     * (should be of dimension 2 and with the same size than the
     * orginal), and a possible (optional translation).
     *
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Display3D).
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
    {
      
    }
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
  struct Update2DDomainPosition : public DrawWithDisplay3DModifier
  {
    
    /**
     * Constructor given from an specific 2D domain index, a new direction
     * (associated to the normal of the 2D domain plane), and and a new
     * position of the bottom-left point.
     * @param anIndex the index of the 2D domain to be modified (should be less than the number of domain added in the current Display3D).
     * @param newDir give the new direction of the domain normal vector.
     * @param posXbottomLeft the x position of the bottom left point.
     * @param posYbottomLeft the y position of the bottom left point.
     * @param posZbottomLeft the z position of the bottom left point.
     *
     */
    Update2DDomainPosition(unsigned int anIndex, Display3D::ImageDirection newDir,
                           double posXbottomLeft, double posYbottomLeft, double posZbottomLeft ):  myIndex(anIndex),
    myPosXBottomLeft(posXbottomLeft),
    myPosYBottomLeft(posYbottomLeft),
    myPosZBottomLeft(posZbottomLeft),
    myNewDirection(newDir)
    
    {
      
    }
    unsigned int myIndex;
    double  myPosXBottomLeft;
    double  myPosYBottomLeft;
    double  myPosZBottomLeft;
    Display3D::ImageDirection myNewDirection;
  };
  
  
} // namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/DrawWithDisplay3DModifier.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DrawWithDisplay3DModifier_h

#undef DrawWithDisplay3DModifier_RECURSES
#endif // else defined(DrawWithDisplay3DModifier_RECURSES)
