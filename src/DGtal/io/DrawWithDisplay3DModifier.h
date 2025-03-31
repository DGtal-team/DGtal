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
 * @author Aline Martin
 *
 * @date 2013/07/02
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
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Alias.h"
#include "DGtal/base/ConstAlias.h"
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
struct DrawWithDisplay3DModifier
{
  std::string className() const;
};



/**
   * @brief Modifier class in a Display3D stream. Useful to choose your
   * own mode for a given class. Realizes the concept
   * CDrawableWithDisplay3D.
   */

struct SetMode3D : public DrawWithDisplay3DModifier
{
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
struct CustomStyle3D : public DrawWithDisplay3DModifier
{
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
   * Custom style class redefining the fill color and the
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

struct ClippingPlane : public DrawWithDisplay3DModifier
{
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
   * @brief class to modify the position and scale to construct better illustration mode.
   * @todo add a constructor to automatically define the shift and the scale according a given associated SCell.
   */
struct TransformedPrism : public DrawWithDisplay3DModifier
{
  /**
     * Constructor.
     *
     * @param aSurfel a DGtal::Z3i::SCell ( KhalimskySpaceND< 3, Integer > SCell ) .
     * @param aShift the shift distance (positive or negative).
     * @param aSizeFactor use to change the Prism size (1.0 initial size).
     */
  TransformedPrism( const DGtal::Z3i::SCell  & aSurfel,
                          double aShift, double aSizeFactor=1.0 ):mySurfel(aSurfel), myShift(aShift), mySizeFactor(aSizeFactor)
  {
  }


  /**
     * Constructor.
     *
     * @param aSurfel a DGtal::Z3i::SCell ( KhalimskySpaceND< 3, Integer > SCell ) .
     * @param aVoxel a  DGtal::Z3i::SCell represent the voxel for which the surfel is associated. It permits to determine automatically the shift parameter (the surfel is automatically shifted towards this voxel).
     * @param aShift the shift distance (positive or negative (default 0.05)).
     * @param aSizeFactor use to change the Prism size (default 0.75).
     */
  TransformedPrism( const DGtal::Z3i::SCell  & aSurfel,
                          const DGtal::Z3i::SCell  & aVoxel,
                          double aShift=0.05, double aSizeFactor=0.75  )
    : mySurfel(aSurfel), myShift(aShift), mySizeFactor(aSizeFactor)
  {
    auto const& preSurfel = aSurfel.preCell();
    auto const& preVoxel  = aVoxel.preCell();

    bool xodd = (preSurfel.coordinates[ 0 ] & 1 );
    bool yodd = (preSurfel.coordinates[ 1 ] & 1 );
    bool zodd = (preSurfel.coordinates[ 2 ] & 1 );
    if(!xodd )
    {
      myShift*= ((preVoxel.coordinates[ 0 ]-preSurfel.coordinates[ 0 ] <0)? -1.0: 1.0);
    }else if(!yodd )
    {
      myShift*=((preVoxel.coordinates[ 1 ]-preSurfel.coordinates[ 1 ] <0)? -1.0: 1.0);
    }else if(!zodd )
    {
      myShift*=((preVoxel.coordinates[ 2 ]-preSurfel.coordinates[ 2 ] <0)? -1.0: 1.0);
    }
  }

  ///@todo FIX this member
  DGtal::Z3i::SCell mySurfel;
  double myShift;
  double mySizeFactor;
};


  /**
   * This structure is used to set the "OpenGL name" (an integer
   * identifier) of the following display command(s).
   * You may use it like
   * \code
   * SCell surfel = ... ;
   * viewer << SetName3D( 100 );
   * viewer << surfel; // surfel is identified with name 100 when clicked.
   * \endcode
   */
  struct SetName3D : public DrawWithDisplay3DModifier {
    /// Sets the "OpenGL name" of future display command(s).
    /// @param aName any integer: an identifier for later selection or -1 for none.
    /// @param sName any string: a string identifier for the later selection or -1 to use a default name. 
    SetName3D( DGtal::int32_t aName = -1, const std::string& sName = "") : 
      name( aName ), strName(sName) 
    {}
    /// @return the class name as a string.
    std::string className() const { return "SetName3D"; }
    /// the "OpenGL name" for selection, or -1 for none.
    DGtal::int32_t name;
    std::string strName;
  };

  /**
   * This structure is used to pass callback functions to the
   * viewer. These callback functions are called when specific
   * graphical objects are selected by the user (@see SetName3D).
   */
  struct SetSelectCallback3D : public DrawWithDisplay3DModifier {
    /// The prototype for a callback function. It is called with a
    /// pointer to the viewer, the "OpenGL name" of the selected
    /// graphical element and a pointer toward the data that was given
    /// at construction of SetSelectCallback3D.
    typedef int (*CallbackFct)( void* viewer, DGtal::int32_t name, void* data );
    SetSelectCallback3D( CallbackFct f,
                         void* data,
                         DGtal::int32_t min = 0, DGtal::int32_t max = 0x7fffffff )
      : myFct( f ), myData( data ), myMin( min ), myMax( max ) {}
    /// @return the class name as a string.
    std::string className() const { return "SetSelectCallback3D"; }
    /// The callback function associated to the selection of an element.
    CallbackFct myFct;
    void* myData;
    DGtal::int32_t myMin;
    DGtal::int32_t myMax;
  };

  enum ImageDirection { xDirection, yDirection, zDirection, undefDirection };
  enum TextureMode {RGBMode, GrayScaleMode }; 
    
  /**
   *
   * @brief class to modify the position and orientation of an textured 2D image.
   *
   */
  template < typename Space, typename KSpace>
  struct UpdateImagePosition : public DrawWithDisplay3DModifier 
  {
    std::string className() const { return "UpdateImagePosition"; }
    /**
     * Constructor given from an specific image index, a new direction
     * (associated to the normal of the image plane), and and a new
     * position of the bottom-left point.
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Viewer3D).
     * @param newDir give the new direction of the image normal vector.
     * @param posXbottomLeft the x position of the bottom left point.
     * @param posYbottomLeft the y position of the bottom left point.
     * @param posZbottomLeft the z position of the bottom left point.
     *
     */
    UpdateImagePosition(unsigned int anIndex, ImageDirection newDir,
                        double posXbottomLeft, double posYbottomLeft, double posZbottomLeft ):  myIndex(anIndex),
                                                                                                myPosXBottomLeft(posXbottomLeft),
                                                                                                myPosYBottomLeft(posYbottomLeft),
                                                                                                myPosZBottomLeft(posZbottomLeft),
                                                                                                myNewDirection(newDir)
    {}

    unsigned int myIndex;
    double  myPosXBottomLeft;
    double  myPosYBottomLeft;
    double  myPosZBottomLeft;
    ImageDirection myNewDirection;
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
   * viewer3D-8-2DSliceImages.cpp ):
   * We can first add a functor to convert grayscale value into RGB int:
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DImagesExtractImagesColorHeader
   *
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DImagesExtractImagesColorFct
   *
   * Then you can define and add the object AddTextureImage2DWithFunctor in a viewer:
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DImagesDisplayImagesColor
   *
   * @note If you change the image date don't forget to specify again.
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DModifImagesColor
   *
   * @see AddTextureImage3DWithFunctor viewer3D-8-2DSliceImages.cpp viewer3D-9-3Dimages.cpp
   */
  template <typename TImageType, typename TFunctor, typename Space, typename KSpace>
  struct AddTextureImage2DWithFunctor : public DrawWithDisplay3DModifier
  {
    std::string className() const { return "AddTextureImage2DWithFunctor"; }
    BOOST_CONCEPT_ASSERT((  concepts::CConstImage<TImageType> )) ;
    
    /**
     * Constructor given from an 2D image and a Functor to apply specific conversion.
     *
     */
    AddTextureImage2DWithFunctor(ConstAlias<TImageType> anImage,
                                 Clone<TFunctor> aFunctor,
                                 TextureMode aMode = TextureMode::GrayScaleMode): my2DImage(&anImage),
                                                                                    myFunctor(aFunctor),
                                                                                    myMode(aMode)
    {

    }
    const TImageType *my2DImage;
    const TFunctor myFunctor;
    TextureMode myMode;
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
   * viewer3D-8-2DSliceImages.cpp viewer3D-9-3Dimages.cpp):
   * We can first add a functor to convert grayscale value into RGB int:
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DImagesExtractImagesColorHeader
   *
   * @snippet io/viewers/viewer3D-8-2DSliceImages.cpp ExampleViewer3D2DImagesExtractImagesColorFct
   *
   * Then you can define and add the object AddTextureImage2DWithFunctor in a viewer:
   * @snippet io/viewers/viewer3D-9-3Dimages.cpp ExampleViewer3D3DImagesDisplayImagesColor
   *
   * @note If you change the image date don't forget to specify again the functor with the UpdateImageData object.
   *
   * @see AddTextureImage2DWithFunctor viewer3D-8-2DSliceImages.cpp viewer3D-9-3Dimages.cpp
   */
  template <typename TImageType, typename TFunctor, typename Space, typename KSpace>
  struct AddTextureImage3DWithFunctor : public DrawWithDisplay3DModifier
  {
    std::string className() const { return "AddTExtureImage3DWithFunctor"; }
    BOOST_CONCEPT_ASSERT((  concepts::CConstImage<TImageType> )) ;
    /**
     * Constructor given from an 2D image and a Functor to apply specific conversion.
     *
     */
    AddTextureImage3DWithFunctor(ConstAlias<TImageType> anImage,
                                 Clone<TFunctor> aFunctor,
                                 TextureMode aMode =
                                 TextureMode::GrayScaleMode): my3DImage(&anImage),
                                                                         myFunctor(aFunctor),
                                                                         myMode(aMode)
    {

    }
    const TImageType *my3DImage;
    const TFunctor myFunctor;
    TextureMode myMode;
  };



  /**
   *
  * @brief class to modify the position and orientation of an textured 2D image.
   *
   */
  template < typename Space, typename KSpace>
  struct UpdateLastImagePosition : public DrawWithDisplay3DModifier 
  {
    std::string className() const { return "UpdateLastImagePosition"; }
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
    UpdateLastImagePosition( ImageDirection newDir,
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
    ImageDirection myNewDirection;
  };




  /**
   * @brief class to modify the data of an given image and also the
   * possibility to translate it (optional).
   *
   */
  template<typename TImageType, typename TFunctor= functors::Cast<unsigned int> >
  struct UpdateImageData : public DrawWithDisplay3DModifier 
  {
    std::string className() const { return "UpdateImageData"; }
    /**
     * Constructor given from an specific image index, a new image
     * (should be of dimension 2 and with the same size than the
     * orginal), and a possible (optional translation).
     *
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Viewer3D).
     * @param anImage the new image which will be used to update the source image  data.
     * @param translateX the x translation value.
     * @param translateY the y translation value.
     * @param translateZ the y translation value.
     * @param rotationAngle the angle of rotation. 
     * @param dirRotation the rotation is applyed arount the given direction (default zDirection).
     * @param aFunctor a functor.
     */
    UpdateImageData(unsigned int anIndex, ConstAlias<TImageType> anImage, double translateX=0,
                    double translateY=0, double translateZ=0,
		    double rotationAngle=0.0, ImageDirection dirRotation=zDirection,
		    Clone<TFunctor> aFunctor = TFunctor() ): myIndex(anIndex),
							   myImage(&anImage),
							   myTranslateX (translateX),
							   myTranslateY (translateY),
							   myTranslateZ (translateZ),
							   myFunctor(aFunctor),
							   myRotationAngle(rotationAngle),
							   myRotationDir(dirRotation)
    {}

    unsigned int myIndex;
    const TImageType *myImage;
    int myTranslateX;
    int myTranslateY;
    int myTranslateZ;
    const TFunctor myFunctor;
    double myRotationAngle;
    ImageDirection  myRotationDir;
   };

  /**
   *
   * @brief class to modify the 3d embedding  of the image (useful to display not only 2D slice images).
   * The embdding can be explicitly given from the 3D position of the four bounding points. 
   */
  template < typename Space, typename KSpace>
  struct UpdateImage3DEmbedding : public DrawWithDisplay3DModifier 
  {
    std::string className() const { return "UpdateImage3DEmbedding"; }
    /**
     * Constructor given from the four embedded 3D points.
     * The first (resp. third) point correspondts to the lower (res. upper) point according the 3 directions and the order should be given CCW. 
     *
     * @param anIndex the index of the image to be modified (should be less than the number of image added in the current Viewer3D).
     * @param aPoint1 the new first point position embedded in 3D associated the lower point of the 2D image.  
     * @param aPoint2 the new second point position embedded in 3D (in CCW order).
     * @param aPoint3 the new third point position embedded in 3D  associated the upper point of the 2D image.  
     * @param aPoint4 the new fourth point position  embedded in 3D (in CCW order).
     *
     **/
    UpdateImage3DEmbedding(unsigned int anIndex, 
                           typename Space::Point aPoint1, typename Space::Point aPoint2,
                           typename Space::Point aPoint3, typename Space::Point aPoint4): myIndex(anIndex),
                                                                                          myNewPoint1(aPoint1),
                                                                                          myNewPoint2(aPoint2),
                                                                                          myNewPoint3(aPoint3),
                                                                                          myNewPoint4(aPoint4)
    {
    }
    unsigned int myIndex;
    typename Space::Point myNewPoint1;
    typename Space::Point myNewPoint2;
    typename Space::Point myNewPoint3;
    typename Space::Point myNewPoint4;
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
