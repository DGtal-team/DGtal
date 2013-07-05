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
 * @file Display3D.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/08/08
 *
 * Header file for module Display3D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Display3D_RECURSES)
#error Recursive header files inclusion detected in Display3D.h
#else // defined(Display3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Display3D_RECURSES

#if !defined Display3D_h
/** Prevents repeated inclusion of headers. */
#define Display3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/io/Color.h"
#include "DGtal/images/CImage.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/shapes/Mesh.h"

/// for embedding
#include "DGtal/topology/CanonicCellEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/kernel/CanonicEmbedder.h"
#include "DGtal/helpers/StdDefs.h"


//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{
//TODO up

/////////////////////////////////////////////////////////////////////////////
// class Display3D
/**
   * Description of class 'Display3D' <p>
   * \brief Aim:   This semi abstract class  defines the stream mechanism to
   display 3d primitive (like BallVector, DigitalSetBySTLSet, Object
   ...). The class Viewer3D and Board3DTo2D implement two different
   ways to display 3D objects. The first one (Viewer3D), permits an
   interactive visualisation (based on @a  OpenGL ) and the second one
   (Board3DTo2D) provides 3D visualisation from 2D vectorial display
   (based on the CAIRO library)

   @see Viewer3D, Board3DTo2D

  */
template < class S, class KS>
class Display3D
{


    // ------------------------- Private Datas --------------------------------
private:


protected:
    // Used to ajust the visualisation for data where scale are not constant in the three axis.
    // Uses in Viewer3D class and on export method.
    float myScaleX;
    float myScaleY;
    float myScaleZ;

    /**
     * Structure used to display line in 3D
     */
    struct lineD3D{
        double x1, y1, z1;
        double x2, y2, z2;
        double width;
        unsigned char R,G,B,T;
        bool isSigned;
        bool signPos;
    };

    /**
     * Defines the 3D cube.
     */
    struct cubeD3D{
        ///  The center coordinate of the cube.
        ///
        int x, y,z;

        ///  The display color of the cube.
        ///
        unsigned char R,G,B,T;

        /// The width of a cube face
        ///
        double width;
    };


    /**
     * Used to define clipping planes (it uses the quadD3D structure)
     * @see Display3D, Viewer3D, Board3DTo2D, quadD3D
     **/
    struct clippingPlaneD3D{
        double a,b,c,d;
    };


    /**
     * This structure is used to display clipping planes and the
     * components of the mySurfelPrismList (allowing to set normal and
     * color).
     * @see Display3D, Viewer3D, Board3DTo2D
     **/
    struct  quadD3D{
        double x1,y1,z1;
        double x2,y2,z2;
        double x3,y3,z3;
        double x4,y4,z4;
        double nx, ny, nz;
        unsigned char R,G,B,T;
    };


    /**
     * This structure is used to display triangle faces.
     * @see Display3D, Viewer3D, Board3DTo2D
     **/
    struct  triangleD3D{
        double x1,y1,z1;
        double x2,y2,z2;
        double x3,y3,z3;
        double nx, ny, nz;
        unsigned char R,G,B,T;
    };


public:

    enum StreamKey {addNewList, updateDisplay, shiftSurfelVisu};
    enum ImageDirection {xDirection, yDirection, zDirection };
    enum TextureMode {RGBMode, GrayScaleMode };

    /// Structure used to display KSPoint in 3D and MeshFromPoints
    /// @see addBall
    ///
    //have to be public because of external functions
    struct ballD3D{
        const double & operator[]( unsigned int i ) const{
            assert(i<3);
            switch (i){
            case 0: {return x;}
            case 1: {return y;}
            case 2: {return z;}
            }
            return x;
        };
        double & operator[]( unsigned int i ) {
            assert(i<3);
            switch (i){
            case 0: {return x;}
            case 1: {return y;}
            case 2: {return z;}
            }
            return x;
        };
        double  x, y, z;
        unsigned char R,G,B,T;
        bool isSigned;
        bool signPos;
        double size;
    };


    /**
     * This structure is used to display polygonal faces.
     * @see Display3D, Viewer3D, Board3DTo2D
     **/
    struct  polygonD3D{
        std::vector<ballD3D> vectBalls;
        double nx, ny, nz;
        unsigned char R,G,B,T;
    };


    /**
     *  Used to display the 2D domain of an image.
     *
     **/
    //have to be public because of external functions
    struct Image2DDomainD3D{
        // The image domain coordinates
        double x1, y1, z1;
        double x2, y2, z2;
        double x3, y3, z3;
        double x4, y4, z4;
        unsigned char R,G,B,T;

        unsigned int myDomainWidth;
        unsigned int myDomainHeight;
        ImageDirection myDirection;

        std::string myMode;

        unsigned int myLineSetIndex;

        /**
        * Constructor
        **/
        template<typename TDomain>
        Image2DDomainD3D( TDomain aDomain, Display3D::ImageDirection normalDir=zDirection,
                          double xBottomLeft=0.0, double yBottomLeft=0.0, double zBottomLeft=0.0, std::string mode= "BoundingBox"){
            BOOST_CONCEPT_ASSERT(( CDomain < TDomain >));
            myMode = mode;
            myDirection=normalDir;
            myDomainWidth = (aDomain.upperBound())[0]-(aDomain.lowerBound())[0]+1;
            myDomainHeight = (aDomain.upperBound())[1]-(aDomain.lowerBound())[1]+1;
            updateDomainOrientation(normalDir, xBottomLeft, yBottomLeft, zBottomLeft);
        }

        /**
       * Update the domain direction from a specific normal direction
       * (Display3D::xDirection, Display3D::yDirection or Display3D::zDirection) and image position
       * from the botton left point.
       * @param normalDir give a predifined normal orientation can be (Display3D::xDirection, Display3D::yDirection or Display3D::zDirection)
       *  @param xBottomLeft the x coordinate of bottom left image point.
       *  @param yBottomLeft the x coordinate of bottom left image point.
       *  @param zBottomLeft the x coordinate of bottom left image point.
       **/
        void updateDomainOrientation( Display3D::ImageDirection normalDir, double xBottomLeft, double yBottomLeft, double zBottomLeft);


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
    struct TextureImage{

        // The quad coordinates should be given in counter clockwise order
        double x1, y1, z1;
        double x2, y2, z2;
        double x3, y3, z3;
        double x4, y4, z4;
        ImageDirection myDirection;
        unsigned int myImageWidth;
        unsigned int myImageHeight;
        unsigned int * myTabImage;

        bool myDrawDomain;
        unsigned int myIndexDomain;
        TextureMode myMode;

        ~TextureImage(){
            delete [] myTabImage;
        };

        /**
       * Copy constructor (needed due to myTabImage)
       **/
        TextureImage(const TextureImage & img):x1(img.x1), y1(img.y1), z1(img.z1),
            x2(img.x2), y2(img.y2), z2(img.z2),
            x3(img.x3), y3(img.y3), z3(img.z3),
            x4(img.x4), y4(img.y4), z4(img.z4),
            myDirection(img.myDirection), myImageWidth(img.myImageWidth),
            myImageHeight(img.myImageHeight),
            myTabImage(img.myTabImage),
            myDrawDomain(img.myDrawDomain),
            myIndexDomain(img.myIndexDomain),
            myMode(img.myMode){

            if(img.myImageHeight>0 && img.myImageWidth>0){
                myTabImage = new  unsigned int [img.myImageWidth*img.myImageHeight];
                for(unsigned int i=0; i<img.myImageWidth*img.myImageHeight; i++){
                    myTabImage[i] = img.myTabImage[i];
                }
            }else{
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
       **/
        template <typename TImageType, typename TFunctor>
        TextureImage( const TImageType & image, const TFunctor &aFunctor,
                      Display3D::ImageDirection normalDir=zDirection,
                      double xBottomLeft=0.0, double yBottomLeft=0.0, double zBottomLeft=0.0,
                      TextureMode aMode= Display3D::GrayScaleMode){
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
       * (Display3D::xDirection, Display3D::yDirection or Display3D::zDirection) and image position
       * from the botton left point.
       * @param normalDir give a predifined normal orientation can be (Display3D::xDirection, Display3D::yDirection or Display3D::zDirection)
       *  @param xBottomLeft the x coordinate of bottom left image point.
       *  @param yBottomLeft the x coordinate of bottom left image point.
       *  @param zBottomLeft the x coordinate of bottom left image point.
       **/

        void updateImageOrientation( Display3D::ImageDirection normalDir, double xBottomLeft, double yBottomLeft, double zBottomLeft);


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
                                     double yTranslation=0.0, double zTranslation=0.0){
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
                it!=itend;
                ++it){
                myTabImage[pos]= aFunctor(image(*it));
                pos++;
            }
        };

        /**
       * return the class name to implment the CDrawableWithDisplay3D concept.
       **/
        std::string className() const;

    private:
        TextureImage(){

        };
    };

    //TODO here

    CanonicEmbedder< S> myEmbedder;

    CanonicCellEmbedder< KS> myCellEmbedder;

    CanonicSCellEmbedder< KS> mySCellEmbedder;




    //----end of private data

    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    virtual ~Display3D(){};


    Display3D(){
        myCurrentFillColor = Color ( 220, 220, 220 );
        myCurrentLineColor = Color ( 22, 22, 222, 50 );
        myScaleX=1.0;
        myScaleY=1.0;
        myScaleZ=1.0;
        myBoundingPtEmptyTag = true;

    };


    Display3D(KS KSEmb){
        myCurrentFillColor = Color ( 220, 220, 220 );
        myCurrentLineColor = Color ( 22, 22, 222, 50 );
        myScaleX=1.0;
        myScaleY=1.0;
        myScaleZ=1.0;
        myBoundingPtEmptyTag = true;

        myCellEmbedder = CanonicCellEmbedder<KS>(KSEmb);
        mySCellEmbedder = CanonicSCellEmbedder<KS>(KSEmb);

    };


    // ----------------------- Interface --------------------------------------
public:

    /**
     * Used to set the current fill color
     * @param aColor the fill color.
     **/
    virtual void setFillColor(DGtal::Color aColor);


    /**
     * Used to set the line fill color
     * @param aColor the line color.
     **/
    virtual void setLineColor(DGtal::Color aColor);


    /**
     * Used to get the fill color
     * @return the current fill color.
     **/

    virtual DGtal::Color getFillColor();

    /**
     * Used to get the line color
     * @return the current line color.
     **/

    virtual DGtal::Color getLineColor();



    virtual unsigned int getCurrentDomainNumber();

    virtual unsigned int getCurrentGLImageNumber();

    /**
     * Add a new 3D Clipping plane represented by ax+by+cz+d = 0
     * A maximal of five clipping plane can be added.
     *
     * @param a a
     * @param b b
     * @param c c
     * @param d d  plane equation.
     **/

    virtual void addClippingPlane(double a, double b, double c, double d, bool drawPlane);




    /**
     * @param objectName the name of the object (generally obtained
     * with a 'object.className()').
     *
     * @return the current mode for the given object name or "" if no
     * specific mode has been set.
     */
    virtual std::string getMode( const std::string & objectName ) const;

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    virtual void createNewLineList(std::string s= "");


    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    virtual void createNewBallList(std::string s= "");


    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     * @todo remove this depthTest perhaps no more important in Viewer3D (and perhaps not used in other viewer).
     **/
    virtual void createNewCubeList(bool depthTest=true, std::string s= "");

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    virtual void createNewQuadList(std::string s= "");

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param s name of the new list
     **/
    virtual void createNewTriangleList(std::string s= "");

    /**
     * Used to create a new list containing new 3D objects
     * (useful to use transparency between different objects).
     * @param depthTest use to include element in a list to apply a depth test (GL_DEPTH_TEST).
     * @param s name of the new list
     **/
    virtual void createNewPolygonList(std::string s= "");


    /**
     * Method to add a specific quad (used by @a addClippingPlane). The normal is computed from the vertex order.
     * x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4 the four coordinates of the quad.
     * @param aColor the quad color.
     */
    virtual void addQuad(double x1, double y1, double z1,  double x2, double y2, double z2,
                         double x3, double y3, double z3,  double x4, double y4, double z4,
                         DGtal::Color aColor);

    /**
     * Method to add a specific quad (used by @a addClippingPlane). The normal is computed from the vertex order.
     * x1, y1, z1, x2, y2, z2, x3, y3, z3  the four coordinates of the triangle.
     * @param aColor the quad color.
     */
    virtual void addTriangle(double x1, double y1, double z1,  double x2, double y2, double z2,
                             double x3, double y3, double z3, DGtal::Color aColor);


    /**
     * Method to add a specific polygon.
     * @param vectPointsPolygon a vector containing the polygon vertex.
     */
    virtual void addPolygon(std::vector<ballD3D> vectPointsPolygon, DGtal::Color aColor);


    /**
     * Method to add a line to the current display.
     *  x1, y1, z1, x2, y2, z2  the two extremty line points.
     * @param color the line color.
     * @param width the line width
     *
     */
    virtual void addLine(double x1, double y1, double z1, double x2, double y2, double z2,
                         const DGtal::Color &color=DGtal::Color(20,20,20,200), double width=0.03);


    /**
     * Method to add specific cube. It includes several modes to
     * display the cube with and without the wire visualisation.
     *
     * @param x cube center x
     * @param y cube center y
     * @param z cube center z.
     * @param color the cube color.
     * @param width the cube width.
     * @param withWire if true add the wire representation.
     */
    virtual void addCube(double x, double y, double z,
                         DGtal::Color color= DGtal::Color(220, 220, 220),
                         double width=1.0);


    /**
     * Method to add a point to the current display.
     * @param x x
     * @param y y
     * @param z z  the point.
     * @param color the point color.
     * @param size the point width
     *
     */
    virtual void addBall(double x, double y, double z ,const DGtal::Color &color=DGtal::Color(200,20,20),
                         double size=0.05);



    /**
     * Specific to display a surfel from Kahlimsky space. The display can
     * take into accounts the sign of the cell.
     *
     *  x,y,z the surfel center.
     *  xSurfel, ySurfel , zSurfel  specify if the surfel has its main face in the direction of
     *                                     the x-axis, y-axis or z-axis.
     * @param sizeShiftFactor set the distance between the display of the surfel and potential Cube.
     * @param positionShift translate the KSsurfel from the asso
     * missing text line 662 in the original document
     * @param isSigned to specify if we want to display an signed or unsigned Cell.
     * @param aSign if @ref isSigned is true it will be used to apply a different displays
     *  according this boolean  parameter  (if @a aSign=true oriented in the direct axis orientation)
     *
     */
    virtual void addSurfelPrism(double x, double y, double z,
                                bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor,
                                double sizeFactor=1.0, bool isSigned= false, bool aSign=true);

    /**
     * Specific to display a surfel from Kahlimsky space in basic mode.
     *
     *  x,y,z the surfel center.
     *  xSurfel, ySurfel , zSurfel  specify if the surfel has its main face in the direction of
     *                                     the x-axis, y-axis or z-axis.
     * @param sizeShiftFactor set the distance between the display of the surfel and potential Cube.
     *
     */
    virtual void addQuad(double x, double y, double z,
                                bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor,
                                double sizeFactor=1.0, bool isSigned= false, bool aSign=true);



    /**
     * Add a signed KSLinel from the Kahlimsky space. Display it as a cone.
     *
     *  x1, y1, z1 first point of the cone.
     *  x2, y2, z2 second point of the cone..

     * @param width the width of the cone (default= 0.02)
     *
     */
    virtual void addCone(double x1, double y1, double z1,
                         double x2, double y2, double z2,
                         double width=0.02);


    /**
     * Add a non signed KSLinel from the Kahlimsky space. Display it as a simple cylinder.
     *
     *  x1, y1, z1 first point of the extremity point of the cylinder.
     *  x2, y2, z2 second point of the extremity point of the cylinder.

     * @param width the width of the cylinder (default= 0.02))
     *
     */
    virtual void addCylinder(double x1, double y1, double z1,
                             double x2, double y2, double z2,
                             double width=0.02);


    /**
     * Used to update the scene bounding box when objects are added.
     *
     *  x, y, z the coordinates to be taken into accounts.
     *
     **/
    void updateBoundingBox(double x, double y, double z);




    /**
     * Export as Mesh the current displayed elements.
     *
     * @param aMesh : (return)  the mesh containing the elements of the display.
     *
     **/
    void exportToMesh(Mesh<Display3D::ballD3D> & aMesh ) const;



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
     **/
    void updateOrientationTextureImage(unsigned int imageIndex,
                                       double xPosition, double yPosition, double zPosition, ImageDirection newDirection);



    /**
     *
     */
    template<typename TDomain>
    void addImage2DDomainD3D(const TDomain &anImageDomain, std::string mode,
                             const DGtal::Color &aColor=DGtal::Color::Red );


    /**
     *
     */
    void updateAn2DDomainOrientation(unsigned int imageIndex,
                                     double xPosition, double yPosition, double zPosition, ImageDirection newDirection);

    /**
     *
     */
    void translateAn2DDomain(unsigned int domainIndex, double xTranslation, double yTranslation, double zTranslation);

    /**
     *
     */
    std::vector<DGtal::Display3D< S, KS>::lineD3D>  compute2DDomainLineRepresentation( Image2DDomainD3D &anImageDomain, double delta );

    /**
     *
     */
    std::vector<DGtal::Display3D< S, KS>::lineD3D>  compute2DDomainLineRepresentation( Image2DDomainD3D &anImageDomain);

    /**
     * Draws the drawable [object] in this board. It should satisfy
     * the concept CDrawableWithViewer3D, which requires for instance a
     * method setStyle( Viewer3D & ).
     *
     * @param object any drawable object.
     * @return a reference on 'this'.
     */
    template <typename TDrawableWithDisplay3D>
    Display3D & operator<<( const  TDrawableWithDisplay3D & object );



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


    /**
     * Use to change the main axis scale (usefull if we need to display
     * data with variable scales, as for instance from medical imagery
     * scanner)
     *
     * @param sx scale factor for the x axis (scale increased if >1, decreased if <1, reflected if -1).
     * @param sy scale factor for the y axis (scale increased if >1, decreased if <1, reflected if -1).
     * @param sz scale factor for the z axis (scale increased if >1, decreased if <1, reflected if -1).
     *
     **/
    void setScale(float sx, float sy, float sz);


    //TODO methods
    /**
      * Use to embed a DGtal point into space
      * @param pt a DGtal Point
      */

    typename DGtal::CanonicEmbedder<S>::RealPoint embed(const Z3i::Point & dp) const ;

    typename DGtal::CanonicEmbedder<S>::RealPoint embed(const ballD3D & dp) const ;

    /**
      * Use to embed a signed DGtal kahlimsky cell into space
      * @param cell a kahlimsky cell
      */
    typename DGtal::CanonicSCellEmbedder<KS>::RealPoint embedKS( const Z3i::SCell & cell ) const;

    /**
      * Use to embed an unsigned DGtal kahlimsky cell into space
      * @param cell kahlimsky cell
      */
    typename DGtal::CanonicCellEmbedder<KS>::RealPoint embedK( const Z3i::Cell & cell ) const;

    //---end interface

    // ------------------------- Protected Datas ------------------------------
public:

    /**
     * The associated map type for storing possible modes used for
     * displaying for digital objects.
     */
    typedef std::map< std::string, std::string > ModeMapping;

    /**
     * The associated map type for storing the default styles of
     * digital objects.
     */
    typedef std::map< std::string,CountedPtr<DrawableWithDisplay3D> > StyleMapping;


    ModeMapping myModes;
    /**
     * For instance, may associate a new style object T1 to the class
     * "HyperRectDomain": myStyles[ "HyperRectDomain" ] = T1.
     *
     * One can also store a new style T2 for a specific mode used for
     * drawing a class:  myStyles[ "HyperRectDomain/Paving" ] = T2.
     *
     * Modes may only be used in objects implementing the concept
     * CDrawableWithBoard2D.
     */
    StyleMapping myStyles;

    /// True if the bounding box is empty (no objects added)
    bool myBoundingPtEmptyTag;
    double  myBoundingPtUp [3];
    double  myBoundingPtLow [3];

protected:

    DGtal::Color myCurrentFillColor;

    DGtal::Color myCurrentLineColor;

    /// Used to specialized visualisation with KS surfels/cubes.
    ///
    double myCurrentfShiftVisuSurfelPrisms;

    /// Used to represent all the list used in the display.
    ///
    std::vector< std::vector<cubeD3D> > myCubeSetList;

    /// Used to represent all the list of line primitive
    ///
    std::vector< std::vector<lineD3D> > myLineSetList;

    /// Used to represent all the list of point primitive
    ///
    std::vector< std::vector<ballD3D> > myBallSetList;

    /// Represent all the clipping planes added to the scene (of maxSize=5).
    ///
    std::vector< clippingPlaneD3D > myClippingPlaneList;

    /// For saving all surfels of Khalimsky space (used to display Khalimsky Space Cell)
    ///
    std::vector< quadD3D > mySurfelPrismList;

    // Represents all the planes drawn in the Display3D
    std::vector<std::vector< quadD3D > > myQuadSetList;

    /// Represents all the triangles drawn in the Display3D
    std::vector<std::vector< triangleD3D > > myTriangleSetList;

    // Represents all the polygon drawn in the Display3D
    std::vector<std::vector<polygonD3D> > myPolygonSetList;

    /// names of the lists in myCubeSetList
    ///
    std::vector<std::string> myCubeSetNameList;

    /// names of the lists in myLineSetList
    ///
    std::vector<std::string> myLineSetNameList;

    /// names of the lists in myBallSetList
    ///
    std::vector<std::string> myBallSetNameList;
    /// names of the lists in myClippingPlaneList
    ///
    std::vector<std::string> myClippingPlaneNameList;

    /// names of the lists in mySurfelPrismList
    ///
    std::vector<std::string> mySurfelPrismNameList;

    /// names of the lists in myQuadList
    ///
    std::vector<std::string> myQuadSetNameList;

    /// names of the lists in myTriangleList
    ///
    std::vector<std::string> myTriangleSetNameList;

    /// names of the lists in myPolygonList
    ///
    std::vector<std::string> myPolygonSetNameList;


    /// Used to define if GL_TEST_DEPTH is used.
    std::vector<bool> myListCubeDepthTest;

    float myMeshDefaultLineWidth;

    // Used to store all displayed images
    std::vector<TextureImage> myGSImageList;


    // Used to store all the domain
    std::vector<Image2DDomainD3D> myImageDomainList;

    //----end of protected datas

    // ------------------------- Hidden services ------------------------------

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Display3D ( const Display3D & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Display3D & operator= ( const Display3D & other );

    //----end of hidden services

    // ------------------------- Internals ------------------------------------
private:

}; // end of class Display3D



/**
   * Calculate the cross product of two 3d vectors and return it.
   * @param dst destination vector.
   * @param srcA source vector A.
   * @param srcB source vector B.
   */
static void cross (double dst[3], double srcA[3], double srcB[3]);

/**
   * Normalize the input 3d vector.
   * @param vec source & destination vector.
   */
static void normalize (double vec[3]);


/**
   * Overloads 'operator<<' for displaying objects of class 'Display3D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Display3D' to write.
   * @return the output stream after the writing.
   */
template <typename S, typename KS>
std::ostream&
operator<< ( std::ostream & out, const DGtal::Display3D<S, KS> & object );

/**
   * Operator ">>" to export a Display3D into a Mesh
   *
   * @param aDisplay3D the Display3D to be exported.
   * @param aMesh (return) the resulting mesh.
   *
   **/
template <typename S, typename KS>
void
operator>> ( const Display3D<S, KS> &aDisplay3D, DGtal::Mesh< typename Display3D<S, KS>::ballD3D> &aMesh);


/**
   * Operator ">>" to export a Display3D directly a file
   *
   * @param aDisplay3D the Display3D to be exported.
   * @param aFilename (return) the resulting mesh.
   *
   **/
template < typename S, typename KS>
void
operator>> ( const Display3D< S, KS> &aDisplay3D,  std::string aFilename);


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/Display3D.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display3D_h

#undef Display3D_RECURSES
#endif // else defined(Display3D_RECURSES)
