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
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Display3D
/**
 * Description of class 'Display3D' <p>
 * \brief Aim:
 */
class Display3D
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
 
  Display3D(){};
  virtual ~Display3D(){};

    // ----------------------- Interface --------------------------------------
public:
  enum StreamKey {addNewList, updateDisplay, shiftSurfelVisu};
  
 
  virtual void setFillColor(QColor aColor);
  virtual void setLineColor(QColor aColor);
   
  /**
   * Add a new 3D Clipping plane represented by ax+by+cz+d = 0 
   * A maximal of five clipping plane can be added.
   *
   * @param a, b, c, d : plane equation.
   **/
  
  virtual void addClippingPlane(double a, double b, double c, double d, bool drawPlane){};
  
   /**
  * Set camera up-vector.
  * @param x x coordinate of up-vector.
  * @param y y coordinate of up-vector.
  * @param z z coordinate of up-vector.
  */
  virtual void setCameraUpVector(double x, double y, double z){}; 
  
  /**
  * Set camera position.
  * @param x x position.
  * @param y y position.
  * @param z z position.
  */
  virtual void setCameraPosition(double x, double y, double z) {  };
  
  /**
  * Set near and far distance.
  * @param near near distance.
  * @param far far distance.
  */
  virtual void setNearFar(double near, double far){};
  

    
  /**
  * Set camera direction.
  * @param x x direction.
  * @param y y direction.
  * @param z z direction.
  */
  virtual void setCameraDirection(double x, double y, double z) { };

  
  
  /**
   * @param objectName the name of the object (generally obtained
   * with a 'object.styleName()').
   *
   * @return the current mode for the given object name or "" if no
   * specific mode has been set.
   */
  virtual std::string getMode( const std::string & objectName ) const;
  
   /**
    * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  

  virtual void createNewLineList(){};
  

  /**
   * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  

  virtual void createNewVoxelList(bool depthTest=true){};



  
  /**
   * Used to create a new list containing new 3D objects
   * (useful to use transparency between different objects).
   * 
   **/  
  
  virtual void createNewPointList(){};



  virtual void addQuad(double x1, double y1, double z1,  double x2, double y2, double z2,
		       double x3, double y3, double z3,  double x4, double y4, double z4, QColor aColor){};

  

  virtual void addLine(double x1, double y1, double z1, double x2, double y2, double z2, 
		       const QColor &color=QColor(20,20,20,200), double width=1.5){};
  
  virtual void addVoxel(int x, int y, int z, QColor color= QColor(220, 220, 220), double width=0.5,bool withWire=false){};
  
  virtual void addPoint(double x, double y, double z ,const QColor &color=QColor(200,20,20), double size=40){
  cerr << "in addd Point pere" << endl;};
  
  virtual QColor getFillColor();
   
  virtual QColor getLineColor();

 
  
  virtual void addKSSurfel(double x, double y, double z, 
		   bool xSurfel, bool ySurfel, bool zSurfel, double sizeShiftFactor, 
			   bool isOriented= false, bool isOrientedPositively=true, bool basicMode=false){};
  
  virtual void addKSVoxel(int x, int y, int z){};
  
  virtual void addKSPointel(double x, double y, double z, double size=0.1,
			    bool isSigned=false, bool signPos=true){};
  
  virtual void addKSLinel(double x1, double y1, double z1,
		  double x2, double y2, double z2,
			  double width=0.02, bool isSigned=false, bool signPos=true){};
  

  /**
   * Draws the drawable [object] in this board. It should satisfy
   * the concept CDrawableWithViewer3D, which requires for instance a
   * method selfDraw( Viewer3D & ).
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
   * The associated map type for storing possible modes used for
   * displaying for digital objects.
   */
  typedef std::map< std::string, std::string > ModeMapping;
  
  
  
  /**
   * The associated map type for storing the default styles of
   * digital objects.
   */
  typedef std::map< std::string,CountedPtr<DrawableWithDisplay3D> > StyleMapping;
  



    // ------------------------- Protected Datas ------------------------------
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
     * CDrawableWithBoard2D.
     */
    StyleMapping myStyles;

  protected:
  QColor myCurrentFillColor;
  QColor myCurrentLineColor;


private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

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

    // ------------------------- Internals ------------------------------------
private:



}; // end of class Display3D


/**
 * Overloads 'operator<<' for displaying objects of class 'Display3D'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Display3D' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const Display3D & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/Display3D.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display3D_h

#undef Display3D_RECURSES
#endif // else defined(Display3D_RECURSES)
