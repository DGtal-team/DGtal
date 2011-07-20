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
 * @file IVViewer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/04
 *
 * Header file for module IVViewer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IVViewer_RECURSES)
#error Recursive header files inclusion detected in IVViewer.h
#else // defined(IVViewer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IVViewer_RECURSES

#if !defined IVViewer_h
/** Prevents repeated inclusion of headers. */
#define IVViewer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#ifdef SGI_INVENTOR
#include <Inventor/Xt/SoXt.h>
#include <Inventor/Xt/viewers/SoXtExaminerViewer.h>
#else
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#endif
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/SbRotation.h>
#include <Inventor/fields/SoSFRotation.h>

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class IVViewer
  /** 
   * Description of class 'IVViewer' <p> 
   * \brief Aim: A facade to represent an
   * inventor window for 3D objects. May be a SoXt or a SoQt examiner
   * viewer.
   * NB: backported from ImaGeneUtils library.
   * @todo SoXt is yet not checked.
   */
class IVViewer
{
    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    ~IVViewer();

    /**
     * Constructor.
     *
     * @param argc the number of parameters.
     * @param argv an array of C strings storing the parameters.
     */
    IVViewer( int argc = 0, char** argv = 0 );

    /**
     * @return the root of the scene graph.
     */
    SoSeparator* root() const;

    /**
     * @param title the name of the window. 
     */
  void setTitle( const std::string & title );

    /**
     * Sets the parameters of the observing camera with Euler
     * spherical coordinates.
     *
     * @param latitude the latitude of observation (in degrees)
     * @param longitude the longitude of observation (in degrees)
     */
    void setCamera( float latitude, float longitude );

    /**
     * Show window and gives hand to inventor main loop.
     */
    void show() const;

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
    // ------------------------- Private Datas --------------------------------
private:
#ifdef SGI_INVENTOR
    /** 
     * Inventor window. May be a Widget or a QWidget*.
     */
    Widget myWindow;

    /** 
     * Inventor viewer. May be a SoXtExaminerViewer* or a SoQtExaminerViewer*
     */
    SoXtExaminerViewer* myViewer;
#else
    /** 
     * Inventor window. May be a Widget or a QWidget*.
     */
    QWidget* myWindow;

    /** 
     * Inventor viewer. May be a SoXtExaminerViewer* or a SoQtExaminerViewer*
     */
    SoQtExaminerViewer* myViewer;
#endif

    /**
     * The root of the scene graph.
     */
    SoSeparator* myRoot;

  /**
   * The title of the window.
   */
  std::string myTitle;

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    IVViewer();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    IVViewer ( const IVViewer & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    IVViewer & operator= ( const IVViewer & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class IVViewer


/**
 * Overloads 'operator<<' for displaying objects of class 'IVViewer'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'IVViewer' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const IVViewer & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/viewers/IVViewer.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IVViewer_h

#undef IVViewer_RECURSES
#endif // else defined(IVViewer_RECURSES)
