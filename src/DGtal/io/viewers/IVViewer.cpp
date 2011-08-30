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

/**
 * @file IVViewer.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/04
 *
 * Implementation of methods defined in IVViewer.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <Inventor/nodes/SoCube.h>
#include "DGtal/io/viewers/IVViewer.h"
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class IVViewer
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
DGtal::IVViewer::~IVViewer()
{
#ifdef SGI_INVENTOR
  if ( myViewer != 0 ) delete (SoXtExaminerViewer*) myViewer;
  //  if ( myWindow != 0 ) delete (Widget) myWindow;
#else
  if ( myViewer != 0 ) delete (SoQtExaminerViewer*) myViewer;
  // if ( myWindow != 0 ) delete (QWidget*) myWindow;
#endif
  if ( myRoot != 0 ) myRoot->unref();
}

/**
 * Constructor.
 *
 * @param argc the number of parameters.
 * @param argv an array of C strings storing the parameters.
 */

DGtal::IVViewer::IVViewer( int argc, char** argv )
  : myWindow( 0 ), myViewer( 0 ), myRoot( 0 ), myTitle( "IVViewer" )
{
  // Initialize Inventor and Xt
#ifdef SGI_INVENTOR
  cout << "Initializing Inventor and Xt..." << endl;
  myWindow = (Widget) SoXt::init( argc, argv, "Digital Viewer" );
#else
  cout << "Initializing Inventor and Qt..." << endl;
  myWindow = (QWidget*) SoQt::init( argc, argv, "Digital Viewer" );
#endif

  if (myWindow == 0) exit(1);

  // Define Inventor node.
  cout << "Creating node, shapehints, material..." << endl;
  myRoot = new SoSeparator;
  myRoot->ref();

#ifdef SGI_INVENTOR
  myViewer = (SoXtExaminerViewer *)
    new SoXtExaminerViewer( (Widget) myWindow );
#else
  myViewer = (SoQtExaminerViewer *)
    new SoQtExaminerViewer( (QWidget*) myWindow );
#endif
  
}

/**
 * @return the root of the scene graph.
 */
SoSeparator* DGtal::IVViewer::root() const
{
  return myRoot;
}

/**
 * @param title the name of the window. 
 */
void DGtal::IVViewer::setTitle( const std::string & title )
{
  myTitle = title;
}



/**
 * Sets the parameters of the observing camera with Euler
 * spherical coordinates.
 *
 * @param latitude the latitude of observation (in degrees)
 * @param longitude the longitude of observation (in degrees)
 */
void DGtal::IVViewer::setCamera( float latitude, float longitude )
{
  SbVec3f x_axis( 1.0, 0.0, 0.0 );
  latitude *= M_PI / 180.0f;
  SbRotation x_rot( x_axis, latitude );
  SbVec3f y_axis( 0.0, 1.0, 0.0 );
  longitude *= - M_PI / 180.0f;
  SbRotation y_rot( y_axis, longitude );
  SoTransform* transform = new SoTransform;
  transform->rotation.setValue( y_rot * x_rot );
  myRoot->insertChild( transform, 0 );
  
}


/**
 * Show window and gives hand to inventor main loop.
 */
void DGtal::IVViewer::show() const
{
#ifdef SGI_INVENTOR
  SoXtExaminerViewer* lViewer = (SoXtExaminerViewer*) myViewer;
#else
  SoQtExaminerViewer* lViewer = (SoQtExaminerViewer*) myViewer;
#endif
 
  lViewer->setSceneGraph( myRoot );
  //  lViewer->setTitle( myTitle.c_str() );
  lViewer->show();
  lViewer->viewAll();

#ifdef SGI_INVENTOR
  SoXt::show((Widget) myWindow);
  SoXt::mainLoop();
#else
  SoQt::show((QWidget*) myWindow);
  SoQt::mainLoop();
#endif

}



///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::IVViewer::selfDisplay ( std::ostream & out ) const
{
    out << "[IVViewer]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::IVViewer::isValid() const
{
    return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
