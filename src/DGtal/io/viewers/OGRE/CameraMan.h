/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

#pragma once

/**
* @file CameraMan.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/10
*
* Header file for module CameraMan.cpp
*
* This file is part of the DGtal library.
*/



#if defined(CameraMan_RECURSES)
#error Recursive header files inclusion detected in CameraMan.h
#else // defined(CameraMan_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CameraMan_RECURSES

#if !defined CameraMane_h
/** Prevents repeated inclusion of headers. */
#define CameraMan_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "OGRE/Ogre.h"
#include "OIS/OIS.h"

//////////////////////////////////////////////////////////////////////////////
// Static variables

const double DefaultCameraSpeed = 1;
const double DefaultCameraRotationSpeed = 0.0008;

//////////////////////////////////////////////////////////////////////////////

//namespace DGtal
//  {

/////////////////////////////////////////////////////////////////////////////
// class CameraMan
/**
* Description of class 'CameraMan' <p>
* \brief Aim: Acts on the camera of the viewer
*/

class CameraMan
  {

      // ----------------------- Standard services ------------------------------

    public:
      /**
      * Constructor
      */
      CameraMan ( Ogre::SceneNode * aNode,Ogre::Camera * aCamera, int aSpeed = DefaultCameraSpeed,
                  int aRotationSpeed = DefaultCameraRotationSpeed );


      /**
      * Destructor.
      */
      virtual ~CameraMan( );

      // ----------------------- Interface --------------------------------------

    public:

      /**
        * Public method that handles the user key press
       */
      bool handleKeyPress ( const OIS::KeyEvent & evt );

      /**
        * Public method that handles the user key Release
       */
      bool handleKeyRelease ( const OIS::KeyEvent & evt );



      /**
        * Public method that handles the user mouse move
       */
      bool handleMouseMove ( const OIS::MouseEvent & evt );



      /**
       *  Public method that handles the user mouse click
      */
      bool handleMouseClick ( const OIS::MouseEvent &e, OIS::MouseButtonID id );


      /**
       *  Public method that handles the user mouse release
      */
      bool handleMouseRelease ( const OIS::MouseEvent &e, OIS::MouseButtonID id );


      /**
       *  Public method that acts on the camera geometric attributes
      */
      void render ( double timeSinceLastFrame  );


      /**
       *  Public method that makes the camera turns around the centralNode
       *
       */
      void turnCamera ( int x, int y );


      /**
      *  Public method that makes the camera translate
      *
      */
      void translateCamera ( double x, double y );


      /**
      *  Public method that updates the scene center
      *
      */
      void setSceneCenter ( Ogre::Vector3 aSceneCenter );

      /**
      *  Public method returns the new  scene center
      *
      */
      Ogre::Vector3 & getNewSceneCenter()
      {
        return mySceneCenter;
      }
      
      
      
      /**
      *  Public method that translates the cameraMan by aVector
      *
      */
      void moveCameraman ( Ogre::Vector3 aVector );
      
      /**
      *  Public method that tests if the cameraMan is free turning
      *
      */
      bool isTurning (  )
      {
		return myIsTurning;
      }
      
      /**
      *  Public method that translates the cameraMan by aVector
      *
      */
      void stopTurning ( )
      {
	 myIsTurning=false;
      }
      
      
      /**
      *  Public method that returns the node
      *
      */
      Ogre::SceneNode * getSceneNode ( );
      
      /**
       *  Public method that sets the initial position
       */
      void setInitialPosition(Ogre::Vector3 & newPos);

      /**
       *  Public method that resets the cameraman orientation and position
       */
      void reset();

      /**
       *  Public method that permits to zoom
       */
      void zoom(int z);
      

      //--------------------------- Protected attributes---------------------------


    protected :

      // The camera Object
      Ogre::Camera * myCamera;
      Ogre::SceneNode * myCameraNode;


      // Control flags
      bool myIsOnShiftMode;

      // Moving flags
      bool myIsGoingFoward;
      bool myIsGoingBackward;
      bool myIsGoingLeftward;
      bool myIsGoingRightward;


      // Moving vectors
      Ogre::Vector3 myTempMouseMove;
      Ogre::Vector3 mySceneCenter;
      Ogre::Vector3 myInitialPosition;
      Ogre::Quaternion mySavedOrientation;

      // Moving attributes
      int mySpeed;
      int myRotationSpeed;
      int myX;
      int myY;

      bool myIsTurning;



// ------------------------- Hidden services ------------------------------

    protected:
      /**
       *  private method that starts a motion
      */

      void startGoingFoward()
      {
        myIsGoingFoward = true;
      }

      /**
       *  private method that starts a motion
      */
      void startGoingBackward()
      {
        myIsGoingBackward = true;
      }

      /**
       *  private method that starts a motion
      */
      void startGoingLeftward()
      {
        myIsGoingLeftward = true;
      }


      /**
       *  private method that starts a motion
      */
      void startGoingRightward()
      {
        myIsGoingRightward = true;
      }


      /**
        *  private method that ends a motion
       */
      void stopGoingFoward()
      {
        myIsGoingFoward = false;
      }


      /**
        *  private method that ends a motion
       */
      void stopGoingBackward()
      {
        myIsGoingBackward = false;
      }


      /**
        *  private method that ends a motion
       */
      void stopGoingLeftward()
      {
        myIsGoingLeftward = false;
      }


      /**
        *  private method that ends a motion
       */
      void stopGoingRightward()
      {
        myIsGoingRightward = false;
      }


      /**
        *  private method that adds a camera moving
       */
      void updateCamera ( double x, double y, double z );

  }; // end of class CameraMan

//} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////

#endif // !defined CameraMan_h
#include "CameraMan.ih"
#undef CameraMan_RECURSES
#endif // else defined(CameraMan_RECURSES)

