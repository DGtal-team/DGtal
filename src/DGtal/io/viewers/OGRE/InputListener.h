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
* @file InputListener.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/10
*
* Header file for module InputListener.cpp
*
* This file is part of the DGtal library.
*/

#if defined(InputListener_RECURSES)
#error Recursive header files inclusion detected in InputListener.h
#else // defined(InputListener_RECURSES)
/** Prevents recursive inclusion of headers. */
#define InputListener_RECURSES

#if !defined InputListener_h
/** Prevents repeated inclusion of headers. */
#define InputListener_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
//----------------------------------------------------Includes OIS
#include <OIS.h>
#include <OGRE/Ogre.h>
#include "DGtal/io/viewers/OGRE/CameraMan.h"
#include "DGtal/io/viewers/OGRE/DGtalSingleton.h"			

#if (OGRE_PLATFORM == OGRE_PLATFORM_APPLE) && __LP64__
#include "DGtal/io/viewers/OGRE/mousecursor.h"			
#endif 
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
  {

/////////////////////////////////////////////////////////////////////////////
// class InputListener
    /**
    * Description of class 'InputListener' <p>
    * \brief Aim:  Manage the inputs for the Ogre3D viewer
    */

        class ViewerOgre3D;

        class InputListener : public DGtal::Singleton<InputListener> ,
			      public Ogre::FrameListener,
			      public OIS::KeyListener, OIS::MouseListener ,
			      public Ogre::WindowEventListener
          {

              // ----------------------- Standard services ------------------------------

            public:

              /**
               * Destructor.
               */
              ~InputListener();

              /**
               * Constructor.
               */
              InputListener ( Ogre::RenderWindow * aWindow,
                              ViewerOgre3D * aViewer,
                              Ogre::Camera * aCamera,
                              Ogre::SceneManager * aSceneManager );



              // ----------------------- Interface --------------------------------------

            public:


              /**
               * Method that initiate the inputManager (First frame)
               */
              virtual bool frameStarted ( const Ogre::FrameEvent & evt );


              /**
               * the rendering method 
               */
              virtual bool frameRenderingQueued ( const Ogre::FrameEvent& );

              /**
                * The last frame
                */
              virtual bool frameEnded ( const Ogre::FrameEvent &evt );


              /**
               * Handles a key press
               */
              bool keyPressed ( const OIS::KeyEvent &e );


              /**
               * Handles a mouse press
               */
              bool mousePressed ( const OIS::MouseEvent &e, OIS::MouseButtonID id );


              /**
               * Handles a mouse release
               */
              bool mouseReleased ( const OIS::MouseEvent &e, OIS::MouseButtonID id );


              /**
               * Handles a key release
               */
              bool keyReleased ( const OIS::KeyEvent &e );


              /**
               * Handles a mouse move
               */
              bool mouseMoved ( const OIS::MouseEvent &e );


              /**
               * Handles a window resize
               */
	      virtual void windowResized(Ogre::RenderWindow* rw);
              /**
               * Handles a window close
               */
              virtual void windowClosed(Ogre::RenderWindow* rw);
	     /**
               * makes the render loop stop
               */
              void stopRendering ( );
	      
	      
              /**
               * makes the render loop starts
               */
              void startRendering ( );
	      
	      
	      /**
	      * Resets the position of the scene center
	      */
	      void resetSCPosition();
	      
	      /**
	       *  Returns a pointer on the cameraman
	       * 
	       */
	        CameraMan * getCameraMan() 
		{
		  return myCameraMan;
		}


	      /**
	       *  Returns a pointer on the keyboard
	       * 
	       */
	        OIS::Keyboard * getKeyBoard() 
		{
		  return myKeyboard;
		}


	      /**
	       *  Returns a pointer on the mouse
	       * 
	       */
	        OIS::Mouse * getMouse() 
		{
		  return myMouse;
		}

	       /**
	       *  Returns a pointer on the mouse
	       * 
	       */
	        Ogre::Timer * getTimer() 
		{
		  return myTimer;
		}

	       /**
	       *  Updates some attributes of the viewer
	       * 
	       */
	        void updateViewer(double timeSinceLastFrame);
		/**
		 *  Tells if the application is currently runing
		 */
                bool viewerIsRunning()
		{
		     return !myShutdown;
		}

	      
	                    // ----------------------- Private members --------------------------------------
            private:
              /**
              * Flag that manage the right clic
              */	    
              bool myMouseRightClicked;
	      
	      /**
              * Flag that manage the left clic
              */
              bool myMouseLeftClicked;
	      
	      /**
              * The viewer
              */	      
              ViewerOgre3D * myViewer;
	      
	      
	     /**
              * The window
              */
              Ogre::RenderWindow * myWindow;
	      
	      
	     /**
              * Flag that manages the rendering
              */  
              bool myShutdown;
	      
	      /**
	       * Flag that tells if the shift key is pressed
	       */
	       bool mySelectionMode;

	      /**
	       * Flag that tells if the control key is pressed
	       */
	      bool myControlMode;
	      /**
              * The keyboard
              */
              OIS::Keyboard * myKeyboard;
	      
	      
	      /**
              * The mouse
              */	      
              OIS::Mouse * myMouse;
              	     
              	     
             /**
              * The mouse cursor
              */	
			#if (OGRE_PLATFORM == OGRE_PLATFORM_APPLE) && __LP64__
				 MouseCursor * myMouseCursor;
			#endif    
    
	      
	      /**
              * The OIS input listenner
              */		      
              OIS::InputManager * myInputManager;
	      
	      
	     /**
              * The Camera Object
              */		   
              Ogre::Camera * myCamera;
	      
	      /**
              * The CameraMan
              */		   
              CameraMan * myCameraMan;
	      
	      
	      /**
              *  Free 3D moving flag
              */	      
              bool myIsInShiftMode;
	      
	      
	      /**
              * The ogre scene manager
              */	
              Ogre::SceneManager * mySceneMgr;
	      
	      int myMouseX;
	      int myMouseY;
              double myTime;

	      /**
	       * The rendering timer
               */
               Ogre::Timer * myTimer;

	      /**
	       * 
   	       */
		virtual void windowMoved(Ogre::RenderWindow * rw);

          };
      }

#include "ViewerOgre3D.h"
#include "InputListener.ih"

  

// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined InputListener_h

#undef XXX_RECURSES
#endif // else defined(InputListener_RECURSES)
            
