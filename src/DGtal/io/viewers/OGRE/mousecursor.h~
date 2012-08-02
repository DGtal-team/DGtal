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
* @file MouseCursor.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/10
*
* Header file for module MouseCursor.cpp
*
* This file is part of the DGtal library.
*/

#if defined(MouseCursor_RECURSES)
#error Recursive header files inclusion detected in MouseCursor.h
#else // defined(MouseCursor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MouseCursor_RECURSES

#if !defined MouseCursor_h
/** Prevents repeated inclusion of headers. */
#define MouseCursor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <OgreOverlay.h>
#include <OgreOverlayManager.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreOverlayContainer.h>
#include <OgreTechnique.h>
#include "OgreTextAreaOverlayElement.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
  {

/////////////////////////////////////////////////////////////////////////////
// class MouseCursor
    /**
    * Description of class 'MouseCursor' <p>
    * \brief Aim: provides a mouse cursor to the viewer
    */

    class MouseCursor
      {
// ----------------------- Standard services ------------------------------

        public:
          /**
          * Constructor.
          */
          MouseCursor();


          /**
          * Destructor.
          */
          ~MouseCursor ( );


          // ----------------------- Interface ------------------------------
          /**
          *  Sets the cursor image.
          */
          void setImage ( const Ogre::String& filename );


          /**
          *  Sets the cursor limits by window dimensions.
          */
          void setWindowDimensions ( unsigned int width, unsigned int height );



          /**
           *  Shows/Hides the cursor
           */
          void setVisible ( bool visible );


          /**
           *  Updates the cursor position
           */
          void updatePosition ( int x, int y );

          /**
           *  Avoids going out of the window
           */
          Ogre::Real clamp ( Ogre::Real a, Ogre::Real min, Ogre::Real max );



          /**
            * Writes/Displays the object on an output stream.
            * @param out the output stream where the object is written.
            */
          void selfDisplay ( std::ostream & out ) const;

          /**
          * Checks the validity/consistency of the object.
          * @return 'true' if the object is valid, 'false' otherwise.
          */
          bool  isValid() const;


        private:
           Ogre::TextAreaOverlayElement* myText ;
          /**
           *  The overlay
           */
          Ogre::Overlay* myGuiOverlay;

          /**
           *  The cursor
           */
          Ogre::OverlayContainer* myCursorContainer;

          /**
           *  It's image
           */
          Ogre::TexturePtr myTexture;


          /**
           *  the material
           */
          Ogre::MaterialPtr  myMaterial;


          /**
           *  Window information
           */
          Ogre::Real myWindowWidth;
          Ogre::Real myWindowHeight;

      };

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#endif


    /**
    * Overloads 'operator<<' for displaying objects of class 'XXX'.
    * @param out the output stream where the object is written.
    * @param object the object of class 'XXX' to write.
    * @return the output stream after the writing.
    */
// //
///////////////////////////////////////////////////////////////////////////////
  }

#endif // !defined XXX_h

#undef XXX_RECURSES
#endif // else defined(XXX_RECURSES)
