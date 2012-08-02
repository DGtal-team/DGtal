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
* @file TextZone.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/12
*
* Header file for module TextZone.cpp
*
* This file is part of the DGtal library.
*/

#if defined(TextZone_RECURSES)
#error Recursive header files inclusion detected in TextZone.h
#else // defined(TextZone_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TextZone_RECURSES

#if !defined TextZone_h
/** Prevents repeated inclusion of headers. */
#define TextZone_h

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
// class TextZone
    /**
    * Description of class 'TextZone' <p>
    * \brief Aim: provides text zones on the screen
    */

    class TextZone
      {
// ----------------------- Standard services ------------------------------

        public:
          /**
          * Constructor.
          */
          TextZone(std::string aContent);


          /**
          * Destructor.
          */
          ~TextZone ( );



          /**
           *  Shows/Hides the cursor
           */
          void setVisible ( bool visible );


          /**
           *  Flips the cursor visibility
           */
          void flipVisibility ( );



	  /**
	    *  Changes the text position
	    */
	  void setPosition ( int x, int y );
	  
	  /**
	   *  Changes the text position
	   */
	  void setCaption(std::string newCaption);

	  /**
	   *  Changes the text size
	   */
	  void setSize(int aSize);
	  
	  
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
	  
	   Ogre::OverlayContainer* myPanel;
	  
	  



      };

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#endif


    /**
    * Overloads 'operator<<' for displaying objects of class 'TextZone'.
    * @param out the output stream where the object is written.
    * @param object the object of class 'TextZone' to write.
    * @return the output stream after the writing.
    */
// //
///////////////////////////////////////////////////////////////////////////////
  }

#endif // !defined TextZone_h

#undef TextZone_RECURSES
#endif // else defined(TextZone_RECURSES)
