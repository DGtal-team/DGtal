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

/**
* @file TextZone.cpp
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/12
*
* Implementation of methods defined in TextZone.h
*
* This file is part of the DGtal library.
*/

///////////////////////////////////////////////////////////////////////////////
#include "TextZone.h"


///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// class TextZone
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
* Constructor.
*/
DGtal::TextZone::TextZone(std::string aContent) : myPanel ( 0 ), myText ( 0 )
{
  using namespace Ogre;
  OverlayManager& overlayManager = OverlayManager::getSingleton();
  
  // Create a panel
   myPanel = static_cast<OverlayContainer*>(
      overlayManager.createOverlayElement("Panel",aContent ));
  myPanel->setMetricsMode(Ogre::GMM_PIXELS);
  myPanel->setPosition(10, 10);
  myPanel->setDimensions(100, 100);
  
  
  // Create a text area
  myText = static_cast<TextAreaOverlayElement*>(
      overlayManager.createOverlayElement("TextArea", aContent + (char)1));
  myText->setMetricsMode(Ogre::GMM_PIXELS);
  myText->setPosition(0, 0);
  myText->setDimensions(100, 100);
  myText->setCaption(aContent);
  myText->setCharHeight(36);
  myText->setFontName("MyFont");
  myText->setColourBottom(ColourValue(0.95, 0.6, 1.0));
  myText->setColourTop(ColourValue(0.95, 0.6, 1.0));
  
  // Create an overlay, and add the panel
  Overlay* overlay = overlayManager.create(aContent+(char)2);
  overlay->add2D(myPanel);
  
  // Add the text area to the panel
  myPanel->addChild(myText);
  
  // Show the overlay
  overlay->show();
}



/**
* Destructor.
*/
DGtal::TextZone::~TextZone()
{
}


/**
  *  Shows/Hides the cursor
  */
void DGtal::TextZone::setVisible ( bool visible )
{
  if ( visible )
    {
      myPanel->show();
    }
  else
    {
      myPanel->hide();
    }
}

/**
*  Flips the cursor visibility
*/
void DGtal::TextZone::flipVisibility ( )
{
	if(myPanel->isVisible())
	{
   		myPanel->hide();
	}
	else
	{
	      myPanel->show();
	}
}


/**
  *  Changes the text position
  */
void DGtal::TextZone::setPosition ( int x, int y )
{
  myPanel->setPosition(x, y);
  myPanel->_update();
}

/**
  *  Changes the text position
  */
void DGtal::TextZone::setCaption(std::string newCaption)
{
  myText->setCaption(newCaption);
  myText->_update();
}


/**
  *  Changes the text size
  */
void DGtal::TextZone::setSize(int aSize)
{
    myText->setCharHeight(aSize);
}

///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
* Writes/Displays the object on an output stream.
* @param out the output stream where the object is written.
*/
void
DGtal::TextZone::selfDisplay ( std::ostream & out ) const
  {
    out << "[TextZone]";
  }

/**
* Checks the validity/consistency of the object.
* @return 'true' if the object is valid, 'false' otherwise.
*/
bool
DGtal::TextZone::isValid() const
  {
    return true;
  }



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

// //
///////////////////////////////////////////////////////////////////////////////
