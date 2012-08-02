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
* @file mousecursor.cpp
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/10
*
* Implementation of methods defined in mousecursor.h
*
* This file is part of the DGtal library.
*/

///////////////////////////////////////////////////////////////////////////////
#include "mousecursor.h"


///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// class MouseCursor
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
* Constructor.
*/
DGtal::MouseCursor::MouseCursor() : myGuiOverlay ( 0 ), myCursorContainer ( 0 )
{
  myMaterial = Ogre::MaterialManager::getSingleton().create ( "MouseCursor/default", "General" );
  myCursorContainer = ( Ogre::OverlayContainer* ) Ogre::OverlayManager::getSingletonPtr()->createOverlayElement ( "Panel", "MouseCursor" );
  myCursorContainer->setMaterialName ( myMaterial->getName() );
  myCursorContainer->setPosition ( 0, 0 );

  myGuiOverlay = Ogre::OverlayManager::getSingletonPtr()->create ( "MouseCursor" );
  myGuiOverlay->setZOrder ( 649 );
  myGuiOverlay->add2D ( myCursorContainer );
  myGuiOverlay->show();
}



/**
* Destructor.
*/
DGtal::MouseCursor::~MouseCursor()
{
}


/**
*  Sets the cursor image.
*/
void DGtal::MouseCursor::setImage ( const Ogre::String& filename )
{
  myTexture = Ogre::TextureManager::getSingleton().load ( filename, "General" );

  Ogre::TextureUnitState *pTexState;
  Ogre::Technique * myTech = myMaterial->getTechnique ( 0 );

  if ( myTech->getPass ( 0 )->getNumTextureUnitStates() )
    {
      pTexState = myMaterial->getTechnique ( 0 )->getPass ( 0 )->getTextureUnitState ( 0 );
    }
  else
    {
      pTexState = myMaterial->getTechnique ( 0 )->getPass ( 0 )->createTextureUnitState ( myTexture->getName() );
    }

  pTexState->setTextureAddressingMode ( Ogre::TextureUnitState::TAM_CLAMP );

  myMaterial->getTechnique ( 0 )->getPass ( 0 )->setSceneBlending ( Ogre::SBT_TRANSPARENT_COLOUR );
}





/**
*  Sets the cursor limits by window dimensions.
*/
void DGtal::MouseCursor::setWindowDimensions ( unsigned int width, unsigned int height )
{
  myWindowWidth = ( width > 0 ) ? width : 1;
  myWindowHeight = ( height > 0 ) ? height : 1;


  myCursorContainer->setWidth ( myTexture->getWidth() / myWindowWidth );
  myCursorContainer->setHeight ( myTexture->getHeight() / myWindowHeight );
}

/**
  *  Shows/Hides the cursor
  */
void DGtal::MouseCursor::setVisible ( bool visible )
{
  if ( visible )
    {
      myCursorContainer->show();
    }
  else
    {
      myCursorContainer->hide();
    }
}

/**
  *  Updates the cursor position
  */
void DGtal::MouseCursor::updatePosition ( int x, int y )
{

  Ogre::Real rx = x / myWindowWidth;
  Ogre::Real ry = y / myWindowHeight;
  myCursorContainer->setPosition ( clamp ( rx, 0.0f, 1.0f ), clamp ( ry, 0.0f, 1.0f ) );
}


/**
  *  Avoids going out of the window
  */
Ogre::Real DGtal::MouseCursor::clamp ( Ogre::Real a, Ogre::Real min, Ogre::Real max )
{
  if ( a < min )
    {
      return min;
    }

  if ( a > max )
    {
      return max;
    }

  return a;
}

///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
* Writes/Displays the object on an output stream.
* @param out the output stream where the object is written.
*/
void
DGtal::MouseCursor::selfDisplay ( std::ostream & out ) const
  {
    out << "[MouseCursor]";
  }

/**
* Checks the validity/consistency of the object.
* @return 'true' if the object is valid, 'false' otherwise.
*/
bool
DGtal::MouseCursor::isValid() const
  {
    return true;
  }



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

// //
///////////////////////////////////////////////////////////////////////////////