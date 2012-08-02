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
* @file Representation.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* INSTITUTION
*
* @date 2012/06/15
*
* Header file for module Representation.cpp
*
* This file is part of the DGtal library.
*/

#if defined(OgreEntity_RECURSES)
#error Recursive header files inclusion detected in Representation.h
#else // defined(OgreEntity_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OgreEntity_RECURSES

#if !defined OgreEntity_h
/** Prevents repeated inclusion of headers. */
#define OgreEntity_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "OGRE/Ogre.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class Representation
/**
* Description of class 'ElementaryDiscreteObject' <p>
* \brief Aim: It's a DGtal primitive
* 
*/


class DGtalNode;


class Representation 
{
    // ----------------------- Standard services ------------------------------
public:

 /**
* Constructor for a real Representation
*/
  Representation( Ogre::SceneManager * aSceneMgr, Ogre::MovableObject * aMovableObject,
  Ogre::SceneNode *aSceneNode, std::string aName,std::string aMovableObjectType);
  
  
/**
* Constructor  for a virtual representation
*/
  Representation( Ogre::SceneManager * aSceneMgr,Ogre::SceneNode *aSceneNode, std::string aName);
  
 
  
    /**
* Destructor.
*/
    ~Representation();

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


   /**
* Return the name of the Representation
*
*/

std::string getName();

/**
* Return the name of the Representation
*
*/
std::string getEntityName();


/**
* Returns a pointer on the Node
*
*/
Ogre::SceneNode * getNode();


/**
* Changes the father of this Representation
*
*/
void  setAttachedParent(DGtalNode * aParent);




/**
* Returns a pointer on the associated DGtal Object
*
*/
DGtalNode * getAttachedParent();



/**
* Set this Representation as a selected one
*
*/
void select();


/**
* Set this Representation as in a selected group
*
*/
void groupSelect();



/**
* Tells if the representation is virtual, is means that is has or not an associated entity 
*
*/
bool isVirtual();


/**
* Tells if the representation is virtual, is means that is has or not an associated entity 
*
*/
void clear();



/**
* Set this entity as a unselected
*
*/
void unselect();

/**
 *  Sets the  material name to apply
 */
void setMaterialName(std::string aMaterial);

/**
 *  Sets the  material to apply
 */
void setMaterial(Ogre::MaterialPtr  aMaterial);

/**
 *  Returns the material 
 */
Ogre::MaterialPtr getMaterial();


/**
 *  Returns the material name
 */
std::string getMaterialName();

/**
  *  
  */
void more( );


/**
  *  
  */
void less( );


/**
*  
*/
void moreTransparency( );


/**
* 
*/
void lessTransparency( );
      
/**
 *  Returns the material List
 */
std::string getMaterials();


    // ------------------------- Protected Datas ------------------------------
private:
    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

    /**
* Constructor.
* Forbidden by default (protected to avoid g++ warnings).
*/
    Representation();

private:

    /**
* Copy constructor.
* @param other the object to clone.
* Forbidden by default.
*/
    Representation ( const Representation & other );

    /**
* Assignment.
* @param other the object to copy.
* @return a reference on 'this'.
* Forbidden by default.
*/
    Representation & operator= ( const Representation & other );

    // ------------------------- Internals ------------------------------------
private:

  
  //---------------------------- Attributes ------------------------------------
protected :
  Ogre::MovableObject * myMovableObject;
  std::string myMovableObjectType;
  Ogre::SceneNode *mySceneNode;
  std::string myName;
  DGtalNode * myParent;
  bool mVirtual;
  Ogre::SceneManager * mySceneMgr;
  std::string myMaterialName;
  Ogre::MaterialPtr  myMaterial;
  int myScale;

}; // end of class Representation


/**
* Overloads 'operator<<' for displaying objects of class 'Representation'.
* @param out the output stream where the object is written.
* @param object the object of class 'Representation' to write.
* @return the output stream after the writing.
*/
std::ostream&
operator<< ( std::ostream & out, const Representation & object );


} // namespace YYY


///////////////////////////////////////////////////////////////////////////////


#include "DGtalNode.h"
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "Representation.ih"
#endif


// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Representation_h

#undef Representation_RECURSES
#endif // else defined(Representation_RECURSES)
    
    
