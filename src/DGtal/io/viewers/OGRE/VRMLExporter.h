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
 * @file VRMLExporter.h
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * Liris CNRS
 *
 * @date 2012/08/30
 *
 * Header file for module VRMLExporter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VRMLExporter_RECURSES)
#error Recursive header files inclusion detected in VRMLExporter.h
#else // defined(VRMLExporter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VRMLExporter_RECURSES

#if !defined VRMLExporter_h
/** Prevents repeated inclusion of headers. */
#define VRMLExporter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "OGRE/Ogre.h"
#include <iostream>
#include <fstream>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class VRMLExporter
  /**
   * Description of class 'VRMLExporter' <p>
   * \brief Aim: A vrml exporter for the ogre viewer
   */


  class VRMLExporter 
  {
  public:

    /**
     *  Constructor 
     */
    VRMLExporter();
	      
    /**
     *  Destructor.
     */
    virtual ~VRMLExporter();
	      
	      
    /**
     *  The scene export method
     *  @param aSceneManager a pointer to the scene manager to parse
     *  @param aFileName the name of the file were the scene will be exported
     */      
    void exportToFile(Ogre::SceneManager * aSceneManager,std::string aFileName);



  protected :


    /**
     *  Manages the export of a scene node
     *  @param afile a reference to file were to write
     *  @param aNode a pointer to the node
     */  
    void exportNode(std::ofstream  & afile, Ogre::SceneNode * aNode, int id, int rank);


    /**
     *  Manages the export of a scene node
     *  @param afile a reference to file were to write
     *  @param aNode a pointer to the node
     */  
    void exportLight(std::ofstream  & afile, Ogre::SceneNode * aNode,int rank);
 
  };
}


#include "DGtal/io/viewers/OGRE/VRMLExporter.ih"
#endif /* VRMLExporter_H_ */

#undef VRMLExporter_RECURSES
#endif // else defined(VRMLExporter_RECURSES)
