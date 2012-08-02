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
 * @file XMLExporter.h
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * Liris CNRS
 *
 * @date 2012/07/25
 *
 * Header file for module XMLExporter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(XMLExporter_RECURSES)
#error Recursive header files inclusion detected in XMLExporter.h
#else // defined(XMLExporter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define XMLExporter_RECURSES

#if !defined XMLExporter_h
/** Prevents repeated inclusion of headers. */
#define XMLExporter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "OGRE/Ogre.h"
#include <iostream>
#include <fstream>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class XMLExporter
  /**
   * Description of class 'XMLExporter' <p>
   * \brief Aim: An Xml exporter for the ogre viewer
   */


  class XMLExporter 
  {
  public:

    /**
     *  Constructor 
     */
    XMLExporter();
	      
    /**
     *  Destructor.
     */
    virtual ~XMLExporter();
	      
	      
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

#endif /* XMLExporter_H_ */

#undef XMLExporter_RECURSES
#endif // else defined(XMLExporter_RECURSES)
