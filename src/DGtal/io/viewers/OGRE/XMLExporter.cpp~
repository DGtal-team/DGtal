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
 * @file XMLExporter.cpp
 * @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
 * Liris CNRS
 *
 * @date 2012/07/25
 *
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

#include "XMLExporter.h"
#include <iostream>
#include <fstream>
using namespace std;
///////////////////////////////////////////////////////////////////////////////
// class XMLExporter
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

 	    

	
/**     
 *  Constructor 	   
 */	      
DGtal::XMLExporter::XMLExporter()
{

}	      
	      
/**	       
 *  Destructor.	       
 */
	      
DGtal::XMLExporter::~XMLExporter()
{

}	      
	      
/**
 *  Manages the export of a scene node
 *  @param afile a reference to file were to write
 *  @param aNode a pointer to the node
 */  
void DGtal::XMLExporter::exportNode(std::ofstream  & aFile, Ogre::SceneNode * aNode, int id,int rank)
{
  for(int i=0;i<rank;i++)
    aFile <<"\t";
  aFile << "<node> name=\""<<aNode->getName()<<" id=\""<<id<<"\">"<<endl;
  for(int i=0;i<rank;i++)
    aFile <<"\t";
  aFile << "<position x=\""<<aNode->getPosition().x<< 
    " \" y=\""<<aNode->getPosition().y<< 
    "\" z=\""<<aNode->getPosition().z<<" \" />"<<endl;
  for(int i=0;i<rank;i++)
    aFile <<"\t";
  aFile << "<scale x=\""<<aNode->getScale().x<< 
    " \" y=\""<<aNode->getScale().y<< 
    "\" z=\""<<aNode->getScale().z<<" \" />"<<endl;
  int j=0;
  while(j<aNode->numAttachedObjects())
    {
      Ogre::MovableObject * aMovable =aNode->getAttachedObject(j);
      for(int i=0;i<rank;i++)
	aFile <<"\t";
      if(aMovable->getMovableType()=="Entity")
	{
          Ogre::Entity * anEntity =(Ogre::Entity *)aMovable;
	  aFile << "<entity name=\""<<anEntity->getName()<< 
	    " \" meshFile=\""<<anEntity->getMesh()->getName()<< 
	    "\" static=\""<<"false"<<" \" />"<<endl;
	}
      else if(aMovable->getMovableType()=="ManualObject")
	{
          Ogre::ManualObject * aManual =(Ogre::ManualObject *)aMovable;
	  aFile << "<manualobject name=\""<<aManual->getName()<<" \" />"<<endl;
	}
      else if(aMovable->getMovableType()=="Light")
	{
          Ogre::Light * aLight =(Ogre::Light *)aMovable;
	  aFile << "<light name=\""<<aLight->getName()<<" \" />"<<endl;
	}
      j++;
    }
  j=0;
  while(aNode->getChild(j)!=NULL)
    {
      exportNode(aFile,(Ogre::SceneNode *)aNode->getChild(j),j,rank+1);
      j++;
    }

  for(int i=0;i<rank;i++)
    aFile <<"\t";
  aFile << "</node>"<<endl;
}


/**
 *  Manages the export of a scene node
 *  @param afile a reference to file were to write
 *  @param aNode a pointer to the node
 */  
void DGtal::XMLExporter::exportLight(ofstream  & aFile, Ogre::SceneNode * aNode,int rank)
{
}

      
/**	       
 *  The scene export method	      
 *  @param aSceneManager a pointer to the scene manager to parse	       
 *  @param aFileName the name of the file were the scene will be exported	       
 */      
	      
void DGtal::XMLExporter::exportToFile(Ogre::SceneManager * aSceneManager, std::string aFileName)
{
  assert(aSceneManager);
  ofstream aFile;
  std::string aName= aFileName+".scene";
  aFile.open (aName.c_str());
  aFile << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>"<<endl;
  aFile << "<scene formatVersion=\"\">"<<endl;
  Ogre::SceneNode  *aRootSceneNode =aSceneManager->getRootSceneNode ();
  int i=0;
  aFile << "\t<nodes>"<<endl;
  while(aRootSceneNode->getChild(i)!=NULL)
    {
      exportNode(aFile,(Ogre::SceneNode *)aRootSceneNode->getChild(i),i,2);
      i++;
    }
  aFile << "\t</nodes>"<<endl;
  aFile << "</scene>"<<endl;
  aFile.close();
  std::cout<< "Scene exported to "<<aFileName<<".scene"<<endl;
}
