/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file   Board3D.cpp
 * @author Aline Martin
 * @date   vendredi 7 juin 2013
 * 
 * @brief
 *
 * This is the implementation of methods defined in Board3D.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/boards/Board3D.h"
#include <limits>




///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class Board3D
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/*!
 * \brief Constructor.
 */
DGtal::Board3D::Board3D()
{
  init();
}

///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::Board3D::selfDisplay ( std::ostream & out ) const
{
    out << "[Board3D]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::Board3D::isValid() const
{
    return true;
}


/**
 * Save a OBJ image.
 * @param filename filename of the image to save.
 */
void saveObj(const char *filename)
{
    for(unsigned int i =0; i< myClippingPlaneList.size(); i++)
      trace.info() << "-> ClippingPlane not implemented in Board3D" << std::endl;
}

/*!
 * \brief init function (should be in Constructor).
 */
void
DGtal::Board3D::init()
{
  createNewVoxelList(true);
  
  vector<lineD3D> listeLine;
  myLineSetList.push_back(listeLine);
  
  vector<pointD3D> listePoint;
  myPointSetList.push_back(listePoint);
  
  myCurrentFillColor = DGtal::Color (220, 220, 220);
  myCurrentLineColor = DGtal::Color (22, 22, 222, 50);
  
  /*createNewVoxelList(true);
  std::vector<voxelD3D> aKSVoxelList;*/
  
  myDefaultColor= DGtal::Color(255, 255, 255);

  myModes["Board3D"]="SolidMode";
}

