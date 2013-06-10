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
 * Class for OBJ export
 * This is the implementation of methods defined in Board3D.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/boards/Board3D.h"
#include <limits>
//#include <vector>




///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

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
void DGtal::Board3D::saveOBJ(const string & filename)
{
    unsigned int j; //id of each elements of a list for the .OBJ identification
    std::ofstream out;
    out.open(filename.c_str());

    out << "#  OBJ format"<< std::endl;
    out << "# generated from Board3D from the DGTal library"<< std::endl;
    out << std::endl;

    // TODOLP tests unitaires pour chaque partie
    // OPT changer les unsigned int en size_t
    for(unsigned int i =0; i< myClippingPlaneList.size();   i++)
        trace.info() << "-> ClippingPlane not implemented in Board3D" << std::endl;

    // Draw the shapes

    // myPointSetList
    ostringstream tmpstream; // checking that points exist before creating an object
    for (std::vector<std::vector<pointD3D> >::const_iterator it = myPointSetList.begin();
         it != myPointSetList.end(); it++)
    {
        //OPT const_ iterator en serie au lieu de l etrange parcours
        for (std::vector<pointD3D>::const_iterator s_it = it->begin();
             s_it != it->end(); s_it++)
        {
           // TODO verifier si  un point reste un point
                //(peu importe le mode )
            tmpstream << "v " << s_it->x << " " << s_it->y << " " << s_it->z << std::endl;
        }
    }
    string tmpstr(tmpstream.str());
    if (tmpstr.size() > 0)
           out << "o  myPointSetList" << std::endl<< tmpstr;


    // myLineSetList
    j =0;//id of each LineSetList for the .OBJ identification
    for(unsigned int i=0; i<myLineSetList.size();   i++)
    {
        for (std::vector<lineD3D>::iterator s_it = myLineSetList.at(i).begin();
             s_it != myLineSetList.at(i).end();
               ++s_it)
        {
            out << "o  myLineSetList_" << j << std::endl;
           // OBJ ne connait pas les courbes il faut faire un pave "plat" de cote width
           double wid = s_it->width;

           std::cout << "largeur de ligne : " << wid << std::endl;
           out << "v " << s_it->x1     << " " << s_it->y1      << " " << s_it->z1 << std::endl;
           out << "v " << s_it->x1     << " " << s_it->y1 +wid  << " " << s_it->z1 << std::endl;
           out << "v " << s_it->x1     << " " << s_it->y1      << " " << s_it->z1 -wid << std::endl;
           out << "v " << s_it->x1     << " " << s_it->y1 +wid  << " " << s_it->z1 -wid << std::endl;

           out << "v " << s_it->x2     << " " << s_it->y2      << " " << s_it->z2 << std::endl;
           out << "v " << s_it->x2     << " " << s_it->y2 +wid  << " " << s_it->z2 << std::endl;
           out << "v " << s_it->x2     << " " << s_it->y2      << " " << s_it->z2 -wid << std::endl;
           out << "v " << s_it->x2     << " " << s_it->y2 +wid  << " " << s_it->z2 -wid << std::endl;

           //OPT factoriser les faces des cubes et paves
           out << "f -8 -7 -5 -6 " << std::endl; //cote G
           out << "f -8 -6 -2 -4 " << std::endl; // face
           out << "f -8 -7 -3 -4 " << std::endl; // dessus
           out << "f -7 -5 -1 -3 " << std::endl; // derriere
           out << "f -6 -5 -1 -2 " << std::endl; // dessous
           out << "f -4 -3 -1 -2 " << std::endl; // cote D

            j++;
        }
    }


    // myVoxelSetList
    j  = 0 ; //id of each VoxelSetList for the .OBJ identification
    for(unsigned int i=0; i<myVoxelSetList.size();   i++)
    {
        for (std::vector<voxelD3D>::iterator s_it = myVoxelSetList.at(i).begin();
             s_it != myVoxelSetList.at(i).end();
               ++s_it)
        {
            out << "o  myVoxelSetList_" << j << std::endl;
            // grid mode not implemented yet

            // TODO finir implementer le mode grid et mettre une condition.
             out << "v " << s_it->x << " " << s_it->y << " " << s_it->z<< std::endl ;


             /*
             // mode grid : un cube de coin supérieur gauche (x,y,z) et de largeur width
             double wid = s_it->width;
             out << "v " << s_it->x     << " " << s_it->y       << " " << s_it->z << std::endl;
             out << "v " << s_it->x +wid << " " << s_it->y       << " " << s_it->z << std::endl;
             out << "v " << s_it->x     << " " << s_it->y +wid   << " " << s_it->z << std::endl;
             out << "v " << s_it->x +wid << " " << s_it->y +wid   << " " << s_it->z << std::endl;
             out << "v " << s_it->x     << " " << s_it->y       << " " << s_it->z -wid << std::endl;
             out << "v " << s_it->x +wid << " " << s_it->y       << " " << s_it->z -wid << std::endl;
             out << "v " << s_it->x     << " " << s_it->y +wid   << " " << s_it->z -wid << std::endl;
             out << "v " << s_it->x +wid << " " << s_it->y +wid   << " " << s_it->z -wid << std::endl;
            */
             j ++;
        }
    }

    j =0; //id of each KSSurfelList for the .OBJ identification
    for(unsigned int i=0; i<myQuadList.size();   i++)
        trace.info() << "-> Quad not YET implemented in Board3DTo2D" << std::endl;

    // Drawing all Khalimsky Space Cells

    // KSSurfel (from updateList)

    for (std::vector<quadD3D>::iterator s_it = myKSSurfelList.begin();
         s_it != myKSSurfelList.end();
           ++s_it)
    {
        out << "o  myKSSurfelList_" << j << std::endl;

        out << "v " << s_it->x1     << " " << s_it->y1      << " " << s_it->z1 << std::endl;
        out << "v " << s_it->x2     << " " << s_it->y2      << " " << s_it->z2 << std::endl;
        out << "v " << s_it->x3     << " " << s_it->y3      << " " << s_it->z3 << std::endl;
        out << "v " << s_it->x4     << " " << s_it->y4      << " " << s_it->z4 << std::endl;

        double nx = s_it->nx;
        double ny = s_it->ny;
        double nz = s_it->nz;

        out << "v " << s_it->x1 +nx     << " " << s_it->y1 +ny      << " " << s_it->z1 +nz << std::endl;
        out << "v " << s_it->x2 +nx     << " " << s_it->y2 +ny      << " " << s_it->z2 +nz << std::endl;
        out << "v " << s_it->x3 +nx     << " " << s_it->y3 +ny      << " " << s_it->z3 +nz << std::endl;
        out << "v " << s_it->x4 +nx     << " " << s_it->y4 +ny      << " " << s_it->z4 +nz << std::endl;


        out << "f -8 -7 -5 -6 " << std::endl; //cote G
        out << "f -8 -6 -2 -4 " << std::endl; // face
        out << "f -8 -7 -3 -4 " << std::endl; // dessus
        out << "f -7 -5 -1 -3 " << std::endl; // derriere
        out << "f -6 -5 -1 -2 " << std::endl; // dessous
        out << "f -4 -3 -1 -2 " << std::endl; // cote D
        j++;
    }


    // KSLinel
    for(unsigned int i=0; i< myKSLinelList.size();  i++)
    {
        lineD3D KSline = myKSLinelList.at(i);

        out << "o  myKSLinelList_" << i << std::endl;
        // OBJ dont know how to draw lines, have to make a cuboid with a depth and height of the line's width
        double wid = KSline.width;
        out << "v " << KSline.x1     << " " << KSline.y1      << " " << KSline.z1 << std::endl;
        out << "v " << KSline.x1     << " " << KSline.y1 +wid  << " " << KSline.z1 << std::endl;
        out << "v " << KSline.x1     << " " << KSline.y1      << " " << KSline.z1 -wid << std::endl;
        out << "v " << KSline.x1     << " " << KSline.y1 +wid  << " " << KSline.z1 -wid << std::endl;

        out << "v " << KSline.x2     << " " << KSline.y2      << " " << KSline.z2 << std::endl;
        out << "v " << KSline.x2     << " " << KSline.y2 +wid  << " " << KSline.z2 << std::endl;
        out << "v " << KSline.x2     << " " << KSline.y2      << " " << KSline.z2 -wid << std::endl;
        out << "v " << KSline.x2     << " " << KSline.y2 +wid  << " " << KSline.z2 -wid << std::endl;

        //OPT factoriser les faces des cubes et paves
        out << "f -8 -7 -5 -6 " << std::endl; //cote G
        out << "f -8 -6 -2 -4 " << std::endl; // face
        out << "f -8 -7 -3 -4 " << std::endl; // dessus
        out << "f -7 -5 -1 -3 " << std::endl; // derriere
        out << "f -6 -5 -1 -2 " << std::endl; // dessous
        out << "f -4 -3 -1 -2 " << std::endl; // cote D
    }


    // KSPointel
    if (myKSPointelList.size() >0)
        out << "o  myKSPointelList" << std::endl;

    for(unsigned int i=0; i< myKSPointelList.size();  i++)
    {
        pointD3D KSPoint = myKSPointelList.at(i);

           // TODO verifier si  un point reste un point
                //(peu importe le mode )
            out << "v " << KSPoint.x << " " << KSPoint.y << " " << KSPoint.z << std::endl;
    }


    out.close();
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

