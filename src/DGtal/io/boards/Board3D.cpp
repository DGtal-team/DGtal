/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; withoutOBJ even the implied warranty of
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
 * @param outOBJ the output stream where the object is written.
 */
void
DGtal::Board3D::selfDisplay ( std::ostream & outOBJ ) const
{
  outOBJ << "[Board3D]";
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
  size_t k, j; //id of each elements and sub elements of a list for the .OBJ identification
  double red, green, blue , transp; // colors of the elements
  double sizePixel = 0.003 ;  // size of one pixel (depending on resolution )
  std::ofstream outOBJ; //OBJ file where to write
  std::ofstream outMTL; //MTL file where to write

  //OPT parametrer la resolution ?
  //WARNING quand deux obj ont le meme nom ils fusionnent (danger deux sets differents fusionnes)
  //TODOLP transferer le cpp dans ih

  //TODOLP mettre des ASSERT
  //TODO faire le .MTL

  string noExt(filename.substr(0, filename.find_first_of("."))); //the filename withoutOBJ any extention
  stringstream strOBJ;
  stringstream strMTL;
  strOBJ << noExt << ".obj";
  strMTL << noExt << ".mtl";

  //OPT identifier les memes materiaux (MTL)
  //Kd R G B    colors(MTL)
  // d 0-1    Transparency(MTL)

  //usemtl Material     (OBJ)


  outOBJ.open(strOBJ.str().c_str());
  outOBJ << "#  OBJ format"<< std::endl;
  outOBJ << "# generated from Board3D from the DGTal library"<< std::endl;
  //outOBJ << "mtllib " <<  strMTL.str() << std::endl;
  outOBJ << std::endl;

  outMTL.open(strMTL.str().c_str());
  outMTL << "#  MTL format"<< std::endl;
  outMTL << "# generated from Board3D from the DGTal library"<< std::endl;

  // TODOLP tests unitaires pour chaque partie

  //myClippingPlaneList++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //std::vector< clippingPlaneD3D >::const_iterator
  {
    if(myClippingPlaneList.size()> 0)
    {
      trace.info() << "-> Quad not YET implemented in Board3D, number of it: "
                   << myClippingPlaneList.size() << std::endl;
    }
  }

  // Draw the shapes -----------------------------------------------------------------------

  // myPointSetList++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    k=0;//id of each PointSetList for the .OBJ identification
    for (std::vector<std::vector<pointD3D> >::const_iterator it = myPointSetList.begin();
         it != myPointSetList.end(); it++)
    {
      ostringstream tmpStream; // checking that points exist before creating an object
      for (std::vector<pointD3D>::const_iterator s_it = it->begin();
           s_it != it->end(); s_it++)
      {
        tmpStream << "v " << s_it->x << " " << s_it->y << " " << s_it->z << std::endl;
        tmpStream << "f " << "-1" << " " << "-1" << " "<< "-1" << std::endl;
        /*
        if (s_it== it->begin())
        {
          red= ((double)s_it->R) /255.0;
          green =  ((double)s_it->G) /255.0;
          blue =  ((double)s_it->B) /255.0;
          transp =  ((double)s_it->T) /255.0;  // NOTE note sure
        }*/
      }

      if (tmpStream.str().size() > 0)
      {
        stringstream name;
        name << myPointSetNameList.at(k);
        if ( name.str() == "")
        {
          name << "myPointSetList_" << k ;
        }
        //NOTE up until now one object = one material
        stringstream matName;
        matName << name.str() << "MAT";

        outOBJ << "o  " << name.str() << std::endl;
        //outOBJ << "usemtl " << matName.str() << std::endl;
        outOBJ << tmpStream.str();

        /* outMTL << "newmtl " << matName.str() <<std::endl;
        outMTL << "kd " << red << green << blue << std::endl;
        outMTL << "d "<< transp << std::endl; */

        //TODO a enlever
        std::cout << "point " << k <<std::endl;
      }
      k++;
    }
  }

  // myLineSetList+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    j =0;
    k=0;//id of each LineSetList for the .OBJ identification
    for(std::vector<std::vector<lineD3D> >::const_iterator it =myLineSetList.begin();
        it!= myLineSetList.end();   it++)
    {
      ostringstream tmpStream;
      for (std::vector<lineD3D>::const_iterator s_it = it->begin();
           s_it != it->end();++s_it)
      {
        // OBJ dont know how to draw lines, have to make a cuboid with a depth and height of a pixel width
        tmpStream << "v " << s_it->x1     << " " << s_it->y1 -sizePixel << " " << s_it->z1 << std::endl;
        tmpStream << "v " << s_it->x1     << " " << s_it->y1 +sizePixel << " " << s_it->z1 << std::endl;
        tmpStream << "v " << s_it->x2     << " " << s_it->y2 +sizePixel << " " << s_it->z2 << std::endl;
        tmpStream << "v " << s_it->x2     << " " << s_it->y2 -sizePixel << " " << s_it->z2 << std::endl;

        tmpStream << "v " << s_it->x1     << " " << s_it->y1     << " " << s_it->z1 -sizePixel << std::endl;
        tmpStream << "v " << s_it->x1     << " " << s_it->y1     << " " << s_it->z1 +sizePixel<< std::endl;
        tmpStream << "v " << s_it->x2     << " " << s_it->y2     << " " << s_it->z2 +sizePixel<< std::endl;
        tmpStream << "v " << s_it->x2     << " " << s_it->y2     << " " << s_it->z2 -sizePixel<< std::endl;
        tmpStream << "f " << "-8" << " " << "-7" << " " << "-5"<< " " << "-6" << std::endl;//left
        tmpStream << "f " << "-8" << " " << "-6" << " " << "-2"<< " " << "-4"<< std::endl;//front
        tmpStream << "f " << "-8" << " " << "-7" << " " << "-3"<< " " << "-4"<< std::endl;//up
        tmpStream << "f " << "-7" << " " << "-5" << " " << "-1"<< " " << "-3"<< std::endl;//back
        tmpStream << "f " << "-6" << " " << "-5" << " " << "-1"<< " " << "-2"<< std::endl;//down
        tmpStream << "f " << "-4" << " " << "-3" << " " << "-1"<< " " << "-2"<< std::endl;//right

        j++;
      }
      if (tmpStream.str() != "")
      {
        stringstream name;
        name << myLineSetNameList.at(k);
        if ( name.str()== "")
        {
          name << "myLineSetList_" << k ;
        }
        stringstream matName;
        matName << name.str() << "MAT";

        outOBJ << "o  " << name.str() << std::endl;
        //outOBJ << "usemtl " << matName.str() << std::endl;
        outOBJ << tmpStream.str();

        /* outMTL << "newmtl " << matName.str() <<std::endl;
        outMTL << "kd " << red << green << blue << std::endl;
        outMTL << "d "<< transp << std::endl; */
      }

      k++;
    }
  }

  // myVoxelSetList++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    j  = 0 ; //id of each Voxel sub list for the .OBJ identification
    k = 0; // id of each list
    for(std::vector<std::vector<voxelD3D> >::const_iterator it =myVoxelSetList.begin();
        it != myVoxelSetList.end();   it++)
    {
      ostringstream tmpStream;
      for (std::vector<voxelD3D>::const_iterator s_it = it->begin();
           s_it != it->end(); ++s_it)
      {
        // this version is  one cube with (x,y,z) the center of it and wid its distance between it and its faces
        double wid = s_it->width;
        tmpStream  << "v " << s_it->x -wid    << " " << s_it->y -wid  << " " << s_it->z +wid << std::endl;
        tmpStream  << "v " << s_it->x +wid    << " " << s_it->y -wid  << " " << s_it->z +wid << std::endl;
        tmpStream  << "v " << s_it->x -wid    << " " << s_it->y -wid  << " " << s_it->z -wid << std::endl;
        tmpStream  << "v " << s_it->x +wid    << " " << s_it->y -wid  << " " << s_it->z -wid << std::endl;
        tmpStream  << "v " << s_it->x -wid    << " " << s_it->y +wid  << " " << s_it->z +wid << std::endl;
        tmpStream  << "v " << s_it->x +wid    << " " << s_it->y +wid  << " " << s_it->z +wid << std::endl;
        tmpStream  << "v " << s_it->x -wid    << " " << s_it->y +wid  << " " << s_it->z -wid << std::endl;
        tmpStream  << "v " << s_it->x +wid    << " " << s_it->y +wid  << " " << s_it->z -wid << std::endl;

        tmpStream << "f " << "-8" << " " << "-7" << " " << "-5"<< " " << "-6" << std::endl;//left
        tmpStream << "f " << "-8" << " " << "-6" << " " << "-2"<< " " << "-4"<< std::endl;//front
        tmpStream << "f " << "-8" << " " << "-7" << " " << "-3"<< " " << "-4"<< std::endl;//up
        tmpStream << "f " << "-7" << " " << "-5" << " " << "-1"<< " " << "-3"<< std::endl;//back
        tmpStream << "f " << "-6" << " " << "-5" << " " << "-1"<< " " << "-2"<< std::endl;//down
        tmpStream << "f " << "-4" << " " << "-3" << " " << "-1"<< " " << "-2"<< std::endl;//right

        /*
                   //obsolete
                    // this version is  one cube with (x,y,z) the upper left vertex and wid its width and height
                    double wid = s_it->width;
                    tmpStream  << "v " << s_it->x     << " " << s_it->y       << " " << s_it->z << std::endl;
                    tmpStream  << "v " << s_it->x +wid << " " << s_it->y       << " " << s_it->z << std::endl;
                    tmpStream  << "v " << s_it->x     << " " << s_it->y +wid   << " " << s_it->z << std::endl;
                    tmpStream  << "v " << s_it->x +wid << " " << s_it->y +wid   << " " << s_it->z << std::endl;
                    tmpStream  << "v " << s_it->x     << " " << s_it->y       << " " << s_it->z -wid << std::endl;
                    tmpStream  << "v " << s_it->x +wid << " " << s_it->y       << " " << s_it->z -wid << std::endl;
                    tmpStream  << "v " << s_it->x     << " " << s_it->y +wid   << " " << s_it->z -wid << std::endl;
                    tmpStream  << "v " << s_it->x +wid << " " << s_it->y +wid   << " " << s_it->z -wid << std::endl;

                    tmpStream << "f " << "-8" << " " << "-7" << " " << "-5"<< " " << "-6" << std::endl;//left
                    tmpStream << "f " << "-8" << " " << "-6" << " " << "-2"<< " " << "-4"<< std::endl;//front
                    tmpStream << "f " << "-8" << " " << "-7" << " " << "-3"<< " " << "-4"<< std::endl;//up
                    tmpStream << "f " << "-7" << " " << "-5" << " " << "-1"<< " " << "-3"<< std::endl;//back
                    tmpStream << "f " << "-6" << " " << "-5" << " " << "-1"<< " " << "-2"<< std::endl;//down
                    tmpStream << "f " << "-4" << " " << "-3" << " " << "-1"<< " " << "-2"<< std::endl;//right
                    */

        j ++;
      }

      if (tmpStream.str() != "")
      {
        stringstream name;
        name << myVoxelSetNameList.at(k);
        if ( name.str() == "")
        {
          name << "myVoxelSetList_" << k ;
        }
        stringstream matName;
        matName << name.str() << "MAT";

        outOBJ << "o  " << name.str() << std::endl;
        //outOBJ << "usemtl " << matName.str() << std::endl;
        outOBJ << tmpStream.str();

        /* outMTL << "newmtl " << matName.str() <<std::endl;
        outMTL << "kd " << red << green << blue << std::endl;
        outMTL << "d "<< transp << std::endl; */
      }
      k++;
    }
  }

  // myQuadList++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //std::vector<quadD3D>::const_iterator
  if(myQuadList.size()> 0)
  {
    trace.info() << "-> Quad not YET implemented in Board3D, number of it: "
                 << myQuadList.size() << std::endl;
  }


  // Drawing all Khalimsky Space Cells --------------------------------------------------------------------

  //TODO chercher comment tester
  // KSSurfel (from updateList)+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    j=0;
    k=0;
    ostringstream tmpStream;

    for (std::vector<quadD3D>::iterator s_it = myKSSurfelList.begin();
         s_it != myKSSurfelList.end();
         ++s_it)
    {
      tmpStream << "v " << s_it->x1     << " " << s_it->y1      << " " << s_it->z1 << std::endl;
      tmpStream << "v " << s_it->x2     << " " << s_it->y2      << " " << s_it->z2 << std::endl;
      tmpStream << "v " << s_it->x3     << " " << s_it->y3      << " " << s_it->z3 << std::endl;
      tmpStream << "v " << s_it->x4     << " " << s_it->y4      << " " << s_it->z4 << std::endl;

      double nx = s_it->nx;
      double ny = s_it->ny;
      double nz = s_it->nz;

      tmpStream << "v " << s_it->x1 +nx     << " " << s_it->y1 +ny      << " " << s_it->z1 +nz << std::endl;
      tmpStream << "v " << s_it->x2 +nx     << " " << s_it->y2 +ny      << " " << s_it->z2 +nz << std::endl;
      tmpStream << "v " << s_it->x3 +nx     << " " << s_it->y3 +ny      << " " << s_it->z3 +nz << std::endl;
      tmpStream << "v " << s_it->x4 +nx     << " " << s_it->y4 +ny      << " " << s_it->z4 +nz << std::endl;

      tmpStream << "f " << "-8" << " " << "-7" << " " << "-5"<< " " << "-6" << std::endl;//left
      tmpStream << "f " << "-8" << " " << "-6" << " " << "-2"<< " " << "-4"<< std::endl;//front
      tmpStream << "f " << "-8" << " " << "-7" << " " << "-3"<< " " << "-4"<< std::endl;//up
      tmpStream << "f " << "-7" << " " << "-5" << " " << "-1"<< " " << "-3"<< std::endl;//back
      tmpStream << "f " << "-6" << " " << "-5" << " " << "-1"<< " " << "-2"<< std::endl;//down
      tmpStream << "f " << "-4" << " " << "-3" << " " << "-1"<< " " << "-2"<< std::endl;//right
      j++;
    }

    if (tmpStream.str() != "")
    {
      string name= myKSSurfelNameList.at(k);
      if ( name== "")
      {
        name = "myKSSurfelList" ;
      }
      outOBJ << "o  " << name << std::endl;
      outOBJ << tmpStream.str() ;
      outOBJ << tmpStream.str();
    }
  }


  //TODO chercher comment tester
  // KSLinel ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    j=0;
    k=0;
    ostringstream tmpStream;

    for(std::vector<lineD3D>::const_iterator it = myKSLinelList.begin(); it != myKSLinelList.end();  it++)
    {
      double wid = it->width;
      tmpStream << "v " << it->x1     << " " << it->y1      << " " << it->z1 << std::endl;
      tmpStream << "v " << it->x1     << " " << it->y1 +wid  << " " << it->z1 << std::endl;
      tmpStream << "v " << it->x1     << " " << it->y1      << " " << it->z1 -wid << std::endl;
      tmpStream << "v " << it->x1     << " " << it->y1 +wid  << " " << it->z1 -wid << std::endl;

      tmpStream << "v " << it->x2     << " " << it->y2      << " " << it->z2 << std::endl;
      tmpStream << "v " << it->x2     << " " << it->y2 +wid  << " " << it->z2 << std::endl;
      tmpStream << "v " << it->x2     << " " << it->y2      << " " << it->z2 -wid << std::endl;
      tmpStream << "v " << it->x2     << " " << it->y2 +wid  << " " << it->z2 -wid << std::endl;

      //OPT factoriser les faces des cubes et paves
      tmpStream << "f " << "-8" << " " << "-7" << " " << "-5"<< " " << "-6" << std::endl;//left
      tmpStream << "f " << "-8" << " " << "-6" << " " << "-2"<< " " << "-4"<< std::endl;//front
      tmpStream << "f " << "-8" << " " << "-7" << " " << "-3"<< " " << "-4"<< std::endl;//up
      tmpStream << "f " << "-7" << " " << "-5" << " " << "-1"<< " " << "-3"<< std::endl;//back
      tmpStream << "f " << "-6" << " " << "-5" << " " << "-1"<< " " << "-2"<< std::endl;//down
      tmpStream << "f " << "-4" << " " << "-3" << " " << "-1"<< " " << "-2"<< std::endl;//right

      j++;
    }

    if (tmpStream.str() != "")
    {
      string name= myKSLinelNameList.at(k);
      if ( name== "")
      {
        name = "myKSLinelList" ;
      }
      outOBJ << "o  " << name << std::endl;
      outOBJ << tmpStream.str() ;
      outOBJ << tmpStream.str();
    }

  }

  //TODO chercher comment tester
  // KSPointel++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  {
    k=0;
    ostringstream tmpStream;
    for(std::vector<pointD3D>::const_iterator it= myKSPointelList.begin();
        it != myKSPointelList.end();  it++)
    {
      outOBJ << "v " << it->x << " " << it->y << " " << it->z << std::endl;
    }

    if (tmpStream.str() != "")
    {
      string name= myKSPointelNameList.at(k);
      if ( name== "")
      {
        name = "myKSPointelList" ;
      }
      outOBJ << "o  " << name << std::endl;
      outOBJ << tmpStream.str() ;
    }
  }

  outOBJ.close();
  outMTL.close();
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

  /*
     // what is the meaning of this part ?
    createNewVoxelList(true);
    std::vector<voxelD3D> aKSVoxelList;
    */

  myDefaultColor= DGtal::Color(255, 255, 255);

  myModes["Board3D"]="SolidMode";

  //OPT faire une plus belle presentation de init

  string nameLineSet;
  myLineSetNameList.push_back(nameLineSet);

  string namePointSet;
  myPointSetNameList.push_back(namePointSet);

  string nameClippingPlane;
  myClippingPlaneNameList.push_back(nameClippingPlane);

  string nameKSSurfel;
  myKSSurfelNameList.push_back(nameKSSurfel);

  string nameKSPointel;
  myKSPointelNameList.push_back(nameKSPointel);

  string nameKSLinel;
  myKSLinelNameList.push_back(nameKSLinel);

  string nameQuad;
  myQuadNameList.push_back(nameQuad);

  string nameTriangle;
  myTriangleNameList.push_back(nameTriangle);

  string namePolygon;
  myPolygonNameList.push_back(namePolygon);
}

