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
 * @file Viewer3D.cpp
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Implementation of methods defined in Viewer3D.h
 *
 * This file is part of the DGtal library.
 */

///all of this have been moved in viewer 3D.ih
/*
///////////////////////////////////////////////////////////////////////////////
#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
#include <GL/gl.h>
#include "DGtal/io/viewers/windows/GL/glext.h"
#endif

#include "DGtal/io/viewers/Viewer3D.h"


#include <limits>
#include <QColor>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace qglviewer;


///////////////////////////////////////////////////////////////////////////////
// class Viewer3D
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :





///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.


template< typename S, typename KS>
inline
void
DGtal::Viewer3D<S, KS>::selfDisplay ( std::ostream & out ) const
{
  out << "[Viewer3D]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.

template< typename S, typename KS>
inline
bool
DGtal::Viewer3D<S, KS>::isValid() const
{
  return true;
}





#if defined( max )
#undef max
#define _HAS_MSVC_MAX_ true
#endif

#if defined( min )
#undef min
#define _HAS_MSVC_MIN_ true
#endif

template< typename S, typename KS>
inline
void
DGtal::Viewer3D<S, KS>::init()
{
  Viewer3D<S, KS>::myMeshDefaultLineWidth=10.0;
  myNbListe=0;
  myViewWire=true;
  myIsDoubleFaceRendering=true;
  Viewer3D<S, KS>::createNewCubeList ( true );
  vector<typename Viewer3D<S, KS>::lineD3D> listeLine;
  Viewer3D<S, KS>::myLineSetList.push_back ( listeLine );
  vector<typename Viewer3D<S, KS>::ballD3D> listeBall;
  Viewer3D<S, KS>::myBallSetList.push_back ( listeBall );
  Viewer3D<S, KS>::myCurrentFillColor = Color ( 220, 220, 220 );
  Viewer3D<S, KS>::myCurrentLineColor = Color ( 22, 22, 222, 50 );
  myDefaultBackgroundColor = Color ( backgroundColor().red(), backgroundColor().green(),
                                     backgroundColor().blue() );
  myIsBackgroundDefault=true;
   Viewer3D<S, KS>::myBoundingPtLow[0]=-10.0;//numeric_limits<double>::max( );
   Viewer3D<S, KS>::myBoundingPtLow[1]=-10.0;//numeric_limits<double>::max( );
   Viewer3D<S, KS>::myBoundingPtLow[2]=-10.0;//numeric_limits<double>::max( );

   Viewer3D<S, KS>::myBoundingPtUp[0]=-10.0;//numeric_limits<double>::min( );
   Viewer3D<S, KS>::myBoundingPtUp[1]=-10.0;//numeric_limits<double>::min( );
  Viewer3D<S, KS>:: myBoundingPtUp[2]=-10.0;//numeric_limits<double>::min( );
   Viewer3D<S, KS>::createNewCubeList ( true );
  typename std::vector< typename Viewer3D<S, KS>::cubeD3D>  aKSCubeList;

  Viewer3D<S, KS>::myCurrentfShiftVisuSurfelPrisms=0.0;
  Viewer3D<S, KS>::myDefaultColor= Color ( 255, 255, 255 );
  camera()->showEntireScene();
  setKeyDescription ( Qt::Key_E, "Export the current display into OFF file (just Cube, surfel and SurfelPrism for now)." );
  setKeyDescription ( Qt::Key_W, "Switch display with and without wired view of triangle and quad faces." );
  setKeyDescription ( Qt::Key_T, "Sort elements for display improvements." );
  setKeyDescription ( Qt::Key_L, "Load last visualisation settings." );
  setKeyDescription ( Qt::Key_B, "Switch background color with White/Black colors." );
  setKeyDescription ( Qt::Key_C, "Show camera informations." );
  setKeyDescription ( Qt::Key_R, "Reset default scale for 3 axes to 1.0f." );
  setKeyDescription ( Qt::Key_D, "Enable/Disable the two side face rendering." );



  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);


  setMouseBindingDescription ( Qt::ShiftModifier+Qt::RightButton, "Delete the mouse selected list." );
  setManipulatedFrame ( new ManipulatedFrame() );


}

#if defined( _HAS_MSVC_MAX_ )
#define max(A,B) ((A)>(B)?(A):(B))
#endif

#if defined( _HAS_MSVC_MIN_ )
#define min(A,B) ((A)<(B)?(A):(B))
#endif


template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::sortSurfelFromCamera()
{
  compFarthestVoxelFromCamera comp;
  comp.posCam= camera()->position();
  for ( unsigned int i=0; i< Viewer3D<S, KS>::myCubeSetList.size(); i++ )
  {
    sort (  Viewer3D<S, KS>::myCubeSetList.at ( i ).begin(),  Viewer3D<S, KS>::myCubeSetList.at ( i ).end(), comp );
  }
  compFarthestSurfelFromCamera compSurf;
  DGtal::trace.info() << "sort surfel size" <<  Viewer3D<S, KS>::mySurfelPrismList.size() << std::endl;
  sort (  Viewer3D<S, KS>::mySurfelPrismList.begin(),  Viewer3D<S, KS>::mySurfelPrismList.end(), compSurf );

}

template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::sortTriangleFromCamera()
{
  compFarthestTriangleFromCamera comp;
  comp.posCam= camera()->position();
  for (typename std::vector<std::vector< typename Viewer3D<S, KS>::triangleD3D> >::iterator it =  Viewer3D<S, KS>::myTriangleSetList.begin(); it !=  Viewer3D<S, KS>::myTriangleSetList.end(); it++)
  {
    DGtal::trace.info() << "sort triangle size" << it->size() << std::endl;
    sort ( it->begin(), it->end(), comp );
  }

}



template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::sortQuadFromCamera()
{
  compFarthestSurfelFromCamera comp;
  comp.posCam= camera()->position();

  for (typename std::vector<std::vector< typename Viewer3D<S, KS>::quadD3D> >::iterator it =  Viewer3D<S, KS>::myQuadSetList.begin(); it !=  Viewer3D<S, KS>::myQuadSetList.end(); it++)
  {
    DGtal::trace.info() << "sort quad size" << it->size() << std::endl;
    sort ( it->begin(), it->end(), comp );
  }

}


template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::sortPolygonFromCamera()
{
  compFarthestPolygonFromCamera comp;
  comp.posCam= camera()->position();

  for (typename std::vector<std::vector< typename Viewer3D<S, KS>::polygonD3D> >::iterator it =  Viewer3D<S, KS>::myPolygonSetList.begin(); it != Viewer3D<S, KS>:: myPolygonSetList.end(); it++)
  {
    DGtal::trace.info() << "sort polygon size" << it->size() << std::endl;
    sort ( it->begin(), it->end(), comp );
  }

}




template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::postSelection ( const QPoint& point )
{
  camera()->convertClickToLine ( point, myOrig, myDir );
  bool found;
  this->myPosSelector= point;
  mySelectedPoint = camera()->pointUnderPixel ( point, found );
  if ( found )
  {
    DGtal::trace.info() << "Element of liste= " << selectedName() << "selected" << endl;
    if ( selectedName() !=-1 )
    {
      unsigned int id = abs ( selectedName()-1 );
      if ( id<  Viewer3D<S, KS>::myCubeSetList.size() )
      {
        DGtal::trace.info() << "deleting list="<< id<<endl;
         Viewer3D<S, KS>::myCubeSetList.erase ( Viewer3D<S, KS>::myCubeSetList.begin() +id );
        updateList ( false );
      }
      else if ( id<  Viewer3D<S, KS>::myCubeSetList.size() + Viewer3D<S, KS>::myLineSetList.size() )
      {
         Viewer3D<S, KS>::myLineSetList.erase (  Viewer3D<S, KS>::myLineSetList.begin() + ( id- Viewer3D<S, KS>::myCubeSetList.size() ) );
        updateList ( false );
      }
      else if ( id<  Viewer3D<S, KS>::myBallSetList.size() + Viewer3D<S, KS>::myLineSetList.size() + Viewer3D<S, KS>::myCubeSetList.size() )
      {
         Viewer3D<S, KS>::myBallSetList.erase (  Viewer3D<S, KS>::myBallSetList.begin() + ( id- Viewer3D<S, KS>::myCubeSetList.size()- Viewer3D<S, KS>::myLineSetList.size() ) );
        updateList ( false );
      }

    }
  }

}



template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::updateList ( bool needToUpdateBoundingBox )
{
  // Additionnaly to the primitive list (of myCubeSetList myLineSetList.size() myBallSetList.size()) we add
  // 6 new lists associated to the mesh Display.
  unsigned int nbList= ( unsigned int ) ( Viewer3D<S, KS>::myCubeSetList.size() + Viewer3D<S, KS>::myLineSetList.size() + Viewer3D<S, KS>::myBallSetList.size() +6 );
  glDeleteLists ( myListToAff, myNbListe );
  myListToAff = glGenLists ( nbList );
  myNbListe=0;
  unsigned int listeID=0;
  glEnable ( GL_BLEND );
  glEnable ( GL_MULTISAMPLE_ARB );
  glEnable ( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );


  for ( unsigned int i=0; i<Viewer3D<S, KS>::myCubeSetList.size(); i++ )
  {
    glNewList ( myListToAff+i, GL_COMPILE );
    if ( Viewer3D<S, KS>::myListCubeDepthTest.at ( i ) )
    {
      glEnable ( GL_DEPTH_TEST );
    }
    else
    {
      glDisable ( GL_DEPTH_TEST );
    }
    myNbListe++;
    glPushName ( myNbListe );
    glBegin ( GL_QUADS );
    for ( typename std::vector<typename Viewer3D<S, KS>::cubeD3D>::iterator s_it = Viewer3D<S, KS>::myCubeSetList.at ( i ).begin();
          s_it != Viewer3D<S, KS>::myCubeSetList.at ( i ).end();
          ++s_it )
    {

      glColor4ub ( ( *s_it ).R, ( *s_it ).G, ( *s_it ).B, ( *s_it ).T );
      double _width= ( *s_it ).width;

      //z+
      glNormal3f ( 0.0, 0.0, 1.0 );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z+_width );


      //z-
      glNormal3f ( 0.0, 0.0, -1.0 );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z-_width );




      //x+
      glNormal3f ( 1.0, 0.0, 0.0 );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z+_width );


      //x-
      glNormal3f ( -1.0, 0.0, 0.0 );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z-_width );




      //y+
      glNormal3f ( 0.0, 1.0, 0.0 );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z-_width );





      //y-
      glNormal3f ( 0.0, -1.0, 0.0 );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
      glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
      glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );



    }
    glEnd();
    glEndList();
  }
  glNewList ( GLuint ( myListToAff+Viewer3D<S, KS>::myCubeSetList.size() ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glEnable ( GL_DEPTH_TEST );
  glEnable ( GL_BLEND );
  glEnable ( GL_MULTISAMPLE_ARB );
  glEnable ( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  glBegin ( GL_QUADS );
  glEnable ( GL_DEPTH_TEST );
  for ( typename std::vector<typename Viewer3D<S, KS>::quadD3D>::iterator s_it = Viewer3D<S, KS>::mySurfelPrismList.begin();
        s_it != Viewer3D<S, KS>::mySurfelPrismList.end();
        ++s_it )
  {
    glColor4ub ( ( *s_it ).R, ( *s_it ).G, ( *s_it ).B, ( *s_it ).T );
    glNormal3f ( ( *s_it ).nx, ( *s_it ).ny, ( *s_it ).nz );
    glVertex3f ( ( *s_it ).x1, ( *s_it ).y1 , ( *s_it ).z1 );
    glVertex3f ( ( *s_it ).x2, ( *s_it ).y2 , ( *s_it ).z2 );
    glVertex3f ( ( *s_it ).x3, ( *s_it ).y3 , ( *s_it ).z3 );
    glVertex3f ( ( *s_it ).x4, ( *s_it ).y4 , ( *s_it ).z4 );

  }
  glEnd();
  glEndList();




  for ( unsigned int i=0; i<Viewer3D<S, KS>::myLineSetList.size(); i++ )
  {
    listeID++;
    glNewList ( GLuint ( myListToAff+Viewer3D<S, KS>::myCubeSetList.size() +i+1 ), GL_COMPILE );
    myNbListe++;
    glDisable ( GL_LIGHTING );
    glPushName ( myNbListe );
    glBegin ( GL_LINES );
    for ( typename std::vector<typename Viewer3D<S, KS>::lineD3D>::iterator s_it = Viewer3D<S, KS>::myLineSetList.at ( i ).begin();
          s_it != Viewer3D<S, KS>::myLineSetList.at ( i ).end();
          ++s_it )
    {
      glColor4ub ( ( *s_it ).R, ( *s_it ).G, ( *s_it ).B, ( *s_it ).T );
      glVertex3f ( ( *s_it ).x1, ( *s_it ).y1, ( *s_it ).z1 );
      glVertex3f ( ( *s_it ).x2, ( *s_it ).y2, ( *s_it ).z2 );

    }
    glEnd();
    glEnable ( GL_LIGHTING );
    glEndList();

  }


  for ( unsigned int i=0; i<Viewer3D<S, KS>::myBallSetList.size(); i++ )
  {
    glNewList ( GLuint ( myListToAff+Viewer3D<S, KS>::myLineSetList.size() +Viewer3D<S, KS>::myCubeSetList.size() +i+1 ), GL_COMPILE );
    myNbListe++;
    glDepthMask ( GL_TRUE );
    glDisable ( GL_TEXTURE_2D );
    glDisable ( GL_POINT_SMOOTH );
    glDisable ( GL_LIGHTING );

    glPushName ( myNbListe );
    glBegin ( GL_POINTS );
    for ( typename std::vector<typename Viewer3D<S, KS>::ballD3D>::iterator s_it = Viewer3D<S, KS>::myBallSetList.at ( i ).begin();
          s_it != Viewer3D<S, KS>::myBallSetList.at ( i ).end();
          ++s_it )
    {

      glColor4ub ( ( *s_it ).R, ( *s_it ).G, ( *s_it ).B, ( *s_it ).T );
      glVertex3f ( ( *s_it ).x, ( *s_it ).y, ( *s_it ).z );
    }
    glEnd();
    glEnable ( GL_LIGHTING );
    glEndList();

  }



  /**
   *  Creation of new lists to display 3D mesh
   *  First list: quad faces.
   *  Second list: Wired version of quad face.
   *  Third list: Triangle faces.
   *  Fourth list: Wired version of triangle face.
   *  Fifth list: Polygonal faces.
   *  Sixth list: Wired version of polygonal face.
   **


  unsigned int nbListOfPrimitives = Viewer3D<S, KS>::myLineSetList.size() +Viewer3D<S, KS>::myCubeSetList.size() + Viewer3D<S, KS>::myBallSetList.size();

  // First List (quad faces)
  glNewList ( GLuint ( myListToAff +nbListOfPrimitives + 1 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );

  glEnable ( GL_LIGHTING );
  glBegin ( GL_QUADS );

  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::quadD3D> >::iterator it = Viewer3D<S, KS>::myQuadSetList.begin(); it != Viewer3D<S, KS>::myQuadSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::quadD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {

      glColor4ub ( it_s->R, it_s->G, it_s->B, it_s->T );
      glNormal3f ( it_s->nx, it_s->ny ,it_s->nz );
      glVertex3f ( it_s->x1, it_s->y1, it_s->z1 );
      glVertex3f ( it_s->x2, it_s->y2, it_s->z2 );
      glVertex3f ( it_s->x3, it_s->y3, it_s->z3 );
      glVertex3f ( it_s->x4, it_s->y4, it_s->z4 );
    }
  }
  glEnd();
  glEndList();

  // Second list: Wired version of quad face.
  glNewList ( GLuint ( myListToAff +nbListOfPrimitives + 2 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glDisable ( GL_LIGHTING );
  glBegin ( GL_LINES );
  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::quadD3D> >::iterator it = Viewer3D<S, KS>::myQuadSetList.begin(); it != Viewer3D<S, KS>::myQuadSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::quadD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {
      glColor4ub ( 150.0,150.0,150.0,255.0 );
      glColor4ub ( Viewer3D<S, KS>::myCurrentLineColor.red(), Viewer3D<S, KS>::myCurrentLineColor.green(), Viewer3D<S, KS>::myCurrentLineColor.blue() , Viewer3D<S, KS>::myCurrentLineColor.alpha() );
      glVertex3f ( it_s->x1, it_s->y1, it_s->z1 );
      glVertex3f ( it_s->x2, it_s->y2, it_s->z2 );
      glVertex3f ( it_s->x2, it_s->y2, it_s->z2 );
      glColor4ub ( Viewer3D<S, KS>::myCurrentLineColor.red(), Viewer3D<S, KS>::myCurrentLineColor.green(), Viewer3D<S, KS>::myCurrentLineColor.blue() , Viewer3D<S, KS>::myCurrentLineColor.alpha() );
      glVertex3f ( it_s->x3, it_s->y3, it_s->z3 );
      glVertex3f ( it_s->x3, it_s->y3, it_s->z3 );
      glVertex3f ( it_s->x4, it_s->y4, it_s->z4 );
      glColor4ub ( Viewer3D<S, KS>::myCurrentLineColor.red(), Viewer3D<S, KS>::myCurrentLineColor.green(), Viewer3D<S, KS>::myCurrentLineColor.blue() , Viewer3D<S, KS>::myCurrentLineColor.alpha() );
      glVertex3f ( it_s->x4, it_s->y4, it_s->z4 );
      glVertex3f ( it_s->x1, it_s->y1, it_s->z1 );

    }
  }
  glEnable ( GL_LIGHTING );
  glEnd();
  glEndList();

  // Third list: Triangle faces.
  glNewList ( GLuint  (myListToAff +nbListOfPrimitives + 3 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glEnable ( GL_LIGHTING );
  glBegin ( GL_TRIANGLES );
  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::triangleD3D> >::iterator it = Viewer3D<S, KS>::myTriangleSetList.begin(); it != Viewer3D<S, KS>::myTriangleSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::triangleD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {
      glColor4ub (it_s->R,it_s->G,it_s->B,it_s->T );
      glNormal3f (it_s->nx,it_s->ny ,it_s->nz );
      glVertex3f (it_s->x1,it_s->y1,it_s->z1 );
      glVertex3f (it_s->x2,it_s->y2,it_s->z2 );
      glVertex3f (it_s->x3,it_s->y3,it_s->z3 );
    }
  }
  glEnd();
  glEndList();


  // Fourth list: Wired version of triangle face.
  glNewList ( GLuint  ( myListToAff + nbListOfPrimitives +4), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );

  glDisable ( GL_LIGHTING );
  glBegin ( GL_LINES );
  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::triangleD3D> >::iterator it = Viewer3D<S, KS>::myTriangleSetList.begin(); it != Viewer3D<S, KS>::myTriangleSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::triangleD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {
      glColor4ub ( Viewer3D<S, KS>::myCurrentLineColor.red(), Viewer3D<S, KS>::myCurrentLineColor.green(), Viewer3D<S, KS>::myCurrentLineColor.blue() , Viewer3D<S, KS>::myCurrentLineColor.alpha() );
      glVertex3f (it_s->x1,it_s->y1,it_s->z1 );
      glVertex3f (it_s->x2,it_s->y2,it_s->z2 );
      glVertex3f (it_s->x2,it_s->y2,it_s->z2 );
      glVertex3f (it_s->x3,it_s->y3,it_s->z3 );
      glVertex3f (it_s->x3,it_s->y3,it_s->z3 );
      glVertex3f (it_s->x1,it_s->y1,it_s->z1 );
    }
  }
  glEnd();
  glEnable ( GL_LIGHTING );
  glEndList();

  // Fifth list: Polygonal faces.
  glNewList ( GLuint  (myListToAff + nbListOfPrimitives +5 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glEnable ( GL_LIGHTING );
  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::polygonD3D> >::iterator it = Viewer3D<S, KS>::myPolygonSetList.begin(); it != Viewer3D<S, KS>::myPolygonSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::polygonD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {
      glBegin ( GL_POLYGON );
      glColor4ub ( it_s->R, it_s->G, it_s->B, it_s->T );
      glNormal3f ( it_s->nx, it_s->ny ,it_s->nz );
      vector<typename Viewer3D<S, KS>::ballD3D> vectVertex = it_s->vectBalls;
      for(unsigned int j=0;j < vectVertex.size();j++){
        glVertex3f ( vectVertex.at(j).x, vectVertex.at(j).y, vectVertex.at ( j ).z );
      }
    }
    glEnd();
  }
  glEndList();


  // Sixth list: Wired version of polygonal face.
  glNewList ( GLuint  (myListToAff + nbListOfPrimitives +6 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glDisable ( GL_LIGHTING );
  glBegin ( GL_LINES );
  for (typename std::vector<std::vector<typename Viewer3D<S, KS>::polygonD3D> >::iterator it = Viewer3D<S, KS>::myPolygonSetList.begin(); it != Viewer3D<S, KS>::myPolygonSetList.end(); it++)
  {
    for (typename std::vector<typename Viewer3D<S, KS>::polygonD3D>::iterator it_s = it->begin(); it_s != it->end(); it_s++)
    {
      glColor4ub ( Viewer3D<S, KS>::myCurrentLineColor.red(), Viewer3D<S, KS>::myCurrentLineColor.green(), Viewer3D<S, KS>::myCurrentLineColor.blue() , Viewer3D<S, KS>::myCurrentLineColor.alpha() );
       vector<typename Viewer3D<S, KS>::ballD3D> vectVertex = it_s->vectBalls;
      for(unsigned int j=0;j < vectVertex.size();j++){
        glVertex3f ( vectVertex.at(j).x, vectVertex.at(j).y, vectVertex.at ( j ).z );
        glVertex3f ( vectVertex.at((j+1)%vectVertex.size()).x, vectVertex.at((j+1)%vectVertex.size()).y, vectVertex.at ( (j+1)%vectVertex.size() ).z );
      }
    }
  }
  glEnd();
  glEndList();




  myVectTextureImage.clear();

  //Filling new image texture from myGSImageList
  for(unsigned int i=0; i<Viewer3D<S, KS>::myGSImageList.size(); i++){
    typename Viewer3D<S, KS>::TextureImage & aGSImage = Viewer3D<S, KS>::myGSImageList.at(i);
    GLTextureImage textureImg(aGSImage);

    glGenTextures(1, &textureImg.myTextureName);
    glBindTexture(GL_TEXTURE_2D, textureImg.myTextureName);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    if(textureImg.myMode==Viewer3D<S, KS>::GrayScaleMode){
      glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, textureImg.myBufferWidth, textureImg.myBufferHeight, 0,
                   GL_LUMINANCE, GL_UNSIGNED_BYTE, textureImg.myTextureImageBufferGS);
    }else if(textureImg.myMode==Viewer3D<S, KS>::RGBMode){
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureImg.myBufferWidth, textureImg.myBufferHeight, 0,
                   GL_RGB, GL_UNSIGNED_BYTE, textureImg.myTextureImageBufferRGB);
    }

    myVectTextureImage.push_back(textureImg);
  }


  if ( needToUpdateBoundingBox )
  {
    setSceneBoundingBox ( qglviewer::Vec ( Viewer3D<S, KS>::myBoundingPtLow[0],Viewer3D<S, KS>::myBoundingPtLow[1],Viewer3D<S, KS>::myBoundingPtLow[2] ),
        qglviewer::Vec ( Viewer3D<S, KS>::myBoundingPtUp[0], Viewer3D<S, KS>::myBoundingPtUp[1], Viewer3D<S, KS>::myBoundingPtUp[2] ) );
    showEntireScene();
  }
  glPopMatrix();

}


template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::glDrawGLLinel ( typename Viewer3D<S, KS>::lineD3D aLinel )
{
  glPushMatrix();
  glTranslatef ( aLinel.x1, aLinel.y1, aLinel.z1 );
  Vec dir ( aLinel.x2-aLinel.x1, aLinel.y2-aLinel.y1, aLinel.z2-aLinel.z1 );
  glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
  GLUquadric* quadric = gluNewQuadric();
  glColor4ub ( aLinel.R, aLinel.G, aLinel.B, aLinel.T );

  gluCylinder ( quadric, ( aLinel.signPos || !aLinel.isSigned ) ? aLinel.width :0 ,
                ( aLinel.signPos && aLinel.isSigned ) ? 0 :aLinel.width  ,
                dir.norm(),10, 4 );
  glPopMatrix();
}


template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::glDrawGLPointel ( typename Viewer3D<S, KS>::ballD3D ballel )
{

  if ( !ballel.isSigned )
  {
    glPushMatrix();
    glTranslatef ( ballel.x, ballel.y, ballel.z );
    GLUquadric* quadric = gluNewQuadric();
    glColor4ub ( ballel.R, ballel.G, ballel.B, ballel.T );
    gluSphere ( quadric, ballel.size, 10, 10 );
    glPopMatrix();
  }
  else
  {
    // a small "+" is drawn with cylinder
    if ( ballel.signPos )
    {
      glPushMatrix();
      glTranslatef ( ballel.x-0.07, ballel.y-0.07, ballel.z );
      Vec dir ( 0.14, 0.14, 0 );
      glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
      GLUquadric* quadric = gluNewQuadric();
      glColor4ub ( ballel.R, ballel.G, ballel.B, ballel.T );
      gluCylinder ( quadric, ballel.size/3.0 , ballel.size/3.0,
                    dir.norm(),10, 4 );
      glPopMatrix();
      glPushMatrix();
      glTranslatef ( ballel.x-0.07, ballel.y+0.07, ballel.z );
      dir=Vec ( 0.14, -0.14, 0 );
      glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
      quadric = gluNewQuadric();
      glColor4ub ( ballel.R, ballel.G, ballel.B, ballel.T );
      gluCylinder ( quadric, ballel.size/3.0 , ballel.size/3.0,
                    dir.norm(),10, 4 );
      glPopMatrix();
    }
    else
    {
      glPushMatrix();
      glTranslatef ( ballel.x, ballel.y+0.07, ballel.z-0.07 );
      Vec dir ( 0.0, -0.14, 0.14 );
      glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
      GLUquadric* quadric = gluNewQuadric();
      glColor4ub ( ballel.R, ballel.G, ballel.B, ballel.T );
      gluCylinder ( quadric, ballel.size/4.0 , ballel.size/4.0,
                    dir.norm(),10, 4 );
      glPopMatrix();
    }
  }
}



template< typename S, typename KS>
void
DGtal::Viewer3D<S, KS>::keyPressEvent ( QKeyEvent *e )
{
  bool handled = false;

  if( e->key() == Qt::Key_D){
    myIsDoubleFaceRendering = !myIsDoubleFaceRendering;
    if(myIsDoubleFaceRendering)
      glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    else
      glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    updateGL();

  }
  if( e->key() == Qt::Key_E){
    trace.info() << "Exporting mesh..." ;
    (*this) >> "exportedMesh.off";
    trace.info() << "[done]"<< endl ;
  }


  if ( ( e->key() ==Qt::Key_W ) )
  {
    myViewWire=!myViewWire;
    updateList(false);
    updateGL();
  }

  if ( ( e->key() ==Qt::Key_R ) )
  {
    Viewer3D<S, KS>::myScaleX=1.0f;
    Viewer3D<S, KS>::myScaleY=1.0f;
    Viewer3D<S, KS>::myScaleZ=1.0f;
    updateGL();
  }


  if ( ( e->key() ==Qt::Key_T ) )
  {
    handled=true;
    DGtal::trace.info() << "sorting surfel according camera position....";
    sortSurfelFromCamera();
    sortTriangleFromCamera();
    sortQuadFromCamera();
    sortPolygonFromCamera();
    DGtal::trace.info() << " [done]"<< std::endl;
    updateList(false);
    updateGL();
  }
  if ( ( e->key() ==Qt::Key_B ) )
  {
    handled=true;
    myIsBackgroundDefault=!myIsBackgroundDefault;
    if ( !myIsBackgroundDefault )
    {
      setBackgroundColor ( QColor ( 255, 255,255 ) );
    }
    else
    {
      setBackgroundColor ( QColor ( 51, 51, 51 ) );
    }
    updateGL();
  }
  if ( ( e->key() ==Qt::Key_L ) )
  {
    restoreStateFromFile();
    updateGL();
  }
  if ( ( e->key() ==Qt::Key_C ) ) // MT
  {
    handled=true;
    GLint    Viewport[4];
    GLdouble Projection[16], Modelview[16];
    //Unused matrix so I remove it (DC)
    //GLdouble matrix[16];

    // Precomputation begin
    glGetIntegerv ( GL_VIEWPORT         , Viewport );
    glGetDoublev ( GL_MODELVIEW_MATRIX , Modelview );
    glGetDoublev ( GL_PROJECTION_MATRIX, Projection );

    for ( unsigned short m=0; m<4; ++m )
    {
      for ( unsigned short l=0; l<4; ++l )
      {
        double sum = 0.0;
        for ( unsigned short k=0; k<4; ++k )
          sum += Projection[l+4*k]*Modelview[k+4*m];
        //matrix[l+4*m] = sum;
      }
    }
    // Precomputation end

    // print
    DGtal::trace.info() << "Viewport: ";
    for ( unsigned short l=0; l<4; ++l )
      DGtal::trace.info() << Viewport[l] << ", ";
    DGtal::trace.info() << std::endl;

    Vec cp = camera()->position();
    Vec cd = camera()->viewDirection();
    Vec cup = camera()->upVector();

    DGtal::trace.info() << "camera.position: " ;
    for ( unsigned short l=0; l<3; ++l )
      DGtal::trace.info() << cp[l] << ", ";
    DGtal::trace.info() << std::endl;

    DGtal::trace.info() << "camera.direction: ";
    for ( unsigned short l=0; l<3; ++l )
      DGtal::trace.info() << cd[l] << ", ";
    DGtal::trace.info() << std::endl;

    DGtal::trace.info() << "camera.upVector: ";
    for ( unsigned short l=0; l<3; ++l )
      DGtal::trace.info() << cup[l] << ", ";
    DGtal::trace.info() << std::endl;

    DGtal::trace.info() << "zNear: " << camera()->zNear() << " - zFar: " << camera()->zFar() << std::endl;
    // print
  }



  if ( !handled )
    QGLViewer::keyPressEvent ( e );

}




template< typename S, typename KS>
QString
DGtal::Viewer3D<S, KS>::helpString() const
{
  QString text ( "<h2> Viewer3D</h2>" );
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it-> Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Ball</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}


///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////


*/
