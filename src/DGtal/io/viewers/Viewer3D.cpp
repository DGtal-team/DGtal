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
 */
void
DGtal::Viewer3D::selfDisplay ( std::ostream & out ) const
{
  out << "[Viewer3D]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::Viewer3D::isValid() const
{
  return true;
}






void
DGtal::Viewer3D::drawWithNames()
{
  for ( unsigned int i=0; i<myVoxelSetList.size(); i++ )
    {
      glCallList ( myListToAff+i );
    }
  for ( unsigned int i=0; i<myLineSetList.size(); i++ )
    {
      glCallList ( GLuint ( myListToAff+myVoxelSetList.size() +i ) );
    }

  for ( unsigned int i=0; i<myPointSetList.size(); i++ )
    {
      glCallList ( GLuint ( myListToAff+myVoxelSetList.size() +myLineSetList.size() +i ) );
    }
}


void
DGtal::Viewer3D::draw()
{
  glPushMatrix();
  glMultMatrixd ( manipulatedFrame()->matrix() );
  
  glPushMatrix();
  glScalef(myScaleX, myScaleY, myScaleZ);    

  for ( unsigned int i =0; i< myClippingPlaneList.size(); i++ )
    {
      clippingPlaneD3D cp = myClippingPlaneList.at ( i );
      double eq [4];
      eq[0]=cp.a;
      eq[1]=cp.b;
      eq[2]=cp.c;
      eq[3]=cp.d;
      glEnable ( GL_CLIP_PLANE0+i );
      glClipPlane ( GL_CLIP_PLANE0+i, eq );
    }


  Vec centerS = sceneCenter();
  Vec posCam = camera()->position();
  double distCam =sqrt ( ( posCam.x-centerS.x ) * ( posCam.x-centerS.x ) +
			 ( posCam.y-centerS.y ) * ( posCam.y-centerS.y ) +
			 ( posCam.z-centerS.z ) * ( posCam.z-centerS.z ) );

 
  for ( unsigned int i=0; i<myPointSetList.size(); i++ )
    {
      if ( myPointSetList.at ( i ).size() !=0 )
        {
	  glPointSize ( ( myPointSetList.at ( i ).at ( 0 ).size ) /distCam );
        }
      glCallList ( GLuint ( myListToAff+myVoxelSetList.size() +myLineSetList.size() +i+1 ) );
    }

  for ( unsigned int i=0; i<myLineSetList.size(); i++ )
    {
      if ( myLineSetList.at ( i ).size() !=0 )
	{
	  glLineWidth ( ( myLineSetList.at ( i ).at ( 0 ).width ) );
	}
      glCallList ( GLuint ( myListToAff+myVoxelSetList.size() +1+i ) );
    }

  glCallList ( GLuint ( myListToAff+myVoxelSetList.size() ) );
  for ( unsigned int i=0; i<myVoxelSetList.size(); i++ )
    {
      glCallList ( myListToAff+i );
    }
   


  // Calling lists associated to Mesh display (see. updateList)
  unsigned int nbListOfPrimitives = myLineSetList.size() +myVoxelSetList.size() + myPointSetList.size();
  glLineWidth ( myMeshDefaultLineWidth /distCam );
  glDisable(GL_CULL_FACE);
  glCallList ( GLuint ( myListToAff+nbListOfPrimitives+1 ) );
  if(myViewWire){
      glLineWidth ( myMeshDefaultLineWidth /distCam );
      glCallList ( GLuint ( myListToAff+nbListOfPrimitives+2 ) );
  }

  glDisable(GL_CULL_FACE);
  glCallList ( GLuint ( myListToAff+nbListOfPrimitives+3 ) );
    
  if(myViewWire){
    glLineWidth ( myMeshDefaultLineWidth /distCam );
    glCallList ( GLuint ( myListToAff+nbListOfPrimitives+4 ) );
  }


  glDisable(GL_CULL_FACE);
  glCallList ( GLuint ( myListToAff+nbListOfPrimitives+5 ) );
  if(myViewWire){
    glLineWidth ( myMeshDefaultLineWidth /distCam );
    glCallList ( GLuint ( myListToAff+nbListOfPrimitives+6 ) );
  }
  

    


    

  // Drawing all Khalimsky Space Cells

  for ( unsigned int i=0; i< myKSPointelList.size(); i++ )
    {
      glDrawGLPointel ( myKSPointelList.at ( i ) );
    }
  for ( unsigned int i=0; i< myKSLinelList.size(); i++ )
    {
      glDrawGLLinel ( myKSLinelList.at ( i ) );
    }
 
  

    

  // Drawing all Khalimsky Space Cells

  for ( unsigned int i=0; i< myKSPointelList.size(); i++ )
    {
      glDrawGLPointel ( myKSPointelList.at ( i ) );
    }
  for ( unsigned int i=0; i< myKSLinelList.size(); i++ )
    {
      glDrawGLLinel ( myKSLinelList.at ( i ) );
    }
    


    

  // Drawing all Khalimsky Space Cells

  for ( unsigned int i=0; i< myKSPointelList.size(); i++ )
    {
      glDrawGLPointel ( myKSPointelList.at ( i ) );
    }
  for ( unsigned int i=0; i< myKSLinelList.size(); i++ )
    {
      glDrawGLLinel ( myKSLinelList.at ( i ) );
    }

  glPopMatrix();

  glPopMatrix();  
}

#if defined( max )
#undef max
#define _HAS_MSVC_MAX_ true
#endif

#if defined( min )
#undef min
#define _HAS_MSVC_MIN_ true
#endif

void
DGtal::Viewer3D::init()
{
  myMeshDefaultLineWidth=10.0;
  myNbListe=0;
  myViewWire=true;
  createNewVoxelList ( true );
  vector<lineD3D> listeLine;
  myLineSetList.push_back ( listeLine );
  vector<pointD3D> listePoint;
  myPointSetList.push_back ( listePoint );
  myCurrentFillColor = Color ( 220, 220, 220 );
  myCurrentLineColor = Color ( 22, 22, 222, 50 );
  myDefaultBackgroundColor = Color ( backgroundColor().red(), backgroundColor().green(),
				     backgroundColor().blue() );
  myIsBackgroundDefault=true;
  myBoundingPtLow[0]=numeric_limits<double>::max( );
  myBoundingPtLow[1]=numeric_limits<double>::max( );
  myBoundingPtLow[2]=numeric_limits<double>::max( );

  myBoundingPtUp[0]=numeric_limits<double>::min( );
  myBoundingPtUp[1]=numeric_limits<double>::min( );
  myBoundingPtUp[2]=numeric_limits<double>::min( );
  createNewVoxelList ( true );
  std::vector<voxelD3D>  aKSVoxelList;

  myCurrentfShiftVisuKSSurfels=0.0;
  myDefaultColor= Color ( 255, 255, 255 );
  camera()->showEntireScene();
  setKeyDescription ( Qt::Key_E, "Export the current display into OFF file (just Voxel, surfel and KSSurfel for now)." );  
  setKeyDescription ( Qt::Key_W, "Switch display with and without wired view of triangle and quad faces." );
  setKeyDescription ( Qt::Key_T, "Sort elements for display improvements." );
  setKeyDescription ( Qt::Key_L, "Load last visualisation settings." );
  setKeyDescription ( Qt::Key_B, "Switch background color with White/Black colors." );
  setKeyDescription ( Qt::Key_C, "Show camera informations." );
  setKeyDescription ( Qt::Key_R, "Reset default scale for 3 axes to 1.0f." );


  setMouseBindingDescription ( Qt::ShiftModifier+Qt::RightButton, "Delete the mouse selected list." );
  setManipulatedFrame ( new ManipulatedFrame() );

}

#if defined( _HAS_MSVC_MAX_ )
#define max(A,B) ((A)>(B)?(A):(B))
#endif

#if defined( _HAS_MSVC_MIN_ )
#define min(A,B) ((A)<(B)?(A):(B))
#endif

void
DGtal::Viewer3D::sortSurfelFromCamera()
{
  compFarthestVoxelFromCamera comp;
  comp.posCam= camera()->position();
  for ( unsigned int i=0; i<myVoxelSetList.size(); i++ )
    {
      sort ( myVoxelSetList.at ( i ).begin(), myVoxelSetList.at ( i ).end(), comp );
    }
  compFarthestSurfelFromCamera compSurf;
  DGtal::trace.info() << "sort surfel size" << myKSSurfelList.size() << std::endl;
  sort ( myKSSurfelList.begin(), myKSSurfelList.end(), compSurf );

}


void
DGtal::Viewer3D::sortTriangleFromCamera()
{
  compFarthestTriangleFromCamera comp;
  comp.posCam= camera()->position();
  
  DGtal::trace.info() << "sort triangle size" << myTriangleList.size() << std::endl;
  sort ( myTriangleList.begin(), myTriangleList.end(), comp );
    
}




void
DGtal::Viewer3D::sortQuadFromCamera()
{
  compFarthestSurfelFromCamera comp;
  comp.posCam= camera()->position();
  
  DGtal::trace.info() << "sort quad size" << myTriangleList.size() << std::endl;
  sort ( myQuadList.begin(), myQuadList.end(), comp );
    
}



void
DGtal::Viewer3D::sortPolygonFromCamera()
{
  compFarthestPolygonFromCamera comp;
  comp.posCam= camera()->position();
  
  DGtal::trace.info() << "sort polygon size" << myPolygonList.size() << std::endl;
  sort ( myPolygonList.begin(), myPolygonList.end(), comp );
    
}





void
DGtal::Viewer3D::postSelection ( const QPoint& point )
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
	  if ( id< myVoxelSetList.size() )
            {
	      DGtal::trace.info() << "deleting list="<< id<<endl;
	      myVoxelSetList.erase ( myVoxelSetList.begin() +id );
	      updateList ( false );
            }
	  else if ( id< myVoxelSetList.size() +myLineSetList.size() )
            {
	      myLineSetList.erase ( myLineSetList.begin() + ( id-myVoxelSetList.size() ) );
	      updateList ( false );
            }
	  else if ( id< myPointSetList.size() +myLineSetList.size() +myVoxelSetList.size() )
            {
	      myPointSetList.erase ( myPointSetList.begin() + ( id-myVoxelSetList.size()-myLineSetList.size() ) );
	      updateList ( false );
            }

        }
    }

}




void
DGtal::Viewer3D::updateList ( bool needToUpdateBoundingBox )
{
  // Additionnaly to the primitive list (of myVoxelSetList myLineSetList.size() myPointSetList.size()) we add 
  // 6 new lists associated to the mesh Display.
  unsigned int nbList= ( unsigned int ) ( myVoxelSetList.size() + myLineSetList.size() + myPointSetList.size() +6 );
  glDeleteLists ( myListToAff, myNbListe );
  myListToAff = glGenLists ( nbList );
  myNbListe=0;
  unsigned int listeID=0;
  glEnable ( GL_BLEND );
  glEnable ( GL_MULTISAMPLE_ARB );
  glEnable ( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    
  for ( unsigned int i=0; i<myVoxelSetList.size(); i++ )
    {
      glNewList ( myListToAff+i, GL_COMPILE );
      if ( myListVoxelDepthTest.at ( i ) )
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
      for ( std::vector<voxelD3D>::iterator s_it = myVoxelSetList.at ( i ).begin();
	    s_it != myVoxelSetList.at ( i ).end();
	    ++s_it )
        {

	  glColor4ub ( ( *s_it ).R, ( *s_it ).G, ( *s_it ).B, ( *s_it ).T );
	  double _width= ( *s_it ).width;

	  //z+
	  glNormal3f ( 0.0, 0.0, 1.0 );
	  glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
	  //z-
	  glNormal3f ( 0.0, 0.0, -1.0 );
	  glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
	  glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
	  //x+
	  glNormal3f ( 1.0, 0.0, 0.0 );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y+_width, ( *s_it ).z-_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
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
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z+_width );
	  glVertex3f ( ( *s_it ).x+_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
	  glVertex3f ( ( *s_it ).x-_width, ( *s_it ).y-_width, ( *s_it ).z-_width );
        }
      glEnd();
      glEndList();
    }
  glNewList ( GLuint ( myListToAff+myVoxelSetList.size() ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glEnable ( GL_DEPTH_TEST );
  glEnable ( GL_BLEND );
  glEnable ( GL_MULTISAMPLE_ARB );
  glEnable ( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc ( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  glBegin ( GL_QUADS );
  glEnable ( GL_DEPTH_TEST );
  for ( std::vector<quadD3D>::iterator s_it = myKSSurfelList.begin();
	s_it != myKSSurfelList.end();
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


  for ( unsigned int i=0; i<myLineSetList.size(); i++ )
    {
      listeID++;
      glNewList ( GLuint ( myListToAff+myVoxelSetList.size() +i+1 ), GL_COMPILE );
      myNbListe++;
      glDisable ( GL_LIGHTING );
      glPushName ( myNbListe );
      glBegin ( GL_LINES );
      for ( std::vector<lineD3D>::iterator s_it = myLineSetList.at ( i ).begin();
	    s_it != myLineSetList.at ( i ).end();
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


  for ( unsigned int i=0; i<myPointSetList.size(); i++ )
    {
      glNewList ( GLuint ( myListToAff+myLineSetList.size() +myVoxelSetList.size() +i+1 ), GL_COMPILE );
      myNbListe++;
      glDepthMask ( GL_TRUE );
      glDisable ( GL_TEXTURE_2D );
      glDisable ( GL_POINT_SMOOTH );
      glDisable ( GL_LIGHTING );

      glPushName ( myNbListe );
      glBegin ( GL_POINTS );
      for ( std::vector<pointD3D>::iterator s_it = myPointSetList.at ( i ).begin();
	    s_it != myPointSetList.at ( i ).end();
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
   **/


  unsigned int nbListOfPrimitives = myLineSetList.size() +myVoxelSetList.size() + myPointSetList.size();

  // First List (quad faces)
  glNewList ( GLuint ( myListToAff +nbListOfPrimitives + 1 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  
  glEnable ( GL_LIGHTING );  
  glBegin ( GL_QUADS );

  for ( unsigned int i=0; i<myQuadList.size(); i++ )
    {        
      glColor4ub ( myQuadList.at ( i ).R, myQuadList.at ( i ).G, myQuadList.at ( i ).B, myQuadList.at ( i ).T );
      glNormal3f ( -myQuadList.at ( i ).nx, -myQuadList.at ( i ).ny ,-myQuadList.at ( i ).nz );
      glVertex3f ( myQuadList.at ( i ).x1, myQuadList.at ( i ).y1, myQuadList.at ( i ).z1 );
      glVertex3f ( myQuadList.at ( i ).x2, myQuadList.at ( i ).y2, myQuadList.at ( i ).z2 );
      glVertex3f ( myQuadList.at ( i ).x3, myQuadList.at ( i ).y3, myQuadList.at ( i ).z3 );
      glVertex3f ( myQuadList.at ( i ).x4, myQuadList.at ( i ).y4, myQuadList.at ( i ).z4 );

    }
  glEnd();
  glEndList();
  
  // Second list: Wired version of quad face.
  glNewList ( GLuint ( myListToAff +nbListOfPrimitives + 2 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glDisable ( GL_LIGHTING );
  glBegin ( GL_LINES );
  for ( unsigned int i=0; i<myQuadList.size(); i++ )
    {
      glColor4ub ( 150.0,150.0,150.0,255.0 );
      glColor4ub ( myCurrentLineColor.red(), myCurrentLineColor.green(), myCurrentLineColor.blue() , myCurrentLineColor.alpha() );
      glVertex3f ( myQuadList.at ( i ).x1, myQuadList.at ( i ).y1, myQuadList.at ( i ).z1 );
      glVertex3f ( myQuadList.at ( i ).x2, myQuadList.at ( i ).y2, myQuadList.at ( i ).z2 );
      glVertex3f ( myQuadList.at ( i ).x2, myQuadList.at ( i ).y2, myQuadList.at ( i ).z2 );
      glColor4ub ( myCurrentLineColor.red(), myCurrentLineColor.green(), myCurrentLineColor.blue() , myCurrentLineColor.alpha() );
      glVertex3f ( myQuadList.at ( i ).x3, myQuadList.at ( i ).y3, myQuadList.at ( i ).z3 );
      glVertex3f ( myQuadList.at ( i ).x3, myQuadList.at ( i ).y3, myQuadList.at ( i ).z3 );
      glVertex3f ( myQuadList.at ( i ).x4, myQuadList.at ( i ).y4, myQuadList.at ( i ).z4 );
      glColor4ub ( myCurrentLineColor.red(), myCurrentLineColor.green(), myCurrentLineColor.blue() , myCurrentLineColor.alpha() );
      glVertex3f ( myQuadList.at ( i ).x4, myQuadList.at ( i ).y4, myQuadList.at ( i ).z4 );
      glVertex3f ( myQuadList.at ( i ).x1, myQuadList.at ( i ).y1, myQuadList.at ( i ).z1 );
        
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
  for ( unsigned int i=0; i<myTriangleList.size(); i++ )
    {
      glColor4ub ( myTriangleList.at ( i ).R, myTriangleList.at ( i ).G, myTriangleList.at ( i ).B, myTriangleList.at ( i ).T );  
      glNormal3f ( -myTriangleList.at ( i ).nx, -myTriangleList.at ( i ).ny ,-myTriangleList.at ( i ).nz );
      glVertex3f ( myTriangleList.at ( i ).x1, myTriangleList.at ( i ).y1, myTriangleList.at ( i ).z1 );
      glVertex3f ( myTriangleList.at ( i ).x2, myTriangleList.at ( i ).y2, myTriangleList.at ( i ).z2 );        
      glVertex3f ( myTriangleList.at ( i ).x3, myTriangleList.at ( i ).y3, myTriangleList.at ( i ).z3 );
    }   
  glEnd();
  glEndList();


  // Fourth list: Wired version of triangle face.
  glNewList ( GLuint  ( myListToAff + nbListOfPrimitives +4), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  
  glDisable ( GL_LIGHTING );
  glBegin ( GL_LINES );
  for ( unsigned int i=0; i<myTriangleList.size(); i++ )
    {
      glColor4ub ( myCurrentLineColor.red(), myCurrentLineColor.green(), myCurrentLineColor.blue() , myCurrentLineColor.alpha() );
      glVertex3f ( myTriangleList.at ( i ).x1, myTriangleList.at ( i ).y1, myTriangleList.at ( i ).z1 );
      glVertex3f ( myTriangleList.at ( i ).x2, myTriangleList.at ( i ).y2, myTriangleList.at ( i ).z2 );
      glVertex3f ( myTriangleList.at ( i ).x2, myTriangleList.at ( i ).y2, myTriangleList.at ( i ).z2 );
      glVertex3f ( myTriangleList.at ( i ).x3, myTriangleList.at ( i ).y3, myTriangleList.at ( i ).z3 );
      glVertex3f ( myTriangleList.at ( i ).x3, myTriangleList.at ( i ).y3, myTriangleList.at ( i ).z3 );
      glVertex3f ( myTriangleList.at ( i ).x1, myTriangleList.at ( i ).y1, myTriangleList.at ( i ).z1 );    
    
    }
  glEnd();
  glEnable ( GL_LIGHTING );
  glEndList();

  // Fifth list: Polygonal faces.
  glNewList ( GLuint  (myListToAff + nbListOfPrimitives +5 ), GL_COMPILE );
  myNbListe++;
  glPushName ( myNbListe );
  glEnable ( GL_LIGHTING );  
  for ( unsigned int i=0; i<myPolygonList.size(); i++ )
    {
      glBegin ( GL_POLYGON );
      glColor4ub ( myPolygonList.at ( i ).R, myPolygonList.at ( i ).G, myPolygonList.at ( i ).B, myPolygonList.at ( i ).T );
      glNormal3f ( -myPolygonList.at ( i ).nx, -myPolygonList.at ( i ).ny ,-myPolygonList.at ( i ).nz );
      vector<pointD3D> vectVertex = myPolygonList.at ( i ).vectPoints;
      for(unsigned int j=0;j < vectVertex.size();j++){
	glVertex3f ( vectVertex.at(j).x, vectVertex.at(j).y, vectVertex.at ( j ).z );
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
  for ( unsigned int i=0; i<myPolygonList.size(); i++ )
    {
      glColor4ub ( myCurrentLineColor.red(), myCurrentLineColor.green(), myCurrentLineColor.blue() , myCurrentLineColor.alpha() );
      vector<pointD3D> vectVertex = myPolygonList.at ( i ).vectPoints;
      for(unsigned int j=0;j < vectVertex.size();j++){
	glVertex3f ( vectVertex.at(j).x, vectVertex.at(j).y, vectVertex.at ( j ).z );
	glVertex3f ( vectVertex.at((j+1)%vectVertex.size()).x, vectVertex.at((j+1)%vectVertex.size()).y, vectVertex.at ( (j+1)%vectVertex.size() ).z );
      }
    }
  glEnd();
  glEndList();
  

  if ( needToUpdateBoundingBox )
    {
      setSceneBoundingBox ( qglviewer::Vec ( myBoundingPtLow[0],myBoundingPtLow[1],myBoundingPtLow[2] ),
			    qglviewer::Vec ( myBoundingPtUp[0], myBoundingPtUp[1], myBoundingPtUp[2] ) );
      showEntireScene();
    }
  glPopMatrix();

}





void
DGtal::Viewer3D::glDrawGLLinel ( lineD3D aLinel )
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





void
DGtal::Viewer3D::glDrawGLPointel ( pointD3D pointel )
{

  if ( !pointel.isSigned )
    {
      glPushMatrix();
      glTranslatef ( pointel.x, pointel.y, pointel.z );
      GLUquadric* quadric = gluNewQuadric();
      glColor4ub ( pointel.R, pointel.G, pointel.B, pointel.T );
      gluSphere ( quadric, pointel.size, 10, 10 );
      glPopMatrix();
    }
  else
    {
      // a small "+" is drawn with cylinder
      if ( pointel.signPos )
        {
	  glPushMatrix();
	  glTranslatef ( pointel.x-0.07, pointel.y-0.07, pointel.z );
	  Vec dir ( 0.14, 0.14, 0 );
	  glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
	  GLUquadric* quadric = gluNewQuadric();
	  glColor4ub ( pointel.R, pointel.G, pointel.B, pointel.T );
	  gluCylinder ( quadric, pointel.size/3.0 , pointel.size/3.0,
			dir.norm(),10, 4 );
	  glPopMatrix();
	  glPushMatrix();
	  glTranslatef ( pointel.x-0.07, pointel.y+0.07, pointel.z );
	  dir=Vec ( 0.14, -0.14, 0 );
	  glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
	  quadric = gluNewQuadric();
	  glColor4ub ( pointel.R, pointel.G, pointel.B, pointel.T );
	  gluCylinder ( quadric, pointel.size/3.0 , pointel.size/3.0,
			dir.norm(),10, 4 );
	  glPopMatrix();
        }
      else
        {
	  glPushMatrix();
	  glTranslatef ( pointel.x, pointel.y+0.07, pointel.z-0.07 );
	  Vec dir ( 0.0, -0.14, 0.14 );
	  glMultMatrixd ( Quaternion ( Vec ( 0,0,1 ), dir ).matrix() );
	  GLUquadric* quadric = gluNewQuadric();
	  glColor4ub ( pointel.R, pointel.G, pointel.B, pointel.T );
	  gluCylinder ( quadric, pointel.size/4.0 , pointel.size/4.0,
			dir.norm(),10, 4 );
	  glPopMatrix();
        }
    }
}





void
DGtal::Viewer3D::keyPressEvent ( QKeyEvent *e )
{
  bool handled = false;

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
      myScaleX=1.0f;
      myScaleY=1.0f;
      myScaleZ=1.0f;
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







QString
DGtal::Viewer3D::helpString() const
{
  QString text ( "<h2> Viewer3D</h2>" );
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  return text;
}


///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////


