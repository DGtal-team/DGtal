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
 * @file DGTalQGLViewer.cpp
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Implementation of methods defined in DGTalQGLViewer.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/3dViewer/DGTalQGLViewer.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/3dViewer/DGTalQGLViewer.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace qglviewer;


///////////////////////////////////////////////////////////////////////////////
// class DGTalQGLViewer
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
DGtal::DGTalQGLViewer::selfDisplay ( std::ostream & out ) const
{
    out << "[DGTalQGLViewer]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::DGTalQGLViewer::isValid() const
{
    return true;
}





void
DGtal::DGTalQGLViewer::draw()
{
  glPushMatrix();
  glMultMatrixd(manipulatedFrame()->matrix());
  for(uint i =0; i< myClippingPlaneList.size(); i++){
    clippingPlaneGL cp = myClippingPlaneList.at(i);
    double eq [4];
    eq[0]=cp.a;
    eq[1]=cp.b;
    eq[2]=cp.c;
    eq[3]=cp.d;
    glEnable(GL_CLIP_PLANE0+i); 
    glClipPlane(GL_CLIP_PLANE0+i, eq );
  }  
  glPopMatrix();   
  
  for(uint i=0; i<myPointSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+myLineSetList.size()+i);
  }   
  for(uint i=0; i<myLineSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+i);
  }
  for(uint i=0; i<myVoxelSetList.size(); i++){
    glCallList(myListToAff+i);
  }
  
}



void
DGtal::DGTalQGLViewer::init(){
  myNbListe=0;
  createNewVoxelList(true);
  vector<lineGL> listeLine;
  myLineSetList.push_back(listeLine);
  vector<pointGL> listePoint;
  myPointSetList.push_back(listePoint);
  myCurrentFillColor = QColor (220, 220, 220);
  myCurrentLineColor = QColor (22, 22, 22, 50);
  myDefaultBackgroundColor = backgroundColor ();
  myIsBackgroundDefault=true;
  camera()->showEntireScene();
  myDefaultColor= QColor(255, 255, 255);
  
  setKeyDescription(Qt::Key_T, "Sort elements for display improvements");
  setKeyDescription(Qt::Key_B, "Switch background color with White/Black colors");
  
  setManipulatedFrame(new ManipulatedFrame());
  restoreStateFromFile();
}



void 
DGtal::DGTalQGLViewer::sortSurfelFromCamera(){
  compFarthestFromCamera comp;
  comp.posCam= camera()->position();
  for(uint i=0; i<myVoxelSetList.size(); i++){
    sort(myVoxelSetList.at(i).begin(), myVoxelSetList.at(i).end(), comp);
  }  
}





void
DGtal::DGTalQGLViewer:: updateList()
{
  uint nbList= myVoxelSetList.size()+ myLineSetList.size()+ myPointSetList.size();
  glDeleteLists(myListToAff, myNbListe);
  myListToAff = glGenLists( nbList  );   
  myNbListe=0;


  glEnable(GL_BLEND);   
  glEnable( GL_MULTISAMPLE_ARB );
  glEnable( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  

  for (uint i=0; i<myVoxelSetList.size(); i++){  
    glNewList(myListToAff+i, GL_COMPILE);
    if(myListVoxelDepthTest.at(i)){
      glEnable( GL_DEPTH_TEST );
    }else{
      glDisable( GL_DEPTH_TEST );
    }
    myNbListe++;
      glBegin(GL_QUADS);
      for (std::vector<voxelGL>::iterator s_it = myVoxelSetList.at(i).begin();
	   s_it != myVoxelSetList.at(i).end();
	   ++s_it){
	
	glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
	double width=(*s_it).width;
	if((*s_it).x <myBoundingPtLow[0])   myBoundingPtLow[0]= (*s_it).x;
	if((*s_it).y <myBoundingPtLow[1])   myBoundingPtLow[1]= (*s_it).y;
	if((*s_it).z <myBoundingPtLow[2])   myBoundingPtLow[2]= (*s_it).z;

	if((*s_it).x >myBoundingPtUp[0])   myBoundingPtUp[0]= (*s_it).x;
	if((*s_it).y >myBoundingPtUp[1])   myBoundingPtUp[1]= (*s_it).y;
	if((*s_it).z >myBoundingPtUp[2])   myBoundingPtUp[2]= (*s_it).z;
									   
	//z+
	glNormal3f( 0.0, 0.0, 1.0);
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width);
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width);
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width);
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width);
	//z-
	glNormal3f( 0.0, 0.0, -1.0);
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width);
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width);
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width);
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width);
	//x+
	glNormal3f( 1.0, 0.0, 0.0);
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width );
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width );
	//x-
	glNormal3f( -1.0, 0.0, 0.0);
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width );
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width );
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width );
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width );
	//y+
	glNormal3f( 0.0, 1.0, 0.0);
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width );
	glVertex3f((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width );
	//y-
	glNormal3f( 0.0, -1.0, 0.0);
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width );
	glVertex3f((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width );
	glVertex3f((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width );      
      }
      glEnd();
      glEndList();
    }
  

  for (uint i=0; i<myLineSetList.size(); i++){  
    glNewList(myListToAff+myVoxelSetList.size()+i, GL_COMPILE);
    myNbListe++;
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);      
    for (std::vector<lineGL>::iterator s_it = myLineSetList.at(i).begin();
	 s_it != myLineSetList.at(i).end();
	 ++s_it){
	if((*s_it).x1 <myBoundingPtLow[0])   myBoundingPtLow[0]= (*s_it).x1;
	if((*s_it).y1 <myBoundingPtLow[1])   myBoundingPtLow[1]= (*s_it).y1;
	if((*s_it).z1 <myBoundingPtLow[2])   myBoundingPtLow[2]= (*s_it).z1;
	
	if((*s_it).x1 >myBoundingPtUp[0])   myBoundingPtUp[0]= (*s_it).x1;
	if((*s_it).y1 >myBoundingPtUp[1])   myBoundingPtUp[1]= (*s_it).y1;
	if((*s_it).z1 >myBoundingPtUp[2])   myBoundingPtUp[2]= (*s_it).z1;

	if((*s_it).x2 <myBoundingPtLow[0])   myBoundingPtLow[0]= (*s_it).x2;
	if((*s_it).y2 <myBoundingPtLow[1])   myBoundingPtLow[1]= (*s_it).y2;
	if((*s_it).z2 <myBoundingPtLow[2])   myBoundingPtLow[2]= (*s_it).z2;
	
	if((*s_it).x2 >myBoundingPtUp[0])   myBoundingPtUp[0]= (*s_it).x2;
	if((*s_it).y2 >myBoundingPtUp[1])   myBoundingPtUp[1]= (*s_it).y2;
	if((*s_it).z2 >myBoundingPtUp[2])   myBoundingPtUp[2]= (*s_it).z2;

	glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
	glVertex3f((*s_it).x1,  (*s_it).y1, (*s_it).z1);
	glVertex3f((*s_it).x2,  (*s_it).y2, (*s_it).z2);
	
      }
      glEnd();
      glEnable(GL_LIGHTING);
      glEndList();
  
  }    


  for (uint i=0; i<myPointSetList.size(); i++){  
    glNewList(myListToAff+myLineSetList.size()+myVoxelSetList.size()+i, GL_COMPILE);
    myNbListe++;
    glDepthMask(GL_TRUE);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_LIGHTING);
    if(myPointSetList.at(i).size()!=0){
      glPointSize((*myPointSetList.at(i).begin()).size);
    }
    glBegin(GL_POINTS);      
    for (std::vector<pointGL>::iterator s_it = myPointSetList.at(i).begin();
	 s_it != myPointSetList.at(i).end();
	 ++s_it){
      glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
      glVertex3f((*s_it).x,  (*s_it).y, (*s_it).z);
    }
    glEnd();
    glEnable(GL_LIGHTING);
    glEndList();
    
  }    


  setSceneBoundingBox(myBoundingPtLow, myBoundingPtUp);
  showEntireScene();

 
}





void 
DGtal::DGTalQGLViewer::keyPressEvent(QKeyEvent *e){
  bool handled = false;
  
  if ((e->key()==Qt::Key_T) ){
    handled=true;
    cerr << "sorting surfel according camera position...";
    sortSurfelFromCamera();
    cerr << " [done]"<< endl;
    updateList();    
    updateGL();
  }
  if( (e->key()==Qt::Key_B)){
    handled=true;
    myIsBackgroundDefault=!myIsBackgroundDefault;
    if(!myIsBackgroundDefault){
      setBackgroundColor(QColor(255, 255,255));
    }else{
      setBackgroundColor(QColor(51, 51, 51));
    }
    updateGL();
  }
  if (!handled)
    QGLViewer::keyPressEvent(e);

}


QString 
DGtal::DGTalQGLViewer::helpString() const
{
  QString text("<h2> DGTalQGLViewer</h2>");
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
