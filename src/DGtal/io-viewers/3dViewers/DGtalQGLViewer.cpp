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
 * @file DGtalQGLViewer.cpp
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Implementation of methods defined in DGtalQGLViewer.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io-viewers/3dViewers/DGtalQGLViewer.h"
#include <limits>
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/io-viewers/3dViewers/DGtalQGLViewer.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace qglviewer;


///////////////////////////////////////////////////////////////////////////////
// class DGtalQGLViewer
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
DGtal::DGtalQGLViewer::selfDisplay ( std::ostream & out ) const
{
    out << "[DGtalQGLViewer]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::DGtalQGLViewer::isValid() const
{
    return true;
}






void
DGtal::DGtalQGLViewer::drawWithNames(){   
  
  for(uint i=0; i<myVoxelSetList.size(); i++){
    glCallList(myListToAff+i);
  }
  for(uint i=0; i<myLineSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+i);
  }
  
  for(uint i=0; i<myPointSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+myLineSetList.size()+i);
  }   


}


void
DGtal::DGtalQGLViewer::draw()
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
    glCallList(myListToAff+myVoxelSetList.size()+myLineSetList.size()+i+1);
  }   
 
  for(uint i=0; i<myLineSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+1+i);
  }
  
  glCallList(myListToAff+myVoxelSetList.size());
  for(uint i=0; i<myVoxelSetList.size(); i++){
    glCallList(myListToAff+i);
  }
  
  
     
  for(uint i=0; i<myQuadList.size(); i++){
    double  ux=myQuadList.at(i).x2-myQuadList.at(i).x1; 
    double  uy=myQuadList.at(i).y2-myQuadList.at(i).y1; 
    double  uz=myQuadList.at(i).z2-myQuadList.at(i).z1; 
    
    double  vx=myQuadList.at(i).x3-myQuadList.at(i).x2; 
    double  vy=myQuadList.at(i).y3-myQuadList.at(i).y2; 
    double  vz=myQuadList.at(i).z3-myQuadList.at(i).z2; 

    Vec n( uy*vz-uz*vy, uz*vx-ux*vz,  ux*vy-uy*vx );
    double normeN=sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
    
    glBegin(GL_QUADS);
    glColor4ub(myQuadList.at(i).R, myQuadList.at(i).G, myQuadList.at(i).B, myQuadList.at(i).T);    
    glNormal3f(-n[0]/normeN,-n[1]/normeN,-n[2]/normeN);
    glVertex3f(myQuadList.at(i).x1, myQuadList.at(i).y1, myQuadList.at(i).z1);
    glVertex3f(myQuadList.at(i).x2, myQuadList.at(i).y2, myQuadList.at(i).z2);
    glVertex3f(myQuadList.at(i).x3, myQuadList.at(i).y3, myQuadList.at(i).z3);
    glVertex3f(myQuadList.at(i).x4, myQuadList.at(i).y4, myQuadList.at(i).z4);
    glEnd();
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor4ub(150.0,150.0,150.0,255.0);
    glVertex3f(myQuadList.at(i).x1, myQuadList.at(i).y1, myQuadList.at(i).z1);
    glVertex3f(myQuadList.at(i).x2, myQuadList.at(i).y2, myQuadList.at(i).z2);
    glVertex3f(myQuadList.at(i).x2, myQuadList.at(i).y2, myQuadList.at(i).z2);
    glVertex3f(myQuadList.at(i).x3, myQuadList.at(i).y3, myQuadList.at(i).z3);
    glVertex3f(myQuadList.at(i).x3, myQuadList.at(i).y3, myQuadList.at(i).z3);
    glVertex3f(myQuadList.at(i).x4, myQuadList.at(i).y4, myQuadList.at(i).z4);
    glVertex3f(myQuadList.at(i).x4, myQuadList.at(i).y4, myQuadList.at(i).z4);
    glVertex3f(myQuadList.at(i).x1, myQuadList.at(i).y1, myQuadList.at(i).z1);
    glEnd();
    glEnable(GL_LIGHTING);
  }

  
  // Drawing all Khalimsky Space Cells 
  
  for(uint i=0; i< myKSPointelList.size();i++){
    glDrawGLPointel(myKSPointelList.at(i));
  }
  for(uint i=0; i< myKSLinelList.size();i++){
    glDrawGLLinel(myKSLinelList.at(i));
  }



}



void
DGtal::DGtalQGLViewer::init(){
  myNbListe=0;
  createNewVoxelList(true);
  vector<lineGL> listeLine;
  myLineSetList.push_back(listeLine);
  vector<pointGL> listePoint;
  myPointSetList.push_back(listePoint);
  myCurrentFillColor = QColor (220, 220, 220);
  myCurrentLineColor = QColor (22, 22, 222, 50);
  myDefaultBackgroundColor = backgroundColor ();
  myIsBackgroundDefault=true;
  myBoundingPtLow[0]=numeric_limits<double>::max( );
  myBoundingPtLow[1]=numeric_limits<double>::max( );
  myBoundingPtLow[2]=numeric_limits<double>::max( );

  myBoundingPtUp[0]=numeric_limits<double>::min( );
  myBoundingPtUp[1]=numeric_limits<double>::min( );
  myBoundingPtUp[2]=numeric_limits<double>::min( );
  

  
  myDefaultColor= QColor(255, 255, 255);
  camera()->showEntireScene();
  
  setKeyDescription(Qt::Key_T, "Sort elements for display improvements");
  setKeyDescription(Qt::Key_L, "Load last visualisation settings.");
  setKeyDescription(Qt::Key_B, "Switch background color with White/Black colors.");

  setMouseBindingDescription(Qt::ShiftModifier+Qt::RightButton, "Delete the mouse selected list.");  
  setManipulatedFrame(new ManipulatedFrame());  

  
}



void 
DGtal::DGtalQGLViewer::sortSurfelFromCamera(){
  compFarthestFromCamera comp;
  comp.posCam= camera()->position();
  for(uint i=0; i<myVoxelSetList.size(); i++){
    sort(myVoxelSetList.at(i).begin(), myVoxelSetList.at(i).end(), comp);
  }  
}



void 
DGtal::DGtalQGLViewer::postSelection(const QPoint& point)
{
  camera()->convertClickToLine(point, myOrig, myDir);
  bool found;
  this->myPosSelector= point;
  mySelectedPoint = camera()->pointUnderPixel(point, found);
  if(found){
    cerr << "Element of liste= " << selectedName() << "selected" << endl; 
    if(selectedName() !=-1){
      uint id = abs(selectedName()-1);
      if(id< myVoxelSetList.size()){
	cerr << "deleting list="<< id<<endl;
	myVoxelSetList.erase(myVoxelSetList.begin()+id);
	updateList(false);
      }else if (id< myVoxelSetList.size()+myLineSetList.size()){
	myLineSetList.erase(myLineSetList.begin()+(id-myVoxelSetList.size()));
	updateList(false);
      }else if (id< myPointSetList.size()+myLineSetList.size()+myVoxelSetList.size()){
	myPointSetList.erase(myPointSetList.begin()+(id-myVoxelSetList.size()-myLineSetList.size()));
	updateList(false);
      } 
      
    }
  }
  
}




void
DGtal::DGtalQGLViewer::updateList(bool updateBoundingBox)
{
  uint nbList= myVoxelSetList.size()+ myLineSetList.size()+ myPointSetList.size();
  glDeleteLists(myListToAff, myNbListe);
  myListToAff = glGenLists( nbList  );   
  myNbListe=0;
  
  uint listeID=0;
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
    glPushName(myNbListe);  
    glBegin(GL_QUADS);
      for (std::vector<voxelGL>::iterator s_it = myVoxelSetList.at(i).begin();
	   s_it != myVoxelSetList.at(i).end();
	   ++s_it){
	
	glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
	double width=(*s_it).width;
   
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
  glNewList(myListToAff+myVoxelSetList.size(), GL_COMPILE);
  myNbListe++;
  glPushName(myNbListe);  
  glEnable(GL_BLEND);   
  glEnable( GL_MULTISAMPLE_ARB );
  glEnable( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  

  glBegin(GL_QUADS);
    for (std::vector<quadGL>::iterator s_it = myKSSurfelList.begin();
       s_it != myKSSurfelList.end();
       ++s_it){
      
      glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
      double x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4;
      x1=(*s_it).x1; x2=(*s_it).x2; x3=(*s_it).x3; x4=(*s_it).x4;
      y1=(*s_it).y1; y2=(*s_it).y2; y3=(*s_it).y3; y4=(*s_it).y4;
      z1=(*s_it).z1; z2=(*s_it).z2; z3=(*s_it).z3; z4=(*s_it).z4;
      
      double dx, dy, dz;
      if(x1==x2 && x2==x3 && x1==x4){
	dx=0.03;
    }else dx=0;
    if(y1==y2 && y2==y3 && y1==y4){
      dy=0.03;
    }else dy=0;
    if(z1==z2 && z2==z3 && z1==z4){
      dz=0.03;
    }else dz=0;
    
    //main up face
    Vec normaleUp( dx!=0? 1.0:0.0, dy!=0 ? 1.0:0.0, dz!=0.0? 1.0:0.0);
    glNormal3f( normaleUp[0], normaleUp[1], normaleUp[2]);
    glVertex3f(x1+dx,  y1+dy, z1+dz);
    glVertex3f(x2+dx,  y2+dy, z2+dz);
    glVertex3f(x3+dx,  y3+dy, z3+dz);
    glVertex3f(x4+dx,  y4+dy, z4+dz);

    
    //small face 1
    Vec vF1 (x2-x1, y2-y1, z2-z1);
    Vec n1 = cross(vF1, normaleUp);
    n1.normalize();
    glNormal3f( n1[0], n1[1], n1[2]);
    
    glVertex3f(x1+dx,  y1+dy, z1+dz);
    glVertex3f(x2+dx,  y2+dy, z2+dz);
    glVertex3f(x2-dx,  y2-dy, z2-dz);
    glVertex3f(x1-dx,  y1-dy, z1-dz);

    //small face 2
    Vec vF2 (x3-x2, y3-y2, z3-z2);
    Vec n2 = cross(vF2, normaleUp);
    n2.normalize();
    glNormal3f( n2[0], n2[1], n2[2]);
    
    glVertex3f(x2+dx,  y2+dy, z2+dz);
    glVertex3f(x3+dx,  y3+dy, z3+dz);
    glVertex3f(x3-dx,  y3-dy, z3-dz);
    glVertex3f(x2-dx,  y2-dy, z2-dz);

    //small face 3
    Vec vF3 (x4-x3, y4-y3, z4-z3);
    Vec n3 = cross(vF3, normaleUp);
    n3.normalize();
    glNormal3f( n3[0], n3[1], n3[2]);
    
    glVertex3f(x3+dx,  y3+dy, z3+dz);
    glVertex3f(x4+dx,  y4+dy, z4+dz);
    glVertex3f(x4-dx,  y4-dy, z4-dz);
    glVertex3f(x3-dx,  y3-dy, z3-dz);

    //small face 4
    Vec vF4 (x1-x4, y1-y4, z1-z4);
    Vec n4 = cross(vF4, normaleUp);
    n4.normalize();
    glNormal3f( n4[0], n4[1], n4[2]);
    
     glVertex3f(x4+dx,  y4+dy, z4+dz);
     glVertex3f(x1+dx,  y1+dy, z1+dz);
     glVertex3f(x1-dx,  y1-dy, z1-dz);
     glVertex3f(x4-dx,  y4-dy, z4-dz);

    
     glNormal3f( -normaleUp[0], -normaleUp[1], -normaleUp[2]);
     glVertex3f(x1-dx,  y1-dy, z1-dz);
     glVertex3f(x2-dx,  y2-dy, z2-dz);
     glVertex3f(x3-dx,  y3-dy, z3-dz);
     glVertex3f(x4-dx,  y4-dy, z4-dz);
     
     
    
    }
    glEnd();
    glEndList();
  

  for (uint i=0; i<myLineSetList.size(); i++){  
    listeID++;
    glNewList(myListToAff+myVoxelSetList.size()+i+1, GL_COMPILE);
    myNbListe++;
    glDisable(GL_LIGHTING);
    glPushName(myNbListe);  
    glBegin(GL_LINES);      
    for (std::vector<lineGL>::iterator s_it = myLineSetList.at(i).begin();
	 s_it != myLineSetList.at(i).end();
	 ++s_it){

	glColor4ub((*s_it).R, (*s_it).G, (*s_it).B, (*s_it).T);
	glVertex3f((*s_it).x1,  (*s_it).y1, (*s_it).z1);
	glVertex3f((*s_it).x2,  (*s_it).y2, (*s_it).z2);
	
      }
      glEnd();
      glEnable(GL_LIGHTING);
      glEndList();
  
  }    


  for (uint i=0; i<myPointSetList.size(); i++){  
    glNewList(myListToAff+myLineSetList.size()+myVoxelSetList.size()+i+1, GL_COMPILE);
    myNbListe++;
    glDepthMask(GL_TRUE);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_LIGHTING);
    if(myPointSetList.at(i).size()!=0){
      glPointSize((*myPointSetList.at(i).begin()).size);
    }
    glPushName(myNbListe);  
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



  if( updateBoundingBox){
    setSceneBoundingBox(myBoundingPtLow, myBoundingPtUp);
    showEntireScene();
  }  
}




void
DGtal::DGtalQGLViewer::glDrawGLLinel(lineGL linel){
  glPushMatrix();
  glTranslatef(linel.x1, linel.y1, linel.z1);
  Vec dir (linel.x2-linel.x1, linel.y2-linel.y1, linel.z2-linel.z1 );
  glMultMatrixd(Quaternion(Vec(0,0,1), dir).matrix());
  GLUquadric* quadric = gluNewQuadric();
  glColor4ub(linel.R, linel.G, linel.B, linel.T);
  gluCylinder(quadric, linel.width, linel.width, dir.norm(), 10, 4);
  glPopMatrix();  
}




void 
DGtal::DGtalQGLViewer::glDrawGLPointel(pointGL pointel){
 glPushMatrix();
 glTranslatef(pointel.x, pointel.y, pointel.z);
 GLUquadric* quadric = gluNewQuadric();
 glColor4ub(pointel.R, pointel.G, pointel.B, pointel.T);
 gluSphere(quadric, pointel.size, 10, 10);
 glPopMatrix();  
  
}
  
 



void 
DGtal::DGtalQGLViewer::keyPressEvent(QKeyEvent *e){
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
  if( (e->key()==Qt::Key_L)){
    restoreStateFromFile();
    updateGL();
  }

  

  if (!handled)
    QGLViewer::keyPressEvent(e);

}


QString 
DGtal::DGtalQGLViewer::helpString() const
{
  QString text("<h2> DGtalQGLViewer</h2>");
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
