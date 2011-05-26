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
 * @file   dgtalCairo.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 25 mai 2011
 * 
 * @brief
 *
 * Implementation of methods defined in DGtalCairo.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io-viewers/CairoViewers/DGtalCairo.h"
#include <limits>
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/io-viewers/CairoViewers/DGtalCairo.ih"
#endif

// cairo
#include <cairo.h>
#include <cairo-pdf.h>
#include <cairo-ps.h>
#include <cairo-svg.h>
// cairo
#include "Matrix.hpp"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
//using namespace qglviewer;


///////////////////////////////////////////////////////////////////////////////
// class DGtalCairo
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
DGtalCairo::DGtalCairo() // MT
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
DGtal::DGtalCairo::selfDisplay ( std::ostream & out ) const
{
    out << "[DGtalCairo]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::DGtalCairo::isValid() const
{
    return true;
}






void
DGtal::DGtalCairo::drawWithNames()
{    
  /*for(unsigned int i=0; i<myVoxelSetList.size(); i++){
    glCallList(myListToAff+i);
  }
  for(unsigned int i=0; i<myLineSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+i);
  }
  
  for(unsigned int i=0; i<myPointSetList.size(); i++){
    glCallList(myListToAff+myVoxelSetList.size()+myLineSetList.size()+i);
  }*/
}

// http://www.libqglviewer.com/refManual/classqglviewer_1_1Camera.html#ac4dc649d17bd2ae8664a7f4fdd50360f
// http://www.songho.ca/opengl/gl_projectionmatrix.html
void DGtal::DGtalCairo::project(double x3d, double y3d, double z3d, double &x2d, double &y2d)
{
      //GLint    Viewport[4];
      //GLdouble Projection[16], Modelview[16];

      // Precomputation begin
      
      //glGetIntegerv(GL_VIEWPORT         , Viewport);
      //glGetDoublev (GL_MODELVIEW_MATRIX , Modelview);
      //glGetDoublev (GL_PROJECTION_MATRIX, Projection);

      double matrix[16];
      
      // Projection: from qglviewer
      /*const float f = 1.0/tan(fieldOfView()/2.0);
      projectionMatrix_[0]  = f/aspectRatio();
      projectionMatrix_[5]  = f;
      projectionMatrix_[10] = (ZNear + ZFar) / (ZNear - ZFar);
      projectionMatrix_[11] = -1.0;
      projectionMatrix_[14] = 2.0 * ZNear * ZFar / (ZNear - ZFar);
      projectionMatrix_[15] = 0.0;
      // same as gluPerspective( 180.0*fieldOfView()/M_PI, aspectRatio(), zNear(), zFar() );*/
      
      double fieldOfView = M_PI/4.;
      double f = 1.0/tan(fieldOfView/2.0);      
      double aspectRatio = (double)Viewport[2]/Viewport[3];
      
      double Projection[16] = { f/aspectRatio, 0.00, 0.00, 0.00, 
			      0.00, f, 0.00, 0.00, 
			      0.00, 0.00, (ZNear + ZFar) / (ZNear - ZFar), -1.00, 
			      0.00, 0.00, 2.0 * ZNear * ZFar / (ZNear - ZFar), 0.00 };
      /*fprintf(stdout, "Projection:\n");
      for (unsigned short m=0; m<4; ++m)
      {
	      for (unsigned short l=0; l<4; ++l)
	      {
		      fprintf(stdout, "%2.2lf, ", Projection[l+4*m]);
	      }
	      fprintf(stdout, "\n");
      }
      fprintf(stdout, "\n");*/
      
      // http://iphone-3d-programming.labs.oreilly.com/apa.html
      vec3 eye(camera_position[0], camera_position[1], camera_position[2]);
      vec3 dir(camera_direction[0], camera_direction[1], camera_direction[2]);
      vec3 up(camera_upVector[0], camera_upVector[1], camera_upVector[2]);
      mat4 mv = mv.LookAtMt(eye, dir, up);
      
      double Modelview[16];
      Modelview[0] = mv.x.x; Modelview[1] = mv.x.y; Modelview[2] = mv.x.z; Modelview[3] = mv.x.w;
      Modelview[4] = mv.y.x; Modelview[5] = mv.y.y; Modelview[6] = mv.y.z; Modelview[7] = mv.y.w;
      Modelview[8] = mv.z.x; Modelview[9] = mv.z.y; Modelview[10] = mv.z.z; Modelview[11] = mv.z.w;
      Modelview[12] = mv.w.x; Modelview[13] = mv.w.y; Modelview[14] = mv.w.z; Modelview[15] = mv.w.w;
      /*fprintf(stdout, "Modelview:\n");
      for (unsigned short m=0; m<4; ++m)
      {
	      for (unsigned short l=0; l<4; ++l)
	      {
		      fprintf(stdout, "%2.2lf, ", Modelview[l+4*m]);
	      }
	      fprintf(stdout, "\n");
      }
      fprintf(stdout, "\n");*/

      for (unsigned short m=0; m<4; ++m)
      {
	      for (unsigned short l=0; l<4; ++l)
	      {
		      double sum = 0.0;
		      for (unsigned short k=0; k<4; ++k)
			      sum += Projection[l+4*k]*Modelview[k+4*m];
		      matrix[l+4*m] = sum;
	      }
      }
      
      // Precomputation end
	      
      double v[4], vs[4];
      v[0]=x3d; v[1]=y3d; v[2]=z3d; v[3]=1.0;

      vs[0]=matrix[0 ]*v[0] + matrix[4 ]*v[1] + matrix[8 ]*v[2] + matrix[12 ]*v[3];
      vs[1]=matrix[1 ]*v[0] + matrix[5 ]*v[1] + matrix[9 ]*v[2] + matrix[13 ]*v[3];
      vs[2]=matrix[2 ]*v[0] + matrix[6 ]*v[1] + matrix[10]*v[2] + matrix[14 ]*v[3];
      vs[3]=matrix[3 ]*v[0] + matrix[7 ]*v[1] + matrix[11]*v[2] + matrix[15 ]*v[3];

      vs[0] /= vs[3];
      vs[1] /= vs[3];
      vs[2] /= vs[3];

      vs[0] = vs[0] * 0.5 + 0.5;
      vs[1] = vs[1] * 0.5 + 0.5;
      vs[2] = vs[2] * 0.5 + 0.5;

      vs[0] = vs[0] * Viewport[2] + Viewport[0];
      vs[1] = vs[1] * Viewport[3] + Viewport[1];

      //return Vec(vs[0], Viewport[3]-vs[1], vs[2]);
      x2d = vs[0];
      y2d = Viewport[3]-vs[1];
}
  
void
DGtal::DGtalCairo::saveCairo(const char *filename, CairoType type, int width, int height)
{
  /*glPushMatrix();
  glMultMatrixd(manipulatedFrame()->matrix());
  for(unsigned int i =0; i< myClippingPlaneList.size(); i++){
    clippingPlaneGL cp = myClippingPlaneList.at(i);
    double eq [4];
    eq[0]=cp.a;
    eq[1]=cp.b;
    eq[2]=cp.c;
    eq[3]=cp.d;
    glEnable(GL_CLIP_PLANE0+i); 
    glClipPlane(GL_CLIP_PLANE0+i, eq );
  }  
  glPopMatrix();*/
   
  Viewport[0] = 0; Viewport[1] = 0; Viewport[2] = width; Viewport[3] = height;
  
  cairo_surface_t *surface;
  cairo_t *cr;
  
  switch (type)
  {
    case CairoPDF:
      surface = cairo_pdf_surface_create (filename, Viewport[2], Viewport[3]); break;
    case CairoPS:
      surface = cairo_ps_surface_create (filename, Viewport[2], Viewport[3]); break;
    case CairoEPS:
      surface = cairo_ps_surface_create (filename, Viewport[2], Viewport[3]); 
      cairo_ps_surface_set_eps(surface, true); break;
    case CairoSVG:
      surface = cairo_svg_surface_create (filename, Viewport[2], Viewport[3]); break;
    case CairoPNG:
    default:
      surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, Viewport[2], Viewport[3]);
  }
  
  cr = cairo_create (surface);
  
  // Fill the background with gray
  cairo_set_source_rgba (cr, .3, .3, .3, 1.);
  cairo_rectangle (cr, 0, 0, Viewport[2], Viewport[3]);
  cairo_fill (cr);
  
  // Draw the shapes
  
  // myPointSetList
  for(unsigned int i=0; i<myPointSetList.size(); i++)
  {
    //fprintf(stdout, " -> myPointSetList\n");
    for (std::vector<pointGL>::iterator s_it = myPointSetList.at(i).begin();
	 s_it != myPointSetList.at(i).end();
	 ++s_it)
	{
	  //fprintf(stdout, " -------> Point\n");
	  {
	      cairo_save (cr);
	      
		cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/255.0);
		
		double x1, y1, x2, y2, x3, y3, x4, y4;
		double width=(*s_it).size/120.; // arbitraire
		
		//z+
		//glNormal3f( 0.0, 0.0, 1.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//z-
		//glNormal3f( 0.0, 0.0, -1.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//x+
		//glNormal3f( 1.0, 0.0, 0.0);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//x-
		//glNormal3f( -1.0, 0.0, 0.0);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//y+
		//glNormal3f( 0.0, 1.0, 0.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//y-
		//glNormal3f( 0.0, -1.0, 0.0);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
	      
	      cairo_restore (cr);
	  }
	}
  }
  
  // myLineSetList
  for(unsigned int i=0; i<myLineSetList.size(); i++)
  {
    //fprintf(stdout, " -> myLineSetList\n");
    for (std::vector<lineGL>::iterator s_it = myLineSetList.at(i).begin();
	 s_it != myLineSetList.at(i).end();
	 ++s_it)
	{
	  //fprintf(stdout, " -------> Line\n");
	  {
	      cairo_save (cr);
	      
		cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/255.0);
		
		double x1, y1;
		double x2, y2;
		project((*s_it).x1, (*s_it).y1, (*s_it).z1, x1, y1);
		project((*s_it).x2, (*s_it).y2, (*s_it).z2, x2, y2);
		cairo_move_to (cr, x1, y1);
		cairo_line_to (cr, x2, y2);
		
		//cairo_set_line_width (cr, (*s_it).width);
		cairo_set_line_width (cr, 1.); // arbitraire car non set
		
		//cairo_set_line_cap (cr, cairoLineCap[_lineCap]);
		//cairo_set_line_join (cr, cairoLineJoin[_lineJoin]);
		//setCairoDashStyle (cr, _lineStyle);

		cairo_stroke (cr);
	      
	      cairo_restore (cr);
	  }
	}
  }
  
  // myVoxelSetList
  for(unsigned int i=0; i<myVoxelSetList.size(); i++)
  {
    //fprintf(stdout, " -> myVoxelSetList\n");
    for (std::vector<voxelGL>::iterator s_it = myVoxelSetList.at(i).begin();
	   s_it != myVoxelSetList.at(i).end();
	   ++s_it)
	{
	  //fprintf(stdout, " -------> Voxel\n");
	  {
	      cairo_save (cr);
	      
		cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/(255.0*1.75)); // *1.75 arbitraire
		
		double x1, y1, x2, y2, x3, y3, x4, y4;
		double width=(*s_it).width;
   
		//z+
		//glNormal3f( 0.0, 0.0, 1.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//z-
		//glNormal3f( 0.0, 0.0, -1.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//x+
		//glNormal3f( 1.0, 0.0, 0.0);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//x-
		//glNormal3f( -1.0, 0.0, 0.0);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//y+
		//glNormal3f( 0.0, 1.0, 0.0);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
		//y-
		//glNormal3f( 0.0, -1.0, 0.0);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x2, y2);
		project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
		project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
		cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); cairo_fill (cr);
	      
	      cairo_restore (cr);
	  }
	}
  }
  
  if (type==CairoPNG)
    cairo_surface_write_to_png (surface, filename);
      
  cairo_destroy (cr);
  cairo_surface_destroy (surface);
  
  /*for(unsigned int i=0; i<myQuadList.size(); i++){
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
  }*/

  
  // Drawing all Khalimsky Space Cells 
  
  /*for(unsigned int i=0; i< myKSPointelList.size();i++){
    glDrawGLPointel(myKSPointelList.at(i));
  }
  for(unsigned int i=0; i< myKSLinelList.size();i++){
    glDrawGLLinel(myKSLinelList.at(i));
  }*/
}



void
DGtal::DGtalCairo::init()
{
  myNbListe=0;
  createNewVoxelList(true);
  vector<lineGL> listeLine;
  myLineSetList.push_back(listeLine);
  vector<pointGL> listePoint;
  myPointSetList.push_back(listePoint);
  myCurrentFillColor = QColor (220, 220, 220);
  myCurrentLineColor = QColor (22, 22, 222, 50);

  myIsBackgroundDefault=true;
  createNewVoxelList(true);
  std::vector<voxelGL>  aKSVoxelList;
  
  myDefaultColor= QColor(255, 255, 255);
  
  // MT
  camera_position[0] = 5.000000; camera_position[1] = 5.000000; camera_position[2] = 29.893368;
  camera_direction[0] = 0.000000; camera_direction[1] = 0.000000; camera_direction[2] = -1.000000;
  camera_upVector[0] = 0.000000; camera_upVector[1] = 1.000000; camera_upVector[2] = 0.000000;
  
  ZNear = 0.001;
  ZFar = 100.0;
  //ZNear = 4.578200;
  //ZFar = 22.578199;
}



void 
DGtal::DGtalCairo::sortSurfelFromCamera()
{
  /*compFarthestFromCamera comp;
  comp.posCam= camera()->position();
  for(unsigned int i=0; i<myVoxelSetList.size(); i++){
    sort(myVoxelSetList.at(i).begin(), myVoxelSetList.at(i).end(), comp);
  }*/
}




void
DGtal::DGtalCairo::updateList(bool updateBoundingBox)
{
  /*unsigned int nbList= myVoxelSetList.size()+ myLineSetList.size()+ myPointSetList.size();
  glDeleteLists(myListToAff, myNbListe);
  myListToAff = glGenLists( nbList  );   
  myNbListe=0;
  
  unsigned int listeID=0;
  glEnable(GL_BLEND);   
  glEnable( GL_MULTISAMPLE_ARB );
  glEnable( GL_SAMPLE_ALPHA_TO_COVERAGE_ARB );
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  

  for (unsigned int i=0; i<myVoxelSetList.size(); i++){  

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
    
    //main down face
    glNormal3f( -normaleUp[0], -normaleUp[1], -normaleUp[2]);
    glVertex3f(x1-dx,  y1-dy, z1-dz);
    glVertex3f(x2-dx,  y2-dy, z2-dz);
    glVertex3f(x3-dx,  y3-dy, z3-dz);
    glVertex3f(x4-dx,  y4-dy, z4-dz);
     
     
    
    }
    glEnd();
    glEndList();
  

  for (unsigned int i=0; i<myLineSetList.size(); i++){  
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


  for (unsigned int i=0; i<myPointSetList.size(); i++){  
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
  }*/
}




void
DGtal::DGtalCairo::glDrawGLLinel(lineGL linel)
{
  /*glPushMatrix();
  glTranslatef(linel.x1, linel.y1, linel.z1);
  Vec dir (linel.x2-linel.x1, linel.y2-linel.y1, linel.z2-linel.z1 );
  glMultMatrixd(Quaternion(Vec(0,0,1), dir).matrix());
  GLUquadric* quadric = gluNewQuadric();
  glColor4ub(linel.R, linel.G, linel.B, linel.T);
  gluCylinder(quadric, linel.width, linel.width, dir.norm(), 10, 4);
  glPopMatrix();*/
}




void 
DGtal::DGtalCairo::glDrawGLPointel(pointGL pointel)
{
 /*glPushMatrix();
 glTranslatef(pointel.x, pointel.y, pointel.z);
 GLUquadric* quadric = gluNewQuadric();
 glColor4ub(pointel.R, pointel.G, pointel.B, pointel.T);
 gluSphere(quadric, pointel.size, 10, 10);
 glPopMatrix();*/
  
}


///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
