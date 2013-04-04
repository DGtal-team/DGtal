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
 * @file   Board3DTo2D.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 22 juin 2011
 * 
 * @brief
 *
 * Class for PDF, PNG, PS, EPS, SVG export drawings with Cairo with 3D->2D projection
 * This is the implementation of methods defined in Board3DTo2D.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/boards/Board3DTo2D.h"
#include <limits>



// Cairo includes
#include <cairo.h>
#include <cairo-pdf.h>
#include <cairo-ps.h>
#include <cairo-svg.h>
// Cairo includes
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class Board3DTo2D
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/*!
 * \brief Constructor.
 */
DGtal::Board3DTo2D::Board3DTo2D()
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
DGtal::Board3DTo2D::selfDisplay ( std::ostream & out ) const
{
    out << "[Board3DTo2D]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::Board3DTo2D::isValid() const
{
    return true;
}


/**
 * Transpose a 4x4 matrix.
 * @param tmat destination matrix.
 * @param mat source matrix.
 */
void 
DGtal::Board3DTo2D::TransposeMt(double tmat[16], double mat[16])
{
    tmat[0] = mat[0]; tmat[1] = mat[4]; tmat[2] = mat[8]; tmat[3] = mat[12];
    tmat[4] = mat[1]; tmat[5] = mat[5]; tmat[6] = mat[9]; tmat[7] = mat[13];
    tmat[8] = mat[2]; tmat[9] = mat[6]; tmat[10] = mat[10]; tmat[11] = mat[14];
    tmat[12] = mat[3]; tmat[13] = mat[7]; tmat[14] = mat[11]; tmat[15] = mat[15];
}

/**
 * Multiply a 3d vector by a 4x4 matrix.
 * @param v destination vector.
 * @param mat source matrix.
 * @param b source vector.
 */
void  
DGtal::Board3DTo2D::MulMt(double v[4], double mat[16], double b[4])
{
    v[0] = mat[0] * b[0] + mat[1] * b[1] + mat[2] * b[2] + mat[3] * b[3];
    v[1] = mat[4] * b[0] + mat[5] * b[1] + mat[6] * b[2] + mat[7] * b[3];
    v[2] = mat[8] * b[0] + mat[9] * b[1] + mat[10] * b[2] + mat[11] * b[3];
    v[3] = mat[12] * b[0] + mat[13] * b[1] + mat[14] * b[2] + mat[15] * b[3];
}

/**
 * Compute 4x4 LookAt matrix.
 * @param mat destination matrix.
 * @param eyex x position of eye.
 * @param eyey y position of eye.
 * @param eyez z position of eye.
 * @param dirx x direction of eye.
 * @param diry y direction of eye.
 * @param dirz z director of eye.
 * @param upx x coordinate of up-vector.
 * @param upy y coordinate of up-vector.
 * @param upz z coordinate of up-vector.
 */
void  
DGtal::Board3DTo2D::LookAtMt(double mat[16],
                             double eyex, double eyey, double eyez,
                             double dirx, double diry, double dirz,
                             double upx, double upy, double upz)
{
    double up[3]; up[0]= upx; up[1]= upy; up[2]= upz;
    
    double z[3]; z[0]= -dirx; z[1]= -diry; z[2]= -dirz; normalize(z);
    double x[3]; cross (x, up, z); normalize(x);
    double y[3]; cross (y, z, x); normalize(y);
    
    double m[16];
    m[0] = x[0]; m[1] = x[1]; m[2] = x[2]; m[3] = 0;
    m[4] = y[0]; m[5] = y[1]; m[6] = y[2]; m[7] = 0;
    m[8] = z[0]; m[9] = z[1]; m[10] = z[2]; m[11] = 0;
    m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
      
    double e[4]; e[0]= -eyex; e[1]= -eyey; e[2]= -eyez; e[3]= 1;
    double eyePrime[4]; MulMt(eyePrime, m, e);
    
    TransposeMt(mat, m);
    mat[12] = eyePrime[0]; mat[13] = eyePrime[1]; mat[14] = eyePrime[2];
}

/**
 * Precompute 4x4 projection matrix for 3D->2D projection.
 */
void DGtal::Board3DTo2D::precompute_projection_matrix()
{
      // Projection: from qglviewer
      /*const double f = 1.0/tan(fieldOfView()/2.0);
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
      
      double Modelview[16];
      LookAtMt(Modelview,
    camera_position[0], camera_position[1], camera_position[2],
    camera_direction[0], camera_direction[1], camera_direction[2],
    camera_upVector[0], camera_upVector[1], camera_upVector[2]);

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
}

/**
 * Project a 3d point (3D->2D).
 * @param x3d x position of the 3d point.
 * @param y3d y position of the 3d point.
 * @param z3d z position of the 3d point.
 * @param x2d x destination projection position of the 2d point.
 * @param y2d y destination projection position of the 2d point.
 */
void DGtal::Board3DTo2D::project(double x3d, double y3d, double z3d, double &x2d, double &y2d)
{
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
      
      x2d = vs[0];
      y2d = Viewport[3]-vs[1];
}

/**
 * Save a Cairo image.
 * @param filename filename of the image to save.
 * @param type type of the image to save (CairoPDF, CairoPNG, CairoPS, CairoEPS, CairoSVG).
 * @param bWidth width of the image to save.
 * @param bHeight height of the image to save.
 */
void
DGtal::Board3DTo2D::saveCairo(const char *filename, CairoType type, int bWidth, int bHeight)
{
  for(unsigned int i =0; i< myClippingPlaneList.size(); i++)
    trace.info() << "-> ClippingPlane not implemented in Board3DTo2D" << std::endl;
   
  Viewport[0] = 0; Viewport[1] = 0; Viewport[2] = bWidth; Viewport[3] = bHeight;
  precompute_projection_matrix();
  
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
    for (std::vector<pointD3D>::iterator s_it = myPointSetList.at(i).begin();
   s_it != myPointSetList.at(i).end();
   ++s_it)
  {
    {
        cairo_save (cr);
        
    cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/255.0);
    cairo_set_line_width (cr, 1.); // arbitraire car non set
    
    double x1, y1, x2, y2, x3, y3, x4, y4;
    //double width=(*s_it).size/120.; // arbitraire
    double width=0.05; // arbitraire
    // TODO:
    /*double distCam =sqrt((camera_position[0]-centerS.x)*(camera_position[0]-centerS.x)+
      (camera_position[1]-centerS.y)*(camera_position[1]-centerS.y)+
      (camera_position[2]-centerS.z)*(camera_position[2]-centerS.z));*/
    
    //z+
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //z-
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //x+
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //x-
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //y+
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //y-
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
        
        cairo_restore (cr);
    }
  }
  }
  
  // myLineSetList
  for(unsigned int i=0; i<myLineSetList.size(); i++)
  {
    for (std::vector<lineD3D>::iterator s_it = myLineSetList.at(i).begin();
   s_it != myLineSetList.at(i).end();
   ++s_it)
  {
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

    cairo_stroke (cr);
        
        cairo_restore (cr);
    }
  }
  }
  
  // myVoxelSetList
  for(unsigned int i=0; i<myVoxelSetList.size(); i++)
  {
    for (std::vector<voxelD3D>::iterator s_it = myVoxelSetList.at(i).begin();
     s_it != myVoxelSetList.at(i).end();
     ++s_it)
  {
    {
        cairo_save (cr);
        
    if (myModes["Board3DTo2D"]=="SolidMode")
      cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/(255.0*1.75)); // *1.75 arbitraire
    else
      cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/(255.0*0.75)); // *0.75 arbitraire
      
    cairo_set_line_width (cr, 1.); // arbitraire car non set
    
    double x1, y1, x2, y2, x3, y3, x4, y4;
    double width=(*s_it).width;
   
    //z+
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //z-
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //x+
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //x-
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //y+
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y+width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y+width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
    //y-
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z+width, x1, y1);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z+width, x2, y2);
    project((*s_it).x+width,  (*s_it).y-width, (*s_it).z-width, x3, y3);
    project((*s_it).x-width,  (*s_it).y-width, (*s_it).z-width, x4, y4);
    cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
        
        cairo_restore (cr);
    }
  }
  }
  
  for(unsigned int i=0; i<myQuadList.size(); i++)
    trace.info() << "-> Quad not YET implemented in Board3DTo2D" << std::endl;
  
  // Drawing all Khalimsky Space Cells 
  
  // KSSurfel (from updateList)
  for (std::vector<quadD3D>::iterator s_it = myKSSurfelList.begin();
       s_it != myKSSurfelList.end();
       ++s_it)
  {
    {
      cairo_save (cr);

      if (myModes["Board3DTo2D"]=="SolidMode")
	cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/(255.0*3.75)); // *3.75 arbitraire
      else
	cairo_set_source_rgba (cr, (*s_it).R/255.0, (*s_it).G/255.0, (*s_it).B/255.0, (*s_it).T/(255.0*0.75)); // *0.75 arbitraire
	
      cairo_set_line_width (cr, 1.); // arbitraire car non set

      double x1, y1, x2, y2, x3, y3, x4, y4;

      project((*s_it).x1,  (*s_it).y1, (*s_it).z1, x1, y1);
      project((*s_it).x2,  (*s_it).y2, (*s_it).z2, x2, y2);
      project((*s_it).x3,  (*s_it).y3, (*s_it).z3, x3, y3);
      project((*s_it).x4,  (*s_it).y4, (*s_it).z4, x4, y4);
      cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);

      cairo_restore (cr);
    }
  }
  
  // KSLinel
  for(unsigned int i=0; i< myKSLinelList.size();i++)
  {
    //if (!myKSLinelList.at(i).isSigned) // for the moment, same for Signed or NonSigned
    {
      {
        cairo_save (cr);
        
	cairo_set_source_rgba (cr, myKSLinelList.at(i).R/255.0, myKSLinelList.at(i).G/255.0, myKSLinelList.at(i).B/255.0, myKSLinelList.at(i).T/255.0);
	cairo_set_line_width (cr, 4.); // arbitraire car non set
	
	double x1, y1, x2, y2;
	
	project(myKSLinelList.at(i).x1,  myKSLinelList.at(i).y1, myKSLinelList.at(i).z1, x1, y1);
	project(myKSLinelList.at(i).x2,  myKSLinelList.at(i).y2, myKSLinelList.at(i).z2, x2, y2);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_close_path (cr); cairo_stroke (cr);
	    
	cairo_restore (cr);
      }
    }
  }
  
  // KSPointel
  for(unsigned int i=0; i< myKSPointelList.size();i++)
  {
    //if (!myKSPointelList.at(i).isSigned) // for the moment, same for Signed or NonSigned
    {
      {
        cairo_save (cr);
        
	cairo_set_source_rgba (cr, myKSPointelList.at(i).R/255.0, myKSPointelList.at(i).G/255.0, myKSPointelList.at(i).B/255.0, myKSPointelList.at(i).T/255.0);
	cairo_set_line_width (cr, 1.); // arbitraire car non set
	
	double x1, y1, x2, y2, x3, y3, x4, y4;
	//double width=myKSPointelList.at(i).size/120.; // arbitraire
	double width=0.04; // arbitraire
	
	//z+
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x1, y1);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x2, y2);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x3, y3);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	//z-
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x1, y1);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x2, y2);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x3, y3);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	//x+
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x1, y1);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x2, y2);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x3, y3);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	//x-
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x1, y1);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x2, y2);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x3, y3);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	//y+
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x1, y1);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z+width, x2, y2);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x3, y3);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y+width, myKSPointelList.at(i).z-width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	//y-
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x1, y1);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z+width, x2, y2);
	project(myKSPointelList.at(i).x+width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x3, y3);
	project(myKSPointelList.at(i).x-width,  myKSPointelList.at(i).y-width, myKSPointelList.at(i).z-width, x4, y4);
	cairo_move_to (cr, x1, y1); cairo_line_to (cr, x2, y2); cairo_line_to (cr, x3, y3); cairo_line_to (cr, x4, y4); cairo_line_to (cr, x1, y1); cairo_close_path (cr); myModes["Board3DTo2D"]=="SolidMode"?cairo_fill (cr):cairo_stroke (cr);
	    
	cairo_restore (cr);
      }
    }
  }
  
  if (type==CairoPNG)
    cairo_surface_write_to_png (surface, filename);
      
  cairo_destroy (cr);
  cairo_surface_destroy (surface);
}

/*!
 * \brief init function (should be in Constructor).
 */
void
DGtal::Board3DTo2D::init()
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
  
  //
  
  camera_position[0] = 5.000000; camera_position[1] = 5.000000; camera_position[2] = 29.893368;
  camera_direction[0] = 0.000000; camera_direction[1] = 0.000000; camera_direction[2] = -1.000000;
  camera_upVector[0] = 0.000000; camera_upVector[1] = 1.000000; camera_upVector[2] = 0.000000;
  
  ZNear = 0.001;
  ZFar = 100.0;
  //ZNear = 4.578200;
  //ZFar = 22.578199;
  
  myModes["Board3DTo2D"]="SolidMode";
}

