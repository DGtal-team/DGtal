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

#pragma once

/**
 * @file OFFReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/06/29
 *
 * Header file for module OFFReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(OFFReader_RECURSES)
#error Recursive header files inclusion detected in OFFReader.h
#else // defined(OFFReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OFFReader_RECURSES

#if !defined OFFReader_h
/** Prevents repeated inclusion of headers. */
#define OFFReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <DGtal/kernel/SpaceND.h>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{



/////////////////////////////////////////////////////////////////////////////
// class OFFMesh
/**
 * Description of class 'OFFReader' <p> \brief Aim: Defined to import
 * OFF surface mesh. It allows to import a MeshFromPoints object but do not takes
 * into accouts the optional color faces.
 * 
 * Example of typical use:
 * @code
 // importating the mesh
 MeshFromPoints<Display3D::pointD3D> anImportedMesh;
 OFFReader<Display3D::pointD3D>::importOFFFile(inputFilename, anImportedMesh, true);
 //Displaying the resulting mesh with Viewed3D 
 viewer.setFillColor(DGtal::Color(240,240,240,150));
 viewer.setLineColor(DGtal::Color(150,0,0,254));
 viewer << anImportedMesh;
 viewer << Viewer3D::updateDisplay;
 @endcode
 *
 *
 * \todo Process the face color information.
 * @see MeshFromPoints
 */




template <typename TPoint> 
struct OFFReader
{
    // ----------------------- Standard services ------------------------------
public:

  
 /** 
  * Main method to import OFF meshes file (Geomview Object File Format) 
  * 
  * @param filename the file name to import.
  * @param aMesh: (return) the mesh object to be imported.
  * @param invertVertexOrder: used to invert (default value=false) the order of imported points (important for normal orientation). 
  * @return an instance of the imported mesh: MeshFromPoint.
  */
  
  static  bool  importOFFFile(const std::string & filename, 
			      MeshFromPoints<TPoint> & aMesh, bool invertVertexOrder=false) throw(DGtal::IOException);
  

  

  


}; // end of class OFFReader









}// namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/OFFReader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined OFFReader_h

#undef OFFReader_RECURSES
#endif // else defined(OFFReader_RECURSES)
