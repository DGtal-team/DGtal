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
 * @file MeshReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/06/29
 *
 * Header file for module MeshReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(MeshReader_RECURSES)
#error Recursive header files inclusion detected in MeshReader.h
#else // defined(MeshReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeshReader_RECURSES

#if !defined MeshReader_h
/** Prevents repeated inclusion of headers. */
#define MeshReader_h

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
 * Description of class 'MeshReader' <p> 
 * \brief Aim: Defined to import
 * OFF and OFS surface mesh. It allows to import a MeshFromPoints object and takes
 * into accouts the optional color faces.
 * 
 * The importation can be done automatically according the input file
 * extension with the operator << 
 * 
 * Example of typical use: 
 Add these include files:
 @snippet tests/io/readers/testMeshReader.cpp MeshReaderUseIncludes

 And automatic import the MeshFromPoints through the filename extension:
 
 @snippet tests/io/readers/testMeshReader.cpp MeshReaderUseImport
 
 Then you can also display the resulting with a Viewer3D:
 @code
 viewer.setLineColor(DGtal::Color(150,0,0,254));
 viewer << a3DMesh;
 viewer << Viewer3D::updateDisplay;
 @endcode
 *
 *
 * @see MeshFromPoints MeshWriter
 */




  template <typename TPoint> 
struct MeshReader
{
    // ----------------------- Standard services ------------------------------
public:

  
 /** 
  * Main method to import OFF meshes file (Geomview Object File Format) 
  * 
  * @param filename the file name to import.
  * @param aMesh (return) the mesh object to be imported.
  * @param invertVertexOrder used to invert (default value=false) the order of imported points (important for normal orientation). 
  * @return an instance of the imported mesh: MeshFromPoint.
  */
  
  static  bool  importOFFFile(const std::string & filename, 
			      DGtal::MeshFromPoints<TPoint> & aMesh, bool invertVertexOrder=false) throw(DGtal::IOException);
  

  

 /** 
  * Main method to import OFS meshes file (an equivalent of OFF format) 
  * 
  * @param filename the file name to import.
  * @param aMesh (return) the mesh object to be imported.
  * @param invertVertexOrder used to invert (default value=false) the order of imported points (important for normal orientation). 
  * @param scale used to avoid to display tiny shapes (since OFS shapes are generally included in a 1x1x1 cube)
  * @return an instance of the imported mesh: MeshFromPoint.
  */
  
  static  bool  importOFSFile(const std::string & filename, 
			      DGtal::MeshFromPoints<TPoint> & aMesh, bool invertVertexOrder=false, double scale=1.0) throw(DGtal::IOException);
  
  
  




}; // end of class MeshReader






  /**
   *  'operator<<' for exporting objects of class 'MeshFromPoints'.
   * @param mesh a mesh 
   * @param filename a filename 
   * @return true if the mesh has been exported correctly.
   */
  template <typename TPoint>
  bool
  operator<< (  MeshFromPoints<TPoint> & mesh, const std::string &filename );
  
  


}// namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/MeshReader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshReader_h

#undef MeshReader_RECURSES
#endif // else defined(MeshReader_RECURSES)
