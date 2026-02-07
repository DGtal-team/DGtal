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
#include <map>
#include <string>
#include <DGtal/kernel/SpaceND.h>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Mesh.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{



/////////////////////////////////////////////////////////////////////////////
// class MeshReader
/**
 * Description of class 'MeshReader' <p>
 * \brief Aim: Defined to import
 * OFF and OFS surface mesh. It allows to import a Mesh object and takes
 * into accounts the optional color faces.
 *
 * The importation can be done automatically according the input file
 * extension with the operator <<
 *
 * Example of typical use:
 Add these include files:
 @code
 #include "DGtal/shapes/Mesh.h"
 #include "DGtal/io/readers/MeshReader.h"
 @endcode


 And automatic import the Mesh through the filename extension:
 @code
  std::string filenameOFF = testPath + "samples/box.off";
  Mesh<Point> a3DMesh;
  bool importOK = a3DMesh << filenameOFF;
@endcode

 Then you can also display the resulting with a PolyscopeViewer:
 @code
 viewer << a3DMesh;
 @endcode
 *
 *
 * @see Mesh MeshWriter
 */




  template <typename TPoint>
struct MeshReader
{
    // ----------------------- Standard services ------------------------------
public:


 /**
  * Main method to import OFF meshes file (Geomview Object File Format)
  *
  * @param[in] filename the file name to import.
  * @param[out] aMesh the mesh object to be imported.
  * @param[in] invertVertexOrder used to invert (default value=false) the order of imported points (important for normal
  * @param[in] onlyFaceVertex flag used to import only vertces associated to a face.
  * @return an instance of the imported mesh: MeshFromPoint.
  */

  static  bool  importOFFFile(const std::string & filename,
			      DGtal::Mesh<TPoint> & aMesh, bool invertVertexOrder=false,
			      bool onlyFaceVertex=false);




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
			      DGtal::Mesh<TPoint> & aMesh, bool invertVertexOrder=false, double scale=1.0);

  /// Reads an input file as an OBJ file format and outputs the
  /// corresponding surface mesh.
  ///
  /// @param[in] filename the input filename of the OBJ file to be read.
  /// @param[out] aMesh the output  mesh.
  /// @param[in] onlyFaceVertex flag used to import only vertces associated to a face.
  ///
  /// @return 'true' if both reading the input stream was ok and the
  /// created mesh is ok.
  static
  bool importOBJFile(const std::string & filename, DGtal::Mesh<TPoint> & aMesh, bool onlyFaceVertex=false );


  /// Checks that each index in \a indices is unique.
  /// @param indices a vector of integer indices
  /// @return 'true' iff the integer indices are all pairwise different.
  static
  bool verifyIndicesUniqueness( const std::vector<
                                typename DGtal::Mesh<TPoint>::Index > &indices );


  /// Reads an input material file associated ot an OBJ file format and outputs the
  /// corresponding materials as a map assiating name and diffuse color.
  ///
  /// @param[in,out] input the input stream where the OBJ file is read.
  ///
  /// @return 'true' if both reading the input stream was ok and the
  /// created mesh is ok.
  static
  std::map<std::string, DGtal::Color> readMaterial( std::istream & input);



}; // end of class MeshReader






  /**
   *  'operator<<' for exporting objects of class 'Mesh'.
   * @param mesh a mesh
   * @param filename a filename
   * @return true if the mesh has been exported correctly.
   */
  template <typename TPoint>
  bool
  operator<< (  Mesh<TPoint> & mesh, const std::string &filename );




}// namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/MeshReader.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshReader_h

#undef MeshReader_RECURSES
#endif // else defined(MeshReader_RECURSES)
