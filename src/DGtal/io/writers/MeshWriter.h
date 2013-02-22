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
 * @file  MeshWriter.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/07/08
 *
 * Header file for module  MeshWriter
 *
 * This file is part of the DGtal library.
 */

#if defined(MeshWriter_RECURSES)
#error Recursive header files inclusion detected in MeshWriter.h
#else // defined(MeshWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define MeshWriter_RECURSES

#if !defined MeshWriter_h
/** Prevents repeated inclusion of headers. */
#define MeshWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <boost/static_assert.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class MeshWriter
  /**
   * Description of template struct 'MeshWriter' <p>
   * \brief Aim: Export a Mesh (MeshFromPoints object) in different format as OFF and OBJ).
   * 
   * The exportation can be done automatically according the input file
   * extension with the ">>" operator  
   * 
   * Example of typical use: 
   * First you have to add the following include files:
   * @snippet tests/io/readers/testMeshWriter.cpp MeshWriterUseIncludes
   * Then you create a simple MeshFromPoints object:
   * @snippet tests/io/readers/testMeshWriter.cpp MeshWriterUseMeshCreation
   * Finally you can export directly the MeshFromPoints object: 
   * @snippet tests/io/readers/testMeshWriter.cpp MeshWriterUseMeshExport
   *
   *
   * @see MeshFromPoints MeshWriter
   *
   *
   */
  template <typename TPoint>
  struct MeshWriter
  {
    // ----------------------- Standard services ------------------------------

   


    /** 
     * Export MeshFromPoints towards a OFF format.
     * 
     * @param out the output stream of the exported OFF object.
     * @param aMesh the MeshFromPoints object to be exported.
     * @param exportColor true to export colors (default false). 
     * @return true if no errors occur.
     */
    
    static bool export2OFF(std::ostream &out, const  MeshFromPoints<TPoint>  &aMesh, 
      bool exportColor=false) throw(DGtal::IOException);
  
  


    /** 
     * Export a MeshFromPoints towards a OBJ format.
     * 
     * @param out the output stream of the exported OBJ object.
     * @param aMesh the MeshFromPoints object to be exported.
     * @return true if no errors occur.
     */
    
    static bool export2OBJ(std::ostream &out, const  MeshFromPoints<TPoint>  &aMesh) throw(DGtal::IOException);
       
    
  };
  




  /**
   *  'operator>>' for exporting objects of class 'MeshFromPoints'.
   *  This operator automatically selects the good method according to
   *  the filename extension (off, obj).
   *  
   * @param aMesh the mesh to be exported.
   * @param aFilename the filename of the file to be exported. 
   * @return true, if the export was successful. 
   */
  template <typename TPoint>
  bool
  operator >> (  MeshFromPoints<TPoint> & aMesh,  const std::string & aFilename  );
  




  /**
   *  'operator>>' for exporting objects of class 'MeshFromPoints' in OFF format.
   *  
   * @param aMesh the mesh to be exported.
   * @param out the output of the OFF exportation. 
   * @return true, if the export was successful. 
   */
  template <typename TPoint>
  bool
  operator >> (  MeshFromPoints<TPoint> & aMesh, std::ostream &out );
  



  

			   
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/MeshWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined MeshWriter_h

#undef MeshWriter_RECURSES
#endif // else defined(MeshWriter_RECURSES)
