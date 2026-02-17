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
 * @file SurfaceMeshReader.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for class SurfaceMeshReader
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceMeshReader_RECURSES)
#error Recursive header files inclusion detected in SurfaceMeshReader.h
#else // defined(SurfaceMeshReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceMeshReader_RECURSES

#if !defined SurfaceMeshReader_h
/** Prevents repeated inclusion of headers. */
#define SurfaceMeshReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/SurfaceMesh.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceMeshReader
  /**
     Description of template class 'SurfaceMeshReader' <p> \brief Aim:
     An helper class for reading mesh files (Wavefront OBJ at this point) and creating a SurfaceMesh.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
  */
  template < typename TRealPoint, typename TRealVector >
  struct SurfaceMeshReader
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef SurfaceMeshReader< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );

    typedef DGtal::SurfaceMesh< RealPoint, RealVector > SurfaceMesh;
    typedef typename SurfaceMesh::Size           Size;
    typedef typename SurfaceMesh::Index          Index;
    typedef typename SurfaceMesh::Vertices       Vertices;
    typedef typename SurfaceMesh::Faces          Faces;
    typedef std::vector<int>                     Materials;

    /// Checks that every index in \a indices are different from the others.
    /// @param indices a vector of integer indices
    /// @return 'true' iff the integer indices are all pairwise different.
    static bool verifyIndicesUniqueness( const std::vector< Index > &indices );

    /// Splits a string \a str into several strings according to a
    /// delimiter \a delim.
    /// @param[in] str any string.
    /// @param[in] delim any delimiter character.
    /// @return the vector of split strings.
    static
    std::vector< std::string > split( const std::string& str, char delim = ' ');

    /// Reads an input file as an OBJ file format and outputs the
    /// corresponding surface mesh.
    ///
    /// @param[in,out] input the input stream where the OBJ file is read.
    /// @param[out] smesh the output surface mesh.
    ///
    /// @return 'true' if both reading the input stream was ok and the
    /// created mesh is ok.
    static
    bool readOBJ( std::istream & input, SurfaceMesh & smesh );

    /// Reads an input file as an OBJ file format and outputs the
    /// corresponding surface mesh.
    ///
    /// @param[in,out] input the input stream from which the OBJ file is read.
    /// @param[out] smesh the output surface mesh.
    /// @param[out] materials a vector containing the material of each face (an index), or an empty vector if there is no material in the OBJ file.
    ///
    /// @return 'true' if both reading the input stream was ok and the
    /// created mesh is ok.
    static
    bool readOBJ( std::istream & input, SurfaceMesh & smesh, Materials& materials );
  };

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "SurfaceMeshReader.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceMeshReader_h

#undef SurfaceMeshReader_RECURSES
#endif // else defined(SurfaceMeshReader_RECURSES)
