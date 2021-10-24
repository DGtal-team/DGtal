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
 * @file CorrectedNormalCurrentComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/18
 *
 * Header file for module CorrectedNormalCurrentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CorrectedNormalCurrentComputer_RECURSES)
#error Recursive header files inclusion detected in CorrectedNormalCurrentComputer.h
#else // defined(CorrectedNormalCurrentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CorrectedNormalCurrentComputer_RECURSES

#if !defined CorrectedNormalCurrentComputer_h
/** Prevents repeated inclusion of headers. */
#define CorrectedNormalCurrentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/meshes/SurfaceMeshMeasure.h"
#include "DGtal/geometry/meshes/CorrectedNormalCurrentFormula.h"
#include "DGtal/shapes/SurfaceMesh.h"


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class CorrectedNormalCurrentComputer
  /**
     Description of template class 'CorrectedNormalCurrentComputer'
     <p> \brief Aim: Utility class to compute curvature measures
     induced by (1) a corrected normal current defined by a surface
     mesh with prescribed normals and (2) the standard
     Lipschitz-Killing invariant forms of area and curvatures.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct CorrectedNormalCurrentComputer
  {
    typedef TRealPoint                                   RealPoint;
    typedef TRealVector                                  RealVector;
    typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );
    typedef DGtal::SurfaceMesh< RealPoint, RealVector >  SurfaceMesh;
    typedef typename RealVector::Component               Scalar;
    typedef SimpleMatrix< Scalar, dimension, dimension > RealTensor;
    typedef CorrectedNormalCurrentFormula< RealPoint, RealVector > Formula;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, Scalar >     ScalarMeasure;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, RealTensor > TensorMeasure;
    typedef std::vector< Scalar >                        Scalars;
    typedef std::vector< RealPoint >                     RealPoints;
    typedef std::vector< RealVector >                    RealVectors;
    typedef std::vector< RealTensor >                    RealTensors;
    
    /// The type for counting elements.
    typedef typename SurfaceMesh::Size                  Size;
    /// The type used for numbering vertices
    typedef typename SurfaceMesh::Size                  Index;
    typedef typename SurfaceMesh::Vertex                Vertex;
    typedef typename SurfaceMesh::Face                  Face;


    /// Constructor from mesh.
    /// @param aMesh any simplified mesh that is referenced in this object.
    CorrectedNormalCurrentComputer( ConstAlias< SurfaceMesh > aMesh,
                                    bool unit_u = false );

    /// @return the \f$ \mu_0 \f$ corrected curvature measure, i.e. the area measure.
    ScalarMeasure computeMu0() const;
    /// @return the \f$ \mu_1 \f$ corrected curvature measure,
    /// i.e. twice the mean curvature measure.
    ScalarMeasure computeMu1() const;
    /// @return the \f$ \mu_2 \f$ corrected curvature measure,
    /// i.e. the Gaussian curvature measure.
    ScalarMeasure computeMu2() const;
    /// @return the \f$ \mu^{X,Y} \f$ corrected curvature measure,
    /// i.e. the anisotropic tensor curvature measure.
    TensorMeasure computeMuXY() const;

    // ------------------------- Public Datas ------------------------------
  public:
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// A reference to the mesh over which computations are done.
    const SurfaceMesh& myMesh;
    /// Tells if we should use a normalized unit vector or not for
    /// interpolated curvature measures.
    bool myUnitU;
    
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  private:
        
  }; // end of class CorrectedNormalCurrentComputer
    
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "CorrectedNormalCurrentComputer.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CorrectedNormalCurrentComputer_h

#undef CorrectedNormalCurrentComputer_RECURSES
#endif // else defined(CorrectedNormalCurrentComputer_RECURSES)

