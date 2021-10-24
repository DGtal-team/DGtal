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
 * @file NormalCycleComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/18
 *
 * Header file for module NormalCycleComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NormalCycleComputer_RECURSES)
#error Recursive header files inclusion detected in NormalCycleComputer.h
#else // defined(NormalCycleComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NormalCycleComputer_RECURSES

#if !defined NormalCycleComputer_h
/** Prevents repeated inclusion of headers. */
#define NormalCycleComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/geometry/meshes/SurfaceMeshMeasure.h"
#include "DGtal/geometry/meshes/NormalCycleFormula.h"
#include "DGtal/shapes/SurfaceMesh.h"


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class NormalCycleComputer
  /**
     Description of template class 'NormalCycleComputer' <p> \brief
     Aim: Utility class to compute curvatures measures induced by (1)
     the normal cycle induced by a SurfaceMesh, (2) the standard
     Lipschitz-Killing invariant forms of area and curvatures.

     @warning Most curvature measures induces by the normal cycle have
     meaning only for surfaces with planar faces.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct NormalCycleComputer
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef NormalCycleComputer< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );
    typedef DGtal::SurfaceMesh< RealPoint, RealVector >  SurfaceMesh;
    typedef typename RealVector::Component               Scalar;
    typedef SimpleMatrix< Scalar, dimension, dimension > RealTensor;
    typedef NormalCycleFormula< RealPoint, RealVector >  Formula;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, Scalar >     ScalarMeasure;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, RealTensor > TensorMeasure;
    typedef std::vector< Scalar >                        Scalars;
    typedef std::vector< RealPoint >                     RealPoints;
    typedef std::vector< RealVector >                    RealVectors;
    typedef std::vector< RealTensor >                    RealTensors;
    typedef typename SurfaceMesh::Size                   Size; 
    typedef typename SurfaceMesh::Index                  Index;
    typedef typename SurfaceMesh::Vertex                 Vertex;
    typedef typename SurfaceMesh::Face                   Face;

    
    /// Constructor from mesh.
    /// @param aMesh any simplified mesh that is referenced in this object.
    NormalCycleComputer( ConstAlias< SurfaceMesh > aMesh );

    /// @return the \f$ \mu_0 \f$ normal cycle measure, i.e. the area measure.
    ScalarMeasure computeMu0() const;
    /// @return the \f$ \mu_1 \f$ normal cycle measure,
    /// i.e. twice the mean curvature measure.
    ScalarMeasure computeMu1() const;
    /// @return the \f$ \mu_2 \f$ normal cycle measure,
    /// i.e. the Gaussian curvature measure.
    ScalarMeasure computeMu2() const;
    /// @return the \f$ \mu^{X,Y} \f$ normal cycle measure,
    /// i.e. the anisotropic tensor curvature measure.
    TensorMeasure computeMuXY() const;
    /// @return the \f$ \tilde{\mu}^{X,Y} \f$ normal cycle measure,
    /// i.e. the anisotropic tensor curvature measure with swapped eigenvectors.
    TensorMeasure computeMuXYs() const;
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// A reference to the mesh over which computations are done.
    const SurfaceMesh& myMesh;
    
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:
    
    
  }; // end of class NormalCycleComputer
    
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "NormalCycleComputer.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NormalCycleComputer_h

#undef NormalCycleComputer_RECURSES
#endif // else defined(NormalCycleComputer_RECURSES)

