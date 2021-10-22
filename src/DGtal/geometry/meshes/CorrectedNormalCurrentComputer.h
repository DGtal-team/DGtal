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
// always include EigenSupport.h before any other Eigen headers
// #include "DGtal/math/linalg/EigenSupport.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "SimplifiedMesh.h"
#include "CorrectedNormalCurrentFormula.h"


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class CorrectedNormalCurrentComputer
  /**
     Description of template class 'CorrectedNormalCurrentComputer'
     <p> \brief Aim: Utility class to compute corrected normal current
     over a simplified mesh.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct CorrectedNormalCurrentComputer
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );

    typedef DGtal::SimplifiedMesh< RealPoint, RealVector > SimplifiedMesh;
    typedef typename RealVector::Component Scalar;
    typedef std::vector< Scalar >          Scalars;
    typedef std::vector< RealPoint >       RealPoints;
    typedef std::vector< RealVector >      RealVectors;
    typedef CorrectedNormalCurrentFormula< RealPoint, RealVector > Formula;
    typedef typename Formula::RealTensor   RealTensor;
    typedef std::vector< RealTensor >      RealTensors;
    
    /// The type for counting elements.
    typedef std::size_t                             Size;
    /// The type used for numbering vertices
    typedef std::size_t                             Index;
    typedef Index                                   Face;
    typedef std::pair< Face, Scalar >               WeightedFace;
    typedef Index                                   Vertex;
    typedef std::pair< Vertex, Scalar >             WeightedVertex;
    /// The type that defines a range of vertices
    typedef std::vector< Vertex >                   Vertices;
    /// The type that defines a range of faces
    typedef std::vector< Face >                     Faces;
    typedef std::vector< WeightedFace >             WeightedFaces;

    enum class Measure { MU0, MU1, MU2, MUXY, ALL_MU };

    /// Constructor from mesh.
    /// @param aMesh any simplified mesh that is referenced in this object.
    CorrectedNormalCurrentComputer( ConstAlias< SimplifiedMesh > aMesh );

    bool computeInterpolatedMeasures( Measure mu, bool unit_u = false );
    bool computeInterpolatedMu0     ( bool unit_u = false );
    bool computeInterpolatedMu1     ( bool unit_u = false );
    bool computeInterpolatedMu2     ( bool unit_u = false );
    bool computeInterpolatedMuXY    ( bool unit_u = false );

    /// Given weighted faces, returns its interpolated mu0 measure.
    Scalar     interpolatedMu0 ( const WeightedFaces& wfaces ) const;
    /// Given weighted faces, returns its interpolated mu1 measure.
    Scalar     interpolatedMu1 ( const WeightedFaces& wfaces ) const;
    /// Given weighted faces, returns its interpolated mu2 measure.
    Scalar     interpolatedMu2 ( const WeightedFaces& wfaces ) const;
    /// Given weighted faces, returns its interpolated muXY measure.
    RealTensor interpolatedMuXY( const WeightedFaces& wfaces ) const;

    // ------------------------- Public Datas ------------------------------
  public:
    /// Per-face mu0 measure (area).
    Scalars     mu0;
    /// Per-face mu1 measure (mean curvature).
    Scalars     mu1;
    /// Per-face mu2 measure (Gaussian curvature).
    Scalars     mu2;
    /// Per-face muXY measure (2nd fundamental form).
    RealTensors muXY;
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    
    /// A reference to the mesh over which computations are done.
    const SimplifiedMesh& myMesh;
    
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

